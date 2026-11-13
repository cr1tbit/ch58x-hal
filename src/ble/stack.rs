//! Runs the BLE stack (TMOS) only when it has work, instead of polling it on a fixed tick.
//!
//! [`crate::embassy::Executor`] calls [`process`] after every poll, i.e. after every interrupt
//! (the radio's interrupts included) and every task wake-up, so the stack sees new radio events
//! and API calls right away. When TMOS has nothing left to do, the BLE library reports the RTC
//! time of its next event through the sleep callback (`bleConfig_t::sleepCB`); the executor
//! arms the RTC trigger for it (`rtc_wake`) and sleeps until then or until another interrupt.
//!
//! Measured on a CH582F (1 s advertising): the old 300 µs polling woke the CPU ~3300 times per
//! second, while TMOS was reporting its next event 0.5 s ahead on average.

use core::cell::Cell;
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::Mutex;

use super::ffi::TMOS_SystemProcess;
use crate::rtc_wake;

static STARTED: AtomicBool = AtomicBool::new(false);

/// Whether `ble::init` ran: from then on the BLE stack's sleep callback does the sleeping.
pub fn is_started() -> bool {
    STARTED.load(Ordering::Acquire)
}
/// Set by the sleep callback during `TMOS_SystemProcess`: TMOS is idle until this RTC count.
static NEXT_EVENT: Mutex<Cell<Option<u32>>> = Mutex::new(Cell::new(None));

/// The BLE library's sleep callback: remember when TMOS needs to run next. Returns non-zero,
/// i.e. "did not sleep" (the WCH SDK's `CH58X_LowPower` returns 0 after sleeping).
pub(super) unsafe extern "C" fn on_idle(next_event: u32) -> u32 {
    critical_section::with(|cs| NEXT_EVENT.borrow(cs).set(Some(next_event)));
    #[cfg(feature = "time-driver-rtc")]
    if crate::embassy::deep_sleep::ENABLED.load(Ordering::Relaxed) {
        return sleep_in_callback(next_event);
    }
    1
}

/// Sleep mode inside the library's sleep callback, as the WCH SDK does (`CH58X_LowPower`):
/// returning 0 tells the library that the chip slept, so it resynchronises its radio timing.
/// `next_event` already includes the crystal start-up margin (`bleConfig_t::WakeUpTime`).
/// Wakes earlier for an embassy timer. Returns 0 if it slept, 2 otherwise (like the SDK).
#[cfg(feature = "time-driver-rtc")]
fn sleep_in_callback(next_event: u32) -> u32 {
    use crate::embassy::deep_sleep::{sleep_then_settle, WAKE_UP_TICKS};
    let now = rtc_wake::now_ticks();
    let ahead = rtc_wake::ticks_until(rtc_wake::count_at(now), next_event);
    if ahead >= rtc_wake::DAY_TICKS / 2 {
        return 2; // already passed
    }
    let ble_wake = now + ahead as u64;
    // expired timers wake their tasks here (and then nothing sleeps)
    let (wake_at, settled_at) = match crate::embassy::time_driver_rtc::next_alarm() {
        Some(timer) if timer.saturating_sub(WAKE_UP_TICKS) < ble_wake => (timer.saturating_sub(WAKE_UP_TICKS), Some(timer)),
        _ => (ble_wake, None),
    };
    if sleep_then_settle(wake_at, settled_at) {
        0
    } else {
        2
    }
}

/// Called at the end of `ble::init`: TMOS may run from now on, the RTC trigger wakes the CPU.
pub(super) fn start() {
    rtc_wake::enable();
    STARTED.store(true, Ordering::Release);
}

/// What the BLE stack needs after [`process`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Next {
    /// More work: run [`process`] again without sleeping.
    Busy,
    /// Idle until this monotonic RTC time (`rtc_wake::now_ticks`), or BLE isn't running (`None`).
    IdleUntil(Option<u64>),
}

/// Run TMOS once (nothing before `ble::init`). Thread mode only.
///
/// [`crate::embassy::Executor`] calls this; call it yourself only with your own executor loop,
/// and arm `rtc_wake` for the returned time before sleeping.
pub fn process() -> Next {
    if !STARTED.load(Ordering::Acquire) {
        return Next::IdleUntil(None);
    }
    critical_section::with(|cs| NEXT_EVENT.borrow(cs).set(None));
    unsafe { TMOS_SystemProcess() };
    match critical_section::with(|cs| NEXT_EVENT.borrow(cs).get()) {
        None => Next::Busy,
        Some(event) => {
            let now = rtc_wake::now_ticks();
            let ahead = rtc_wake::ticks_until(rtc_wake::count_at(now), event);
            // a deadline that already passed shows up as almost a day ahead
            if ahead >= rtc_wake::DAY_TICKS / 2 {
                Next::Busy
            } else {
                Next::IdleUntil(Some(now + ahead as u64))
            }
        }
    }
}
