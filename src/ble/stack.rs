//! Runs the BLE stack (TMOS) only when it has work, instead of polling it on a fixed tick.
//!
//! [`crate::embassy::Executor`] calls [`process`] after every poll, i.e. after every interrupt
//! (the radio's interrupts included) and every task wake-up, so the stack sees new radio events
//! and API calls right away. When TMOS has nothing left to do, the BLE library reports the RTC
//! time of its next event through the sleep callback (`bleConfig_t::sleepCB`). [`process`]
//! arms the RTC trigger for that time, and the executor sleeps until then or until another
//! interrupt.
//!
//! Measured on a CH582F (1 s advertising): the old 300 µs polling woke the CPU ~3300 times per
//! second, while TMOS was reporting its next event 0.5 s ahead on average.

use core::cell::Cell;
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::Mutex;

use super::ffi::TMOS_SystemProcess;
use crate::pac;
use crate::safe_access::safe_modify;

/// `RTC_MAX_COUNT` in the WCH SDK: the 32 kHz counter wraps to 0 here (one day).
const RTC_MAX_COUNT: u32 = 0xA8C0_0000;
/// Don't arm the trigger for less than this many 32 kHz ticks ahead: the counter must not
/// pass the trigger value before it is written. Process again right away instead.
const MIN_TICKS_AHEAD: u32 = 2;

static STARTED: AtomicBool = AtomicBool::new(false);
/// Set by the sleep callback during `TMOS_SystemProcess`: TMOS is idle until this RTC count.
static NEXT_EVENT: Mutex<Cell<Option<u32>>> = Mutex::new(Cell::new(None));

/// The BLE library's sleep callback: remember when TMOS needs to run next. Returns non-zero,
/// i.e. "did not sleep" (the WCH SDK's `CH58X_LowPower` returns 0 after sleeping).
pub(super) unsafe extern "C" fn on_idle(next_event: u32) -> u32 {
    critical_section::with(|cs| NEXT_EVENT.borrow(cs).set(Some(next_event)));
    1
}

/// Called at the end of `ble::init`: TMOS may run from now on, the RTC trigger wakes the CPU.
pub(super) fn start() {
    let rtc = unsafe { &*pac::RTC::PTR };
    rtc.flag_ctrl().modify(|_, w| w.trig_clr().set_bit());
    safe_modify(rtc.mode_ctrl(), |_, w| w.trig_en().set_bit());
    unsafe { qingke::pfic::enable_interrupt(pac::Interrupt::RTC as u8) };
    STARTED.store(true, Ordering::Release);
}

/// The RTC's 32 kHz counter (`R32_RTC_CNT_32K`), the BLE library's time base.
pub fn rtc_count() -> u32 {
    let rtc = unsafe { &*pac::RTC::PTR };
    // the 32-bit register spans CNT_32K and CNT_2S; read until two reads agree
    let reg = rtc.cnt_32k() as *const _ as *const u32;
    loop {
        let a = unsafe { reg.read_volatile() };
        if a == unsafe { reg.read_volatile() } {
            return a;
        }
    }
}

/// Ticks from `now` until `then`, across the counter's wrap at [`RTC_MAX_COUNT`].
fn ticks_until(now: u32, then: u32) -> u32 {
    if then >= now {
        then - now
    } else {
        then + (RTC_MAX_COUNT - now)
    }
}

/// Run TMOS once. Returns `true` if it is idle and the RTC trigger is armed for its next
/// event, so the CPU may sleep; `false` if it has more work and must run again right away.
/// Does nothing (returns `true`) before `ble::init`. Thread mode only.
///
/// [`crate::embassy::Executor`] calls this; call it yourself only with your own executor loop.
pub fn process() -> bool {
    if !STARTED.load(Ordering::Acquire) {
        return true;
    }
    critical_section::with(|cs| NEXT_EVENT.borrow(cs).set(None));
    unsafe { TMOS_SystemProcess() };
    let Some(next_event) = critical_section::with(|cs| NEXT_EVENT.borrow(cs).get()) else {
        return false; // still busy
    };
    if ticks_until(rtc_count(), next_event) < MIN_TICKS_AHEAD {
        return false;
    }
    let rtc = unsafe { &*pac::RTC::PTR };
    safe_modify(rtc.trig(), |_, w| unsafe { w.bits(next_event) });
    // The deadline may have passed while the trigger was written: run again instead of
    // sleeping past it (the trigger fires only when the counter equals it).
    ticks_until(rtc_count(), next_event) >= 1 && ticks_until(rtc_count(), next_event) < RTC_MAX_COUNT / 2
}

/// RTC interrupt: the trigger (TMOS deadline) or the periodic timer fired. Waking the CPU is
/// all it has to do; the executor then runs [`process`].
#[qingke_rt::interrupt]
fn RTC() {
    let rtc = unsafe { &*pac::RTC::PTR };
    rtc.flag_ctrl().write(|w| w.trig_clr().set_bit().tmr_clr().set_bit());
}
