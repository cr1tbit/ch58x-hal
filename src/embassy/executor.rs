//! Thread-mode embassy executor that sleeps between polls without losing wake-ups.
//!
//! Use it instead of embassy-executor's `arch-riscv32` executor (do not enable that feature: it
//! defines its own `__pender`):
//!
//! ```ignore
//! #[qingke_rt::entry]
//! fn main() -> ! {
//!     hal::embassy::Executor::new().run(|spawner| spawner.must_spawn(app(spawner)))
//! }
//! ```
//!
//! # Why not the `arch-riscv32` executor
//!
//! It masks interrupts, checks a "work pending" flag and then sleeps with `wfi`, which qingke-rt
//! turns into WFE. On QingKe a masked WFE wakes only on a *new* pending interrupt. An interrupt
//! that became pending just before the sleep doesn't count, and while it stays pending, the same
//! interrupt can't become pending again. USB enumeration then stalls until some other interrupt
//! happens to arrive. Measured on a CH582F: 0 of 6 enumerations with that executor.
//!
//! # How this one sleeps
//!
//! It polls, then sleeps in WFE with interrupts *enabled*. QingKe has an event register like
//! Cortex-M: an event raised while the CPU is awake is kept, and the next WFE returns at once.
//! - An interrupt that fires between the poll and the WFE is taken right away, and with
//!   SEVONPEND its pending state also raises the event, so the WFE doesn't sleep.
//! - A task woken from thread mode (by another task) calls the pender, which raises the event
//!   with `PFIC_SCTLR.SETEVENT`.
//! - An interrupt during the sleep wakes the CPU and runs its handler first.
//!
//! Measured on a CH582F: SETEVENT then WFE returns after 3 SysTick counts, a second WFE then
//! sleeps until the next interrupt. With BLE running (TMOS polled every 300 us) the loop runs
//! about 2900 times per second, against about 635000 when polled continuously.

use core::marker::PhantomData;

use embassy_executor::{raw, Spawner};

const PFIC_SCTLR: *mut u32 = 0xE000_ED10 as *mut u32;
const SETEVENT: u32 = 1 << 5;
const SEVONPEND: u32 = 1 << 4;
const WFITOWFE: u32 = 1 << 3;

/// Called by embassy when a task is woken, from any context: makes the next WFE return at once.
#[export_name = "__pender"]
fn pender(_context: *mut ()) {
    #[cfg(feature = "time-driver-rtc")]
    super::deep_sleep::WOKEN.store(true, core::sync::atomic::Ordering::Relaxed);
    // SAFETY: PFIC_SCTLR is always accessible. Other bits are written back unchanged; SETEVENT
    // is write-only and self-clearing.
    unsafe { PFIC_SCTLR.write_volatile(PFIC_SCTLR.read_volatile() | SETEVENT) };
}

/// Thread-mode executor for CH58x. See the [module docs](self).
pub struct Executor {
    inner: raw::Executor,
    deep_sleep: bool,
    not_send: PhantomData<*mut ()>,
}

impl Executor {
    pub fn new() -> Self {
        Self {
            inner: raw::Executor::new(core::ptr::null_mut()),
            deep_sleep: false,
            not_send: PhantomData,
        }
    }

    /// Put the chip into sleep mode (32 MHz crystal, PLL, flash and peripheral clocks off)
    /// whenever the next deadline is far enough away, see [`super::deep_sleep`]. Only for
    /// firmware that needs no clocked peripheral (UART receive, USB, PWM, timers) in between.
    #[cfg(feature = "time-driver-rtc")]
    pub fn with_deep_sleep(mut self) -> Self {
        self.deep_sleep = true;
        self
    }

    /// Spawn the initial tasks with `init`, then poll forever, sleeping while there is no work.
    ///
    /// Interrupts must be enabled (they are after `qingke_rt::entry`), or the CPU never wakes up.
    pub fn run(self, init: impl FnOnce(Spawner)) -> ! {
        // SAFETY: this function never returns (and panics abort), so `self` lives in this stack
        // frame forever and is never moved again.
        let this: &'static Self = unsafe { &*(&self as *const Self) };

        // WFI behaves as WFE, and every pending interrupt raises an event (also set by
        // `hal::init`, repeated here so the executor doesn't depend on call order).
        critical_section::with(|_| unsafe {
            PFIC_SCTLR.write_volatile(PFIC_SCTLR.read_volatile() | WFITOWFE | SEVONPEND)
        });

        #[cfg(feature = "time-driver-rtc")]
        super::deep_sleep::ENABLED.store(this.deep_sleep, core::sync::atomic::Ordering::Relaxed);

        init(this.inner.spawner());
        loop {
            #[cfg(feature = "time-driver-rtc")]
            super::deep_sleep::WOKEN.store(false, core::sync::atomic::Ordering::Relaxed);
            // SAFETY: only this thread polls the executor, never from the pender.
            unsafe { this.inner.poll() };
            // a completed pass: the firmware isn't stuck (see `wdog`)
            crate::wdog::feed_if_enabled();
            let Some(wake_at) = prepare_sleep() else {
                continue; // more work
            };
            // With BLE running, the BLE stack's sleep callback sleeps (see `ble::stack`).
            #[cfg(feature = "time-driver-rtc")]
            if super::deep_sleep::is_enabled() && !ble_started() {
                if let Some(at) = wake_at {
                    if super::deep_sleep::sleep_until(at) {
                        // woke early for the crystal: poll, then idle until the deadline
                        continue;
                    }
                }
            }
            #[cfg(any(feature = "ble", feature = "time-driver-rtc"))]
            if let Some(at) = wake_at {
                if !crate::rtc_wake::arm(at) {
                    continue; // deadline too close to sleep
                }
            }
            let _ = wake_at;
            // WFE: returns at once if an event was raised since the last WFE, see module docs.
            unsafe { core::arch::asm!("wfi") };
        }
    }
}

#[cfg(feature = "time-driver-rtc")]
fn ble_started() -> bool {
    #[cfg(feature = "ble")]
    return crate::ble::stack::is_started();
    #[cfg(not(feature = "ble"))]
    return false;
}

/// Run the BLE stack and the RTC timer queue. Returns `None` if there is more work (don't
/// sleep), otherwise the earliest deadline to wake up for (`None` inside: no deadline).
fn prepare_sleep() -> Option<Option<u64>> {
    #[allow(unused_mut)]
    let mut wake_at: Option<u64> = None;
    // BLE stack: runs after every interrupt and task wake-up (see `ble::stack`)
    #[cfg(feature = "ble")]
    match crate::ble::stack::process() {
        crate::ble::stack::Next::Busy => return None,
        crate::ble::stack::Next::IdleUntil(at) => wake_at = at,
    }
    // Expired timers wake their tasks here; the pender then makes the WFE return at once.
    #[cfg(feature = "time-driver-rtc")]
    if let Some(at) = crate::embassy::time_driver_rtc::next_alarm() {
        wake_at = Some(wake_at.map_or(at, |w| w.min(at)));
    }
    Some(wake_at)
}

impl Default for Executor {
    fn default() -> Self {
        Self::new()
    }
}
