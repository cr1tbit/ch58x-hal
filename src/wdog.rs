//! Watchdog timer and reset cause.
//!
//! The watchdog is an 8-bit counter clocked at hclk / 131072 (`R8_WDOG_COUNT`); with reset
//! enabled, its overflow resets the chip. Timeout = 256 × 131072 / hclk: 1.68 s at 20 MHz,
//! 0.56 s at 60 MHz.
//!
//! It counts only while the system clock runs, so it doesn't count in sleep mode
//! (`embassy::deep_sleep`): the timeout is awake time, not wall time.
//!
//! With [`enable`], `hal::embassy::Executor` feeds it on every pass of its loop, so it resets the
//! chip when the CPU stays awake for the timeout without the executor completing a pass: a task
//! stuck in a loop, a hung interrupt handler, a panic or fatal handler that loops. It doesn't
//! catch a task that waits forever while the executor keeps running.
//!
//! The watchdog is off after any reset, including a watchdog reset; a probe that resets and
//! halts the chip for flashing is not affected.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::pac;
use crate::safe_access::safe_modify;

static ENABLED: AtomicBool = AtomicBool::new(false);

/// Reload the counter (start a new timeout).
#[inline(always)]
pub fn feed() {
    let sys = unsafe { &*pac::SYS::PTR };
    sys.wdog_count().write(|w| unsafe { w.bits(0) });
}

/// Start the watchdog with reset on overflow. From now on the HAL executor feeds it.
pub fn enable() {
    feed();
    let sys = unsafe { &*pac::SYS::PTR };
    safe_modify(sys.rst_wdog_ctrl(), |_, w| w.wdog_rst_en().set_bit());
    ENABLED.store(true, Ordering::Relaxed);
}

/// Feed the watchdog if [`enable`] was called. Called by the executor on every pass.
#[inline(always)]
pub fn feed_if_enabled() {
    if ENABLED.load(Ordering::Relaxed) {
        feed();
    }
}

/// What caused the most recent reset (`R8_RESET_STATUS`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResetCause {
    /// Software reset (`hal::soft_reset`).
    Software,
    /// Power-on.
    PowerOn,
    /// Watchdog timeout.
    Watchdog,
    /// RST pin.
    Pin,
    /// Wake-up from shutdown mode.
    ShutdownWake,
    /// Power-on reset during sleep (e.g. a supply dip).
    PowerOnInSleep,
}

impl ResetCause {
    pub fn name(self) -> &'static str {
        match self {
            ResetCause::Software => "software",
            ResetCause::PowerOn => "power-on",
            ResetCause::Watchdog => "watchdog",
            ResetCause::Pin => "pin",
            ResetCause::ShutdownWake => "shutdown-wake",
            ResetCause::PowerOnInSleep => "power-on-in-sleep",
        }
    }
}

/// The cause of the most recent reset.
pub fn reset_cause() -> ResetCause {
    // R8_RESET_STATUS, bits 0..2
    let flags = unsafe { (0x4000_1044 as *const u8).read_volatile() } & 0x07;
    match flags {
        0 => ResetCause::Software,
        1 => ResetCause::PowerOn,
        2 => ResetCause::Watchdog,
        3 => ResetCause::Pin,
        5 => ResetCause::ShutdownWake,
        _ => ResetCause::PowerOnInSleep,
    }
}
