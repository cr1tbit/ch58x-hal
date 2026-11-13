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
    // SAFETY: PFIC_SCTLR is always accessible. Other bits are written back unchanged; SETEVENT
    // is write-only and self-clearing.
    unsafe { PFIC_SCTLR.write_volatile(PFIC_SCTLR.read_volatile() | SETEVENT) };
}

/// Thread-mode executor for CH58x. See the [module docs](self).
pub struct Executor {
    inner: raw::Executor,
    not_send: PhantomData<*mut ()>,
}

impl Executor {
    pub fn new() -> Self {
        Self {
            inner: raw::Executor::new(core::ptr::null_mut()),
            not_send: PhantomData,
        }
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

        init(this.inner.spawner());
        loop {
            // SAFETY: only this thread polls the executor, never from the pender.
            unsafe { this.inner.poll() };
            // BLE stack: runs after every interrupt and task wake-up; if it still has work,
            // poll again instead of sleeping (see `ble::stack`).
            #[cfg(feature = "ble")]
            if !crate::ble::stack::process() {
                continue;
            }
            // WFE: returns at once if an event was raised since the last WFE, see module docs.
            unsafe { core::arch::asm!("wfi") };
        }
    }
}

impl Default for Executor {
    fn default() -> Self {
        Self::new()
    }
}
