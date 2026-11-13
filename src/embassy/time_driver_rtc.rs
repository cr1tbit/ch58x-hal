//! RTC-based time driver (feature `time-driver-rtc`): embassy time keeps running while the chip
//! sleeps, unlike SysTick, which stops with the system clock.
//!
//! Ticks are the RTC's 32 kHz count (`rtc_wake::now_ticks`, 30.5 µs), which matches the HAL's
//! `tick-hz-32_768`. Alarms use the RTC trigger, which is shared with the BLE stack: this driver
//! only keeps the queue, and [`crate::embassy::Executor`] arms the trigger before sleeping for
//! the earliest of [`next_alarm`] and the BLE deadline. It therefore needs that executor.
//!
//! The tick rate is that of the 32 kHz clock: 32768 Hz on the crystal (LSE), but ~32000 Hz on
//! the internal oscillator (LSI), where embassy time runs ~2.3 % slow.

use core::cell::RefCell;
use core::task::Waker;

use critical_section::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

use crate::rtc_wake;

pub struct RtcDriver {
    queue: Mutex<RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: RtcDriver = RtcDriver {
    queue: Mutex::new(RefCell::new(Queue::new())),
});

impl Driver for RtcDriver {
    fn now(&self) -> u64 {
        rtc_wake::now_ticks()
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        // Only queued here: the executor arms the trigger before it sleeps. Timers are polled
        // from tasks (thread mode), so the executor always runs after this.
        critical_section::with(|cs| {
            self.queue.borrow_ref_mut(cs).schedule_wake(at, waker);
        });
    }
}

/// Wake the tasks whose timers expired and return the next alarm time (`None`: no timers).
/// Called by the executor before it sleeps.
pub(crate) fn next_alarm() -> Option<u64> {
    critical_section::with(|cs| {
        let next = DRIVER.queue.borrow_ref_mut(cs).next_expiration(rtc_wake::now_ticks());
        (next != u64::MAX).then_some(next)
    })
}

pub(crate) fn init() {
    rtc_wake::enable();
}
