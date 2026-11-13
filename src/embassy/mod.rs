#[cfg(not(feature = "time-driver-rtc"))]
pub mod time_driver_systick;
#[cfg(feature = "time-driver-rtc")]
pub mod time_driver_rtc;
#[cfg(feature = "time-driver-rtc")]
pub mod deep_sleep;

#[cfg(feature = "executor")]
pub mod executor;
#[cfg(feature = "executor")]
pub use executor::Executor;

// This should be called after global clocks inited
pub fn init() {
    #[cfg(not(feature = "time-driver-rtc"))]
    time_driver_systick::init();
    #[cfg(feature = "time-driver-rtc")]
    time_driver_rtc::init();

    unsafe {
        crate::gpio::init();
    }
}
