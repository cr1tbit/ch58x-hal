//! Boot-time LSI calibration for boards without a 32.768 kHz crystal.
//!
//! With the internal oscillator as 32k clock source, the BLE stack runs on a 32000 Hz
//! timebase, but nothing calibrates the oscillator. Call [`calibrate_lsi`] after
//! `embassy::init()` and before [`init`](super::init), while the 32 MHz crystal provides
//! the system clock reference. Do not retune the oscillator during a BLE connection.

use embedded_hal_1::delay::DelayNs;

use crate as hal;
use crate::delay::CycleDelay;
use crate::rtc::Rtc;

const TARGET_HZ: u32 = 32_000;
const MAX_ERROR_HZ: u32 = 16; // 500 ppm

#[derive(Debug)]
pub enum LsiCalibrationError {
    ClockNotReady,
    OscillatorStopped,
    OutOfRange,
}

/// Tune the internal oscillator to the HAL's 32000 Hz BLE timebase.
///
/// Uses the Embassy SysTick counter as a crystal-derived HCLK/8 reference. This blocks
/// briefly at startup and leaves SysTick configuration and the RTC counter unchanged.
/// The original trim is restored if calibration fails.
pub fn calibrate_lsi() -> Result<u32, LsiCalibrationError> {
    let sys = unsafe { &*hal::pac::SYS::PTR };
    let systick = unsafe { &*hal::pac::SYSTICK::PTR };
    let ctlr = systick.ctlr().read();
    if !ctlr.ste().bit_is_set()
        || !ctlr.stclk().is_hclk_div8()
        || sys.ck32k_config().read().clk_osc32k_xt().bit_is_set()
    {
        return Err(LsiCalibrationError::ClockNotReady);
    }
    let reference_hz = hal::sysctl::clocks().hclk.to_Hz() as u64 / 8;
    let rtc = Rtc {};
    // Only use the low 16-bit counter and wrapping subtraction. A sample lasts far
    // less than one rollover, and avoids a torn read of the RTC's separate counters.
    let rtc_tick = || rtc.counter_tick() as u16;
    let measure = || -> Result<u32, LsiCalibrationError> {
        // Align the start to an RTC edge; count 256 complete oscillator periods.
        let deadline = systick.cnt().read().bits() + reference_hz / 10;
        let edge = rtc_tick();
        while rtc_tick() == edge {
            if systick.cnt().read().bits() >= deadline {
                return Err(LsiCalibrationError::OscillatorStopped);
            }
        }
        let start = rtc_tick();
        let hstart = systick.cnt().read().bits();
        while rtc_tick().wrapping_sub(start) < 256 {
            if systick.cnt().read().bits() >= deadline {
                return Err(LsiCalibrationError::OscillatorStopped);
            }
        }
        let hend = systick.cnt().read().bits();
        let end = rtc_tick();
        Ok((end.wrapping_sub(start) as u64 * reference_hz / (hend - hstart)) as u32)
    };
    let set_trim = |coarse: u16, fine: u16| {
        hal::safe_write(sys.int32k_tune(), |w| unsafe { w.bits((fine << 5) | coarse) });
        CycleDelay.delay_ms(1);
    };
    let original = sys.int32k_tune().read().bits();
    let result = (|| {
        // Low five bits are coarse trim; the upper eight bits are fine trim.
        // Search coarse trim with fine trim centred, then search the fine range.
        let mut low = 0u16;
        let mut high = 31u16;
        while low < high {
            let mid = (low + high) / 2;
            set_trim(mid, 128);
            if measure()? < TARGET_HZ {
                low = mid + 1;
            } else {
                high = mid;
            }
        }
        let coarse = low;
        low = 0;
        high = 255;
        while low < high {
            let mid = (low + high) / 2;
            set_trim(coarse, mid);
            if measure()? < TARGET_HZ {
                low = mid + 1;
            } else {
                high = mid;
            }
        }
        set_trim(coarse, low);
        let upper_hz = measure()?;
        // Choose the nearer of the two fine settings around the target.
        if low > 0 {
            set_trim(coarse, low - 1);
            if measure()?.abs_diff(TARGET_HZ) > upper_hz.abs_diff(TARGET_HZ) {
                set_trim(coarse, low);
            }
        }
        let hz = measure()?;
        if hz.abs_diff(TARGET_HZ) > MAX_ERROR_HZ {
            return Err(LsiCalibrationError::OutOfRange);
        }
        Ok(hz)
    })();
    if result.is_err() {
        hal::safe_write(sys.int32k_tune(), |w| unsafe { w.bits(original) });
    }
    result
}
