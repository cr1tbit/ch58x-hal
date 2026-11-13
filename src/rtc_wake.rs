//! Waking the CPU with the RTC trigger, shared by the BLE stack (`ble::stack`) and the RTC time
//! driver (`embassy::time_driver_rtc`).
//!
//! The RTC counts 32 kHz ticks in `R32_RTC_CNT_32K` (0 .. [`DAY_TICKS`], then the day counter
//! advances). The trigger (`R32_RTC_TRIG`) raises the RTC interrupt when the count *equals* it,
//! which also wakes the chip from sleep. There is only one trigger, so the executor arms it once
//! per idle pass for the earliest of all deadlines ([`arm`]).
//!
//! Writing the RTC time (`rtc::Rtc::set_datatime`) moves this time base: do it before BLE and
//! timers are in use.

use crate::pac;
use crate::safe_access::safe_modify;

/// 32 kHz ticks per RTC day (`RTC_MAX_COUNT` in the WCH SDK); the count wraps to 0 here.
pub const DAY_TICKS: u32 = 0xA8C0_0000;

/// The RTC's 32 kHz count within the current day (`R32_RTC_CNT_32K`).
pub fn count() -> u32 {
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

/// Monotonic 32 kHz ticks: day counter × [`DAY_TICKS`] + count.
pub fn now_ticks() -> u64 {
    let rtc = unsafe { &*pac::RTC::PTR };
    loop {
        let day = rtc.cnt_day().read().cnt_day().bits();
        let ticks = count();
        if rtc.cnt_day().read().cnt_day().bits() == day {
            return day as u64 * DAY_TICKS as u64 + ticks as u64;
        }
    }
}

/// Ticks from `now` until `then` (both RTC counts), across the daily wrap.
pub fn ticks_until(now: u32, then: u32) -> u32 {
    if then >= now {
        then - now
    } else {
        then + (DAY_TICKS - now)
    }
}

/// The RTC count `ticks` (monotonic, see [`now_ticks`]) corresponds to.
pub fn count_at(ticks: u64) -> u32 {
    (ticks % DAY_TICKS as u64) as u32
}

/// Enable the trigger and its interrupt (idempotent).
pub(crate) fn enable() {
    let rtc = unsafe { &*pac::RTC::PTR };
    let sys = unsafe { &*pac::SYS::PTR };
    // the RTC may wake the chip from sleep mode
    safe_modify(sys.slp_wake_ctrl(), |_, w| w.slp_rtc_wake().set_bit());
    if !rtc.mode_ctrl().read().trig_en().bit_is_set() {
        rtc.flag_ctrl().modify(|_, w| w.trig_clr().set_bit());
        safe_modify(rtc.mode_ctrl(), |_, w| w.trig_en().set_bit());
    }
    unsafe { qingke::pfic::enable_interrupt(pac::Interrupt::RTC as u8) };
}

/// Arm the trigger for monotonic time `at`. Returns `false` if `at` is less than two ticks
/// ahead or passed while arming: the caller must not sleep then (the trigger fires only when
/// the count equals it, so a missed value would not fire for a whole day).
pub(crate) fn arm(at: u64) -> bool {
    let now = now_ticks();
    if at < now + 2 {
        return false;
    }
    let rtc = unsafe { &*pac::RTC::PTR };
    let target = count_at(at);
    safe_modify(rtc.trig(), |_, w| unsafe { w.bits(target) });
    now_ticks() < at
}

/// RTC interrupt: the trigger or the periodic timer fired. Waking the CPU is all it has to do;
/// the executor then runs the BLE stack and the expired timers.
#[qingke_rt::interrupt]
fn RTC() {
    let rtc = unsafe { &*pac::RTC::PTR };
    rtc.flag_ctrl().write(|w| w.trig_clr().set_bit().tmr_clr().set_bit());
}
