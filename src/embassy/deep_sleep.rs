//! Sleep mode between events (`Executor::with_deep_sleep`), ported from the WCH SDK
//! (`LowPower_Sleep` in `CH58x_pwr.c`, `CH58X_LowPower` in the BLE examples' `SLEEP.c`).
//!
//! In sleep mode the 32 MHz crystal, the PLL, the flash and the peripheral clocks are off; RAM,
//! the core state, the BLE/USB block registers and the RTC (32 kHz) are kept. The RTC trigger
//! wakes the chip. Everything that needs a clock stops: SysTick, timers, PWM, UART, USB. Use it
//! with the RTC time driver (feature `time-driver-rtc`) and nothing that must run in between.
//!
//! The crystal needs time to start, so the chip wakes [`WAKE_UP_TICKS`] before the deadline and
//! the executor idles (WFE) for the rest, like the SDK.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::rtc_wake;
use crate::safe_access::SafeAccessWord;

/// 32 MHz crystal start-up margin: 1.4 ms (`WAKE_UP_RTC_MAX_TIME` in the SDK).
pub const WAKE_UP_TICKS: u64 = 46;
/// Don't sleep for less than 1 ms (`SLEEP_RTC_MIN_TIME` in the SDK); idle instead.
pub const MIN_SLEEP_TICKS: u64 = 33;

/// Set by the pender: a task was woken since the executor's last poll, so don't sleep.
pub(crate) static WOKEN: AtomicBool = AtomicBool::new(false);

const PFIC_SCTLR: *mut u32 = 0xE000_ED10 as *mut u32;
const SLEEPDEEP: u32 = 1 << 2;

const R8_SLP_POWER_CTRL: *mut u8 = 0x4000_100F as *mut u8;
const RB_RAM_RET_LV: u8 = 0x40;
const R16_POWER_PLAN: *mut u16 = 0x4000_1020 as *mut u16;
const RB_PWR_RAM2K: u16 = 0x02;
const RB_PWR_CORE: u16 = 0x04;
const RB_PWR_EXTEND: u16 = 0x08;
const RB_PWR_RAM30K: u16 = 0x10;
const RB_PWR_DCDC_EN: u16 = 0x0200;
const RB_PWR_DCDC_PRE: u16 = 0x0400;
const RB_PWR_MUST_0010: u16 = 0x1000;
const RB_PWR_PLAN_EN: u16 = 0x8000;
const R8_BAT_DET_CTRL: *mut u8 = 0x4000_1024 as *mut u8;
const R8_PLL_CONFIG: *mut u8 = 0x4000_104B as *mut u8;
const R8_XT32M_TUNE: *mut u8 = 0x4000_104E as *mut u8;
const RB_XT32M_I_BIAS: u8 = 0x03;
const XT32M_BIAS_100: u8 = 0x01;
const XT32M_BIAS_150: u8 = 0x03;

const CMD_GET_ROM_INFO: u8 = 0x06;
const ROM_CFG_MAC_ADDR: u32 = 0x7F018;

/// Number of times the chip slept, for diagnostics.
pub static SLEEPS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// 32 kHz ticks spent in sleep mode (wraps after ~36 h), for diagnostics.
pub static SLEPT_TICKS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Sleep mode on/off: set by `Executor::run` from `with_deep_sleep`, changed by [`set_enabled`].
pub(crate) static ENABLED: AtomicBool = AtomicBool::new(false);

/// Switch sleep mode on or off at runtime (`Executor::with_deep_sleep` sets the start value).
///
/// The debug probe can't attach while the chip sleeps: a debug request doesn't wake it from
/// sleep mode. Firmware that sleeps right after boot can then only be flashed by holding BOOT
/// through a power cycle. Starting without sleep mode for a few seconds after boot, then
/// enabling it here, leaves a window to attach after any reset or power cycle.
pub fn set_enabled(on: bool) {
    ENABLED.store(on, Ordering::Relaxed);
}

/// Whether sleep mode is on.
pub fn is_enabled() -> bool {
    ENABLED.load(Ordering::Relaxed)
}

/// Executor path (no BLE running): sleep until `at` minus [`WAKE_UP_TICKS`]; the executor then
/// idles until `at` while the crystal settles. Returns whether it slept.
pub(crate) fn sleep_until(at: u64) -> bool {
    if at < rtc_wake::now_ticks() + WAKE_UP_TICKS + MIN_SLEEP_TICKS {
        return false;
    }
    sleep_then_settle(at - WAKE_UP_TICKS, None)
}

/// Sleep until `wake_at` (monotonic RTC ticks), then idle until `settled_at` (default:
/// [`WAKE_UP_TICKS`] later) while the 32 MHz crystal starts, like the SDK's `CH58X_LowPower`.
/// Doesn't sleep (returns `false`) if a task was woken, or `wake_at` is too close.
pub(crate) fn sleep_then_settle(wake_at: u64, settled_at: Option<u64>) -> bool {
    if wake_at < rtc_wake::now_ticks() + MIN_SLEEP_TICKS {
        return false;
    }
    wait_uarts_idle();
    if !rtc_wake::arm(wake_at) {
        return false;
    }
    // Interrupts stay masked until the flash is back: their handlers run from flash. A new
    // pending interrupt (the RTC trigger) still ends the WFE thanks to SEVONPEND.
    let before = rtc_wake::now_ticks();
    let slept = critical_section::with(|_| {
        if WOKEN.load(Ordering::Relaxed) {
            return false;
        }
        unsafe { low_power_sleep() };
        true
    });
    if slept {
        let ticks = rtc_wake::now_ticks().saturating_sub(before) as u32;
        SLEEPS.store(SLEEPS.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);
        SLEPT_TICKS.store(SLEPT_TICKS.load(Ordering::Relaxed).wrapping_add(ticks), Ordering::Relaxed);
    }
    // standard crystal bias again (the SDK's `HSECFG_Current(HSE_RCur_100)`)
    unsafe {
        let tune = (R8_XT32M_TUNE.read_volatile() & !RB_XT32M_I_BIAS) | XT32M_BIAS_100;
        u8::safe_store(R8_XT32M_TUNE, tune);
    }
    if slept {
        // idle (clocks on, CPU stopped) until the crystal is stable
        let settled_at = settled_at.unwrap_or(wake_at + WAKE_UP_TICKS);
        if rtc_wake::arm(settled_at) {
            critical_section::with(|_| unsafe {
                clear_event();
                core::arch::asm!("wfi");
            });
        }
    }
    slept
}

/// A byte still in a UART's shift register would be cut off by the sleep.
fn wait_uarts_idle() {
    const RB_LSR_TX_ALL_EMP: u8 = 0x40;
    // R8_SLP_CLK_OFF0: bit set = UART clock off (bits 2..5 = UART0..3)
    let clk_off = unsafe { (0x4000_100C as *const u8).read_volatile() };
    // R8_UARTx_LSR
    for (uart, lsr) in [0x4000_3005u32, 0x4000_3405, 0x4000_3805, 0x4000_3C05].into_iter().enumerate() {
        if clk_off & (0x04 << uart) != 0 {
            continue;
        }
        let lsr = lsr as *const u8;
        for _ in 0..20_000 {
            if unsafe { lsr.read_volatile() } & RB_LSR_TX_ALL_EMP != 0 {
                break;
            }
        }
    }
}

/// Busy loop that doesn't touch flash: ~`cycles` CPU cycles.
#[inline(always)]
fn spin(cycles: u32) {
    let mut n = cycles / 2;
    while n > 0 {
        unsafe { core::arch::asm!("addi {0}, {0}, -1", inout(reg) n) };
    }
}

/// Empty QingKe's event register: raise the event, then consume it with a WFE that returns at
/// once (the CH32 SDK's `__WFE` does the same). Runs from RAM, sleep bits must be clear.
#[inline(always)]
unsafe fn clear_event() {
    const SETEVENT: u32 = 1 << 5;
    PFIC_SCTLR.write_volatile(PFIC_SCTLR.read_volatile() | SETEVENT);
    core::arch::asm!("wfi"); // WFE (WFITOWFE)
}

#[repr(C, align(4))]
struct Mac([u8; 8]);

/// MAC address read through the ROM (runs from RAM). Fails while the flash is still asleep.
#[inline(always)]
unsafe fn mac_word() -> u32 {
    let mut mac = Mac([0; 8]);
    crate::isp::FLASH_EEPROM_CMD(CMD_GET_ROM_INFO, ROM_CFG_MAC_ADDR, mac.0.as_mut_ptr(), 0);
    (mac.0.as_ptr() as *const u32).read_volatile()
}

/// `LowPower_Sleep(RB_PWR_RAM2K | RB_PWR_RAM30K | RB_PWR_EXTEND)` from the WCH SDK. Runs from
/// RAM: the flash is off while the chip sleeps and for a while after it wakes.
#[qingke_rt::highcode]
#[inline(never)]
unsafe fn low_power_sleep() {
    let mac = mac_word();

    // crystal bias 150 % for a fast start, battery detector off
    let tune = (R8_XT32M_TUNE.read_volatile() & !RB_XT32M_I_BIAS) | XT32M_BIAS_150;
    u8::safe_store(R8_BAT_DET_CTRL, 0);
    u8::safe_store(R8_XT32M_TUNE, tune);

    // A latched event (an interrupt or the pender since the last WFE) would end the sleep WFE
    // at once. Interrupts are masked and no task was woken, so dropping it is safe.
    clear_event();
    PFIC_SCTLR.write_volatile(PFIC_SCTLR.read_volatile() | SLEEPDEEP);
    let plan = (R16_POWER_PLAN.read_volatile() & (RB_PWR_DCDC_EN | RB_PWR_DCDC_PRE))
        | RB_PWR_PLAN_EN
        | RB_PWR_MUST_0010
        | RB_PWR_CORE
        | RB_PWR_RAM2K
        | RB_PWR_RAM30K
        | RB_PWR_EXTEND;
    u8::safe_store(R8_SLP_POWER_CTRL, R8_SLP_POWER_CTRL.read_volatile() | RB_RAM_RET_LV);
    u8::safe_store(R8_PLL_CONFIG, R8_PLL_CONFIG.read_volatile() | (1 << 5));
    u16::safe_store(R16_POWER_PLAN, plan);

    loop {
        // WFI is WFE here (WFITOWFE), with interrupts masked: wakes on a new pending interrupt
        core::arch::asm!("wfi", "nop", "nop");
        spin(6000); // 300 µs at 20 MHz, as in the SDK
        if mac_word() == mac {
            break; // flash is back
        }
    }

    u16::safe_store(R16_POWER_PLAN, R16_POWER_PLAN.read_volatile() & !RB_PWR_PLAN_EN);
    u8::safe_store(R8_PLL_CONFIG, R8_PLL_CONFIG.read_volatile() & !(1 << 5));
    PFIC_SCTLR.write_volatile(PFIC_SCTLR.read_volatile() & !SLEEPDEEP);
    spin(400); // 20 µs
}
