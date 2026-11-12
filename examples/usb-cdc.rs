//! USB CDC-ACM serial port that prints a log line every second and echoes what you type.
//!
//! Connect, then open the port, e.g. `picocom /dev/ttyACM1`. Uses the `USB` controller
//! (PB10/PB11); change `p.USB` / the interrupt to `USB2` for PB12/PB13.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::fmt::Write as _;
use core::ptr::addr_of_mut;

use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Instant, Ticker};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use hal::usb::{Driver, EndpointBuffers};
use {ch58x_hal as hal, panic_halt as _};

#[qingke_rt::interrupt]
fn USB() {
    unsafe { hal::usb::on_interrupt() };
}

static mut BUFFERS: EndpointBuffers = EndpointBuffers::new();

/// `core::fmt::Write` into a fixed buffer.
struct Line {
    buf: [u8; 64],
    len: usize,
}

impl core::fmt::Write for Line {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let end = self.len + s.len();
        if end > self.buf.len() {
            return Err(core::fmt::Error);
        }
        self.buf[self.len..end].copy_from_slice(s.as_bytes());
        self.len = end;
        Ok(())
    }
}

async fn log_session<'d>(class: &mut CdcAcmClass<'d, Driver<'d, hal::peripherals::USB>>) -> Result<(), EndpointError> {
    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut count = 0u32;
    let mut rx = [0u8; 64];
    loop {
        match select(ticker.next(), class.read_packet(&mut rx)).await {
            Either::First(()) => {
                count += 1;
                let mut line = Line { buf: [0; 64], len: 0 };
                let _ = write!(line, "[{:>8} ms] log line {}\r\n", Instant::now().as_millis(), count);
                class.write_packet(&line.buf[..line.len]).await?;
            }
            Either::Second(n) => {
                let n = n?;
                class.write_packet(b"echo: ").await?;
                class.write_packet(&rx[..n]).await?;
                class.write_packet(b"\r\n").await?;
            }
        }
    }
}

#[qingke_rt::entry]
fn main() -> ! {
    hal::embassy::Executor::new().run(|spawner| spawner.must_spawn(run()))
}

#[embassy_executor::task]
async fn run() -> ! {
    let mut config = hal::Config::default();
    config.clock.use_pll_60mhz();
    let p = hal::init(config);
    hal::embassy::init();

    let driver = Driver::new(p.USB, unsafe { &mut *addr_of_mut!(BUFFERS) });

    // pid.codes test VID/PID, for development only
    let mut usb_config = Config::new(0x1209, 0x0001);
    usb_config.manufacturer = Some("ch58x-hal");
    usb_config.product = Some("CH58x CDC log");
    usb_config.serial_number = Some("0001");
    usb_config.max_power = 100;
    usb_config.max_packet_size_0 = 64;
    // CDC uses an interface association descriptor (needed by Windows)
    usb_config.device_class = 0xEF;
    usb_config.device_sub_class = 0x02;
    usb_config.device_protocol = 0x01;
    usb_config.composite_with_iads = true;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut state = State::new();
    let mut builder = Builder::new(
        driver,
        usb_config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buf,
    );
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let mut usb = builder.build();

    let logger = async {
        loop {
            class.wait_connection().await;
            let _ = log_session(&mut class).await;
        }
    };
    join(usb.run(), logger).await;
    loop {}
}
