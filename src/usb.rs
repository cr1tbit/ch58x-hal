//! USB full-speed device driver for [`embassy-usb`](https://docs.rs/embassy-usb).
//!
//! Works with either controller: `USB` (UD+/UD- on PB11/PB10) or `USB2` (PB13/PB12).
//! Only one controller can be used at a time.
//!
//! Usage: bind the interrupt, then build an `embassy_usb::UsbDevice` on top of [`Driver`]:
//!
//! ```ignore
//! #[qingke_rt::interrupt]
//! fn USB() {
//!     unsafe { hal::usb::on_interrupt() };
//! }
//!
//! static mut BUFFERS: hal::usb::EndpointBuffers = hal::usb::EndpointBuffers::new();
//! let driver = hal::usb::Driver::new(p.USB, unsafe { &mut *addr_of_mut!(BUFFERS) });
//! ```
//!
//! Supported: control endpoint 0, and endpoints 1..=3 in both directions (bulk / interrupt,
//! up to 64 bytes per packet). Endpoint 4 (shares the EP0 buffer) and 5..=7 are not used.
//!
//! How it works: the interrupt handler completes every transfer right away. It records the
//! result per endpoint (received length, IN done, SETUP packet), switches that endpoint to NAK
//! so its buffer is kept, clears the transfer flag and wakes the waiting task. While the flag is
//! set the controller NAKs all traffic (`RB_UC_INT_BUSY`), so clearing it in the handler keeps
//! one endpoint from blocking the others. Reads are cancel-safe: data received after a read
//! was dropped stays in the buffer for the next read.
//!
//! Known issue: with qingke-rt's WFE-based idle (`#[embassy_executor::main]`), a pending USB
//! interrupt does not always wake the CPU and enumeration stalls. Poll the executor
//! continuously instead, see `examples/usb-cdc.rs`.
//!
//! Register reference: WCH SDK `CH583SFR.h`, `CH58x_usbdev.c`.

use core::cell::Cell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::ptr;
use core::task::Poll;

use critical_section::Mutex;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{
    Direction, EndpointAddress, EndpointAllocError, EndpointError, EndpointInfo, EndpointType, Event, Unsupported,
};

use crate::{peripherals, Peripheral};

// ---- registers (byte offsets from the controller base) ----

const R8_USB_CTRL: usize = 0x00;
const R8_UDEV_CTRL: usize = 0x01;
const R8_USB_INT_EN: usize = 0x02;
const R8_USB_DEV_AD: usize = 0x03;
const R8_USB_MIS_ST: usize = 0x05;
const R8_USB_INT_FG: usize = 0x06;
const R8_USB_INT_ST: usize = 0x07;
const R8_USB_RX_LEN: usize = 0x08;
const R8_UEP4_1_MOD: usize = 0x0C;
const R8_UEP2_3_MOD: usize = 0x0D;
/// `R16_UEPn_DMA`, n = 0..=3
const fn uep_dma(ep: usize) -> usize {
    0x10 + 4 * ep
}
/// `R8_UEPn_T_LEN`, n = 0..=4
const fn uep_t_len(ep: usize) -> usize {
    0x20 + 4 * ep
}
/// `R8_UEPn_CTRL`, n = 0..=4
const fn uep_ctrl(ep: usize) -> usize {
    0x22 + 4 * ep
}

const RB_UC_DEV_PU_EN: u8 = 0x20;
const RB_UC_INT_BUSY: u8 = 0x08;
const RB_UC_RESET_SIE: u8 = 0x04;
const RB_UC_CLR_ALL: u8 = 0x02;
const RB_UC_DMA_EN: u8 = 0x01;

const RB_UD_PD_DIS: u8 = 0x80;
const RB_UD_PORT_EN: u8 = 0x01;

const RB_UIE_SUSPEND: u8 = 0x04;
const RB_UIE_TRANSFER: u8 = 0x02;
const RB_UIE_BUS_RST: u8 = 0x01;

const RB_UMS_SUSPEND: u8 = 0x04;

const RB_UIF_FIFO_OV: u8 = 0x10;
const RB_UIF_HST_SOF: u8 = 0x08;
const RB_UIF_SUSPEND: u8 = 0x04;
const RB_UIF_TRANSFER: u8 = 0x02;
const RB_UIF_BUS_RST: u8 = 0x01;

const RB_UIS_SETUP_ACT: u8 = 0x80;
const RB_UIS_TOG_OK: u8 = 0x40;
const MASK_UIS_TOKEN: u8 = 0x30;
const UIS_TOKEN_OUT: u8 = 0x00;
const UIS_TOKEN_IN: u8 = 0x20;
const MASK_UIS_ENDP: u8 = 0x0F;

const RB_UEP_R_TOG: u8 = 0x80;
const RB_UEP_T_TOG: u8 = 0x40;
/// Automatic data toggle; only endpoints 1..=3 have it.
const RB_UEP_AUTO_TOG: u8 = 0x10;
const MASK_UEP_R_RES: u8 = 0x0C;
const MASK_UEP_T_RES: u8 = 0x03;

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum Res {
    Ack = 0b00,
    Nak = 0b10,
    Stall = 0b11,
}

/// `R16_PIN_ANALOG_IE` in the SYS block.
const R16_PIN_ANALOG_IE: *mut u16 = 0x4000_101A as *mut u16;

const EP0_SIZE: usize = 64;
const MAX_PACKET_SIZE: u16 = 64;
/// Endpoints 0..=3 are used.
const NR_EP: usize = 4;

// ---- state shared between the interrupt handler and the tasks ----

/// The controller in use (base address and IRQ number), set by [`Driver::new`].
static ACTIVE: Mutex<Cell<Option<(usize, u8)>>> = Mutex::new(Cell::new(None));

static BUS_WAKER: AtomicWaker = AtomicWaker::new();
static IN_WAKERS: [AtomicWaker; NR_EP] = [const { AtomicWaker::new() }; NR_EP];
static OUT_WAKERS: [AtomicWaker; NR_EP] = [const { AtomicWaker::new() }; NR_EP];

struct State {
    /// Bus events not yet reported by `Bus::poll`.
    reset: Cell<bool>,
    suspend: Cell<Option<bool>>,
    /// A SETUP packet waiting for `ControlPipe::setup`.
    setup: Cell<Option<[u8; 8]>>,
    /// Received OUT packet length per endpoint; the data waits in the endpoint buffer (NAK).
    out_len: [Cell<Option<u8>>; NR_EP],
    /// IN packet sent per endpoint.
    in_done: [Cell<bool>; NR_EP],
    /// Enabled endpoints: bit n = IN endpoint n, bit n + 4 = OUT endpoint n.
    enabled: Cell<u8>,
}

static STATE: Mutex<State> = Mutex::new(State {
    reset: Cell::new(false),
    suspend: Cell::new(None),
    setup: Cell::new(None),
    out_len: [const { Cell::new(None) }; NR_EP],
    in_done: [const { Cell::new(false) }; NR_EP],
    enabled: Cell::new(0),
});

fn with_state<R>(f: impl FnOnce(&State) -> R) -> R {
    critical_section::with(|cs| f(STATE.borrow(cs)))
}

fn enabled_bit(ep: usize, dir: Direction) -> u8 {
    match dir {
        Direction::In => 1 << ep,
        Direction::Out => 1 << (ep + 4),
    }
}

fn is_enabled(ep: usize, dir: Direction) -> bool {
    with_state(|s| s.enabled.get() & enabled_bit(ep, dir) != 0)
}

#[derive(Clone, Copy)]
struct Regs {
    base: usize,
}

impl Regs {
    fn active() -> Self {
        let (base, _) = critical_section::with(|cs| ACTIVE.borrow(cs).get()).expect("USB driver not started");
        Self { base }
    }

    fn read(self, offset: usize) -> u8 {
        unsafe { ptr::read_volatile((self.base + offset) as *const u8) }
    }

    fn write(self, offset: usize, value: u8) {
        unsafe { ptr::write_volatile((self.base + offset) as *mut u8, value) }
    }

    fn modify(self, offset: usize, f: impl FnOnce(u8) -> u8) {
        critical_section::with(|_| self.write(offset, f(self.read(offset))));
    }

    fn set_dma(self, ep: usize, addr: *const u8) {
        // The DMA address register holds the low 16 bits of the RAM address.
        unsafe { ptr::write_volatile((self.base + uep_dma(ep)) as *mut u16, addr as u32 as u16) }
    }

    fn set_t_res(self, ep: usize, res: Res) {
        self.modify(uep_ctrl(ep), |v| (v & !MASK_UEP_T_RES) | res as u8);
    }

    fn set_r_res(self, ep: usize, res: Res) {
        self.modify(uep_ctrl(ep), |v| (v & !MASK_UEP_R_RES) | ((res as u8) << 2));
    }

    fn t_res(self, ep: usize) -> u8 {
        self.read(uep_ctrl(ep)) & MASK_UEP_T_RES
    }

    fn r_res(self, ep: usize) -> u8 {
        (self.read(uep_ctrl(ep)) & MASK_UEP_R_RES) >> 2
    }

    fn set_t_tog(self, ep: usize, data1: bool) {
        self.modify(uep_ctrl(ep), |v| if data1 { v | RB_UEP_T_TOG } else { v & !RB_UEP_T_TOG });
    }

    fn set_r_tog(self, ep: usize, data1: bool) {
        self.modify(uep_ctrl(ep), |v| if data1 { v | RB_UEP_R_TOG } else { v & !RB_UEP_R_TOG });
    }

    fn flip_t_tog(self, ep: usize) {
        self.modify(uep_ctrl(ep), |v| v ^ RB_UEP_T_TOG);
    }

    fn flip_r_tog(self, ep: usize) {
        self.modify(uep_ctrl(ep), |v| v ^ RB_UEP_R_TOG);
    }
}

/// Handle the USB interrupt. Call it from the `USB` (or `USB2`) interrupt handler.
///
/// # Safety
/// Must only be called from the interrupt handler of the controller passed to [`Driver::new`].
pub unsafe fn on_interrupt() {
    let regs = Regs::active();
    let flags = regs.read(R8_USB_INT_FG);

    // The SOF flag is set every frame even with its interrupt disabled.
    if flags & (RB_UIF_FIFO_OV | RB_UIF_HST_SOF) != 0 {
        regs.write(R8_USB_INT_FG, flags & (RB_UIF_FIFO_OV | RB_UIF_HST_SOF));
    }

    if flags & RB_UIF_BUS_RST != 0 {
        reset_endpoints(regs);
        regs.write(R8_USB_INT_FG, RB_UIF_BUS_RST);
        with_state(|s| s.reset.set(true));
        BUS_WAKER.wake();
    }

    if flags & RB_UIF_SUSPEND != 0 {
        regs.write(R8_USB_INT_FG, RB_UIF_SUSPEND);
        let suspended = regs.read(R8_USB_MIS_ST) & RB_UMS_SUSPEND != 0;
        with_state(|s| s.suspend.set(Some(suspended)));
        BUS_WAKER.wake();
    }

    if flags & RB_UIF_TRANSFER != 0 {
        let st = regs.read(R8_USB_INT_ST);
        let ep = (st & MASK_UIS_ENDP) as usize;
        // Same order as the WCH SDK: first a completed IN/OUT, then a received SETUP.
        match st & MASK_UIS_TOKEN {
            UIS_TOKEN_OUT if ep < NR_EP && st & RB_UIS_TOG_OK != 0 => {
                // Keep the data in the buffer until a read takes it.
                regs.set_r_res(ep, Res::Nak);
                let len = regs.read(R8_USB_RX_LEN);
                with_state(|s| s.out_len[ep].set(Some(len)));
                OUT_WAKERS[ep].wake();
            }
            UIS_TOKEN_IN if ep < NR_EP => {
                regs.set_t_res(ep, Res::Nak);
                with_state(|s| s.in_done[ep].set(true));
                IN_WAKERS[ep].wake();
            }
            // repeated OUT packet (toggle mismatch), SOF, or an endpoint we don't use
            _ => {}
        }
        if st & RB_UIS_SETUP_ACT != 0 {
            let mut request = [0u8; 8];
            ep0_buffer().read(&mut request);
            // Data and status stages start with DATA1; NAK until the stack decides.
            regs.write(
                uep_ctrl(0),
                RB_UEP_R_TOG | RB_UEP_T_TOG | (Res::Nak as u8) | ((Res::Nak as u8) << 2),
            );
            with_state(|s| {
                s.setup.set(Some(request));
                // a SETUP aborts any control transfer in progress
                s.out_len[0].set(None);
                s.in_done[0].set(false);
            });
            OUT_WAKERS[0].wake();
            IN_WAKERS[0].wake();
        }
        regs.write(R8_USB_INT_FG, RB_UIF_TRANSFER);
    }
}

/// After start and bus reset: address 0, EP0 ready for SETUP, all other endpoints disabled.
fn reset_endpoints(regs: Regs) {
    regs.write(R8_USB_DEV_AD, 0);
    regs.write(uep_ctrl(0), (Res::Nak as u8) | ((Res::Ack as u8) << 2));
    for ep in 1..NR_EP {
        regs.write(uep_t_len(ep), 0);
        regs.write(uep_ctrl(ep), RB_UEP_AUTO_TOG | (Res::Nak as u8) | ((Res::Nak as u8) << 2));
    }
    with_state(|s| {
        s.enabled.set(0);
        s.setup.set(None);
        for ep in 0..NR_EP {
            s.out_len[ep].set(None);
            s.in_done[ep].set(false);
        }
    });
    for ep in 0..NR_EP {
        IN_WAKERS[ep].wake();
        OUT_WAKERS[ep].wake();
    }
}

/// A USB controller.
pub trait Instance: Peripheral<P = Self> + 'static {
    #[doc(hidden)]
    const BASE: usize;
    #[doc(hidden)]
    const IRQ: u8;
    /// `RB_PIN_USB*_IE | RB_PIN_USB*_DP_PU` in `R16_PIN_ANALOG_IE`.
    #[doc(hidden)]
    const ANALOG_IE: u16;
}

impl Instance for peripherals::USB {
    const BASE: usize = 0x4000_8000;
    const IRQ: u8 = crate::pac::Interrupt::USB as u8;
    const ANALOG_IE: u16 = 0x80 | 0x40;
}

impl Instance for peripherals::USB2 {
    const BASE: usize = 0x4000_8400;
    const IRQ: u8 = crate::pac::Interrupt::USB2 as u8;
    const ANALOG_IE: u16 = 0x20 | 0x10;
}

/// RAM the controller reads and writes by DMA: 64 bytes for EP0, and 64 OUT + 64 IN bytes
/// for each of the endpoints 1..=3.
#[repr(C, align(4))]
pub struct EndpointBuffers {
    ep0: [u8; EP0_SIZE],
    ep: [[u8; 2 * MAX_PACKET_SIZE as usize]; NR_EP - 1],
}

impl EndpointBuffers {
    pub const fn new() -> Self {
        Self {
            ep0: [0; EP0_SIZE],
            ep: [[0; 2 * MAX_PACKET_SIZE as usize]; NR_EP - 1],
        }
    }
}

/// EP0 buffer address, set by [`Driver::new`] (the interrupt handler reads SETUP packets).
static EP0_BUFFER: Mutex<Cell<usize>> = Mutex::new(Cell::new(0));

fn ep0_buffer() -> DmaBuf {
    DmaBuf(critical_section::with(|cs| EP0_BUFFER.borrow(cs).get()) as *mut u8)
}

/// Endpoint data buffer in DMA RAM.
#[derive(Clone, Copy)]
struct DmaBuf(*mut u8);

impl DmaBuf {
    fn read(self, dst: &mut [u8]) {
        for (i, byte) in dst.iter_mut().enumerate() {
            *byte = unsafe { ptr::read_volatile(self.0.add(i)) };
        }
    }

    fn write(self, src: &[u8]) {
        for (i, &byte) in src.iter().enumerate() {
            unsafe { ptr::write_volatile(self.0.add(i), byte) };
        }
    }
}

pub struct Driver<'d, T: Instance> {
    _usb: PhantomData<&'d mut T>,
    buffers: *mut EndpointBuffers,
    /// bit n: IN endpoint n allocated, bit n + 4: OUT endpoint n allocated
    allocated: u8,
}

impl<'d, T: Instance> Driver<'d, T> {
    pub fn new(_usb: impl Peripheral<P = T> + 'd, buffers: &'d mut EndpointBuffers) -> Self {
        critical_section::with(|cs| {
            ACTIVE.borrow(cs).set(Some((T::BASE, T::IRQ)));
            EP0_BUFFER.borrow(cs).set(buffers.ep0.as_mut_ptr() as usize);
        });
        Self {
            _usb: PhantomData,
            buffers,
            allocated: 0,
        }
    }

    fn alloc<D>(
        &mut self,
        dir: Direction,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Endpoint<'d, D>, EndpointAllocError> {
        if max_packet_size > MAX_PACKET_SIZE || ep_type == EndpointType::Isochronous {
            return Err(EndpointAllocError);
        }
        let ep = (1..NR_EP)
            .find(|&ep| self.allocated & enabled_bit(ep, dir) == 0)
            .ok_or(EndpointAllocError)?;
        self.allocated |= enabled_bit(ep, dir);
        // Both directions of an endpoint are always enabled in the controller, so the
        // buffer layout is fixed: OUT data first, IN data 64 bytes later.
        let base = unsafe { (*self.buffers).ep[ep - 1].as_mut_ptr() };
        let buf = match dir {
            Direction::Out => base,
            Direction::In => unsafe { base.add(MAX_PACKET_SIZE as usize) },
        };
        Ok(Endpoint {
            _phantom: PhantomData,
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(ep, dir),
                ep_type,
                max_packet_size,
                interval_ms,
            },
            buf: DmaBuf(buf),
        })
    }
}

impl<'d, T: Instance> embassy_usb_driver::Driver<'d> for Driver<'d, T> {
    type EndpointOut = Endpoint<'d, Out>;
    type EndpointIn = Endpoint<'d, In>;
    type ControlPipe = ControlPipe<'d>;
    type Bus = Bus<'d, T>;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.alloc(Direction::Out, ep_type, max_packet_size, interval_ms)
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.alloc(Direction::In, ep_type, max_packet_size, interval_ms)
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        assert!(control_max_packet_size as usize <= EP0_SIZE);
        let regs = Regs::active();
        unsafe { qingke::pfic::disable_interrupt(T::IRQ) };

        // Reset the SIE, then device mode with DMA, still disconnected (no pull-up yet).
        regs.write(R8_USB_CTRL, RB_UC_RESET_SIE | RB_UC_CLR_ALL);
        crate::delay_us(10);
        regs.write(R8_USB_CTRL, 0);

        regs.write(R8_UEP4_1_MOD, 0xC0); // EP1 OUT + IN
        regs.write(R8_UEP2_3_MOD, 0xCC); // EP2 OUT + IN, EP3 OUT + IN
        let ep0 = unsafe { (*self.buffers).ep0.as_mut_ptr() };
        regs.set_dma(0, ep0);
        for ep in 1..NR_EP {
            regs.set_dma(ep, unsafe { (*self.buffers).ep[ep - 1].as_ptr() });
        }
        reset_endpoints(regs);

        regs.write(R8_USB_CTRL, RB_UC_INT_BUSY | RB_UC_DMA_EN);
        regs.write(R8_USB_INT_FG, 0xFF);
        regs.write(R8_USB_INT_EN, RB_UIE_SUSPEND | RB_UIE_TRANSFER | RB_UIE_BUS_RST);
        unsafe { qingke::pfic::enable_interrupt(T::IRQ) };

        (
            Bus {
                _usb: PhantomData,
                power_reported: false,
            },
            ControlPipe {
                _phantom: PhantomData,
                buf: DmaBuf(ep0),
            },
        )
    }
}

pub struct Bus<'d, T: Instance> {
    _usb: PhantomData<&'d mut T>,
    power_reported: bool,
}

impl<'d, T: Instance> embassy_usb_driver::Bus for Bus<'d, T> {
    async fn enable(&mut self) {
        let regs = Regs::active();
        // Connect: enable the port and the D+ pull-up.
        unsafe { R16_PIN_ANALOG_IE.write_volatile(R16_PIN_ANALOG_IE.read_volatile() | T::ANALOG_IE) };
        regs.write(R8_UDEV_CTRL, RB_UD_PD_DIS | RB_UD_PORT_EN);
        regs.modify(R8_USB_CTRL, |v| v | RB_UC_DEV_PU_EN);
    }

    async fn disable(&mut self) {
        let regs = Regs::active();
        regs.modify(R8_USB_CTRL, |v| v & !RB_UC_DEV_PU_EN);
        regs.write(R8_UDEV_CTRL, 0);
        unsafe { R16_PIN_ANALOG_IE.write_volatile(R16_PIN_ANALOG_IE.read_volatile() & !T::ANALOG_IE) };
    }

    async fn poll(&mut self) -> Event {
        // No VBUS sensing: assume the device is powered by the bus.
        if !self.power_reported {
            self.power_reported = true;
            return Event::PowerDetected;
        }
        poll_fn(|cx| {
            BUS_WAKER.register(cx.waker());
            with_state(|s| {
                if s.reset.replace(false) {
                    Poll::Ready(Event::Reset)
                } else {
                    match s.suspend.take() {
                        Some(true) => Poll::Ready(Event::Suspend),
                        Some(false) => Poll::Ready(Event::Resume),
                        None => Poll::Pending,
                    }
                }
            })
        })
        .await
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        let ep = ep_addr.index();
        if ep == 0 || ep >= NR_EP {
            return;
        }
        let regs = Regs::active();
        let dir = ep_addr.direction();
        // A (re)configured endpoint starts with DATA0 and responds NAK until used.
        match dir {
            Direction::In => {
                regs.set_t_tog(ep, false);
                regs.set_t_res(ep, Res::Nak);
            }
            Direction::Out => {
                regs.set_r_tog(ep, false);
                regs.set_r_res(ep, Res::Nak);
            }
        }
        with_state(|s| {
            let bit = enabled_bit(ep, dir);
            s.enabled.set(if enabled { s.enabled.get() | bit } else { s.enabled.get() & !bit });
            s.out_len[ep].set(None);
            s.in_done[ep].set(false);
        });
        match dir {
            Direction::In => IN_WAKERS[ep].wake(),
            Direction::Out => OUT_WAKERS[ep].wake(),
        }
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        let ep = ep_addr.index();
        if ep >= NR_EP {
            return;
        }
        let regs = Regs::active();
        let res = if stalled { Res::Stall } else { Res::Nak };
        match ep_addr.direction() {
            Direction::In => {
                if !stalled {
                    regs.set_t_tog(ep, false);
                }
                regs.set_t_res(ep, res);
            }
            Direction::Out => {
                if !stalled {
                    regs.set_r_tog(ep, false);
                }
                regs.set_r_res(ep, res);
            }
        }
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        let ep = ep_addr.index();
        if ep >= NR_EP {
            return false;
        }
        let regs = Regs::active();
        match ep_addr.direction() {
            Direction::In => regs.t_res(ep) == Res::Stall as u8,
            Direction::Out => regs.r_res(ep) == Res::Stall as u8,
        }
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

pub enum In {}
pub enum Out {}

pub struct Endpoint<'d, D> {
    _phantom: PhantomData<(&'d (), D)>,
    info: EndpointInfo,
    buf: DmaBuf,
}

impl<'d, D> Endpoint<'d, D> {
    fn ep(&self) -> usize {
        self.info.addr.index()
    }
}

impl<'d, D> embassy_usb_driver::Endpoint for Endpoint<'d, D> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        let (ep, dir) = (self.ep(), self.info.addr.direction());
        poll_fn(|cx| {
            match dir {
                Direction::In => IN_WAKERS[ep].register(cx.waker()),
                Direction::Out => OUT_WAKERS[ep].register(cx.waker()),
            }
            if is_enabled(ep, dir) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

impl<'d> embassy_usb_driver::EndpointIn for Endpoint<'d, In> {
    async fn write(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        let ep = self.ep();
        if data.len() > self.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }
        if !is_enabled(ep, Direction::In) {
            return Err(EndpointError::Disabled);
        }
        let regs = Regs::active();
        // If an earlier write was dropped before its packet went out, it is replaced here.
        regs.set_t_res(ep, Res::Nak);
        with_state(|s| s.in_done[ep].set(false));
        self.buf.write(data);
        regs.write(uep_t_len(ep), data.len() as u8);
        regs.set_t_res(ep, Res::Ack);

        poll_fn(|cx| {
            IN_WAKERS[ep].register(cx.waker());
            with_state(|s| {
                if s.enabled.get() & enabled_bit(ep, Direction::In) == 0 {
                    Poll::Ready(Err(EndpointError::Disabled))
                } else if s.in_done[ep].replace(false) {
                    // the toggle advances by itself (RB_UEP_AUTO_TOG)
                    Poll::Ready(Ok(()))
                } else {
                    Poll::Pending
                }
            })
        })
        .await
    }
}

impl<'d> embassy_usb_driver::EndpointOut for Endpoint<'d, Out> {
    async fn read(&mut self, data: &mut [u8]) -> Result<usize, EndpointError> {
        let ep = self.ep();
        if !is_enabled(ep, Direction::Out) {
            return Err(EndpointError::Disabled);
        }
        let regs = Regs::active();
        // Accept a packet, unless one already waits in the buffer (from a dropped read).
        if with_state(|s| s.out_len[ep].get().is_none()) {
            regs.set_r_res(ep, Res::Ack);
        }

        let len = poll_fn(|cx| {
            OUT_WAKERS[ep].register(cx.waker());
            with_state(|s| {
                if s.enabled.get() & enabled_bit(ep, Direction::Out) == 0 {
                    Poll::Ready(Err(EndpointError::Disabled))
                } else if let Some(len) = s.out_len[ep].take() {
                    Poll::Ready(Ok(len as usize))
                } else {
                    Poll::Pending
                }
            })
        })
        .await?;
        // The endpoint NAKs until the next read, so the buffer is still intact.
        if len > data.len() {
            return Err(EndpointError::BufferOverflow);
        }
        self.buf.read(&mut data[..len]);
        Ok(len)
    }
}

pub struct ControlPipe<'d> {
    _phantom: PhantomData<&'d ()>,
    buf: DmaBuf,
}

/// Wait for the IN packet on EP0, or a new SETUP (which aborts the transfer).
async fn ep0_in_done() -> Result<(), EndpointError> {
    poll_fn(|cx| {
        IN_WAKERS[0].register(cx.waker());
        with_state(|s| {
            if s.setup.get().is_some() {
                Poll::Ready(Err(EndpointError::Disabled))
            } else if s.in_done[0].replace(false) {
                Poll::Ready(Ok(()))
            } else {
                Poll::Pending
            }
        })
    })
    .await
}

/// Wait for an OUT packet on EP0, or a new SETUP (which aborts the transfer).
async fn ep0_out_len() -> Result<usize, EndpointError> {
    poll_fn(|cx| {
        OUT_WAKERS[0].register(cx.waker());
        with_state(|s| {
            if s.setup.get().is_some() {
                Poll::Ready(Err(EndpointError::Disabled))
            } else if let Some(len) = s.out_len[0].take() {
                Poll::Ready(Ok(len as usize))
            } else {
                Poll::Pending
            }
        })
    })
    .await
}

impl<'d> embassy_usb_driver::ControlPipe for ControlPipe<'d> {
    fn max_packet_size(&self) -> usize {
        EP0_SIZE
    }

    async fn setup(&mut self) -> [u8; 8] {
        poll_fn(|cx| {
            OUT_WAKERS[0].register(cx.waker());
            with_state(|s| match s.setup.take() {
                Some(request) => Poll::Ready(request),
                None => Poll::Pending,
            })
        })
        .await
    }

    async fn data_out(&mut self, data: &mut [u8], first: bool, _last: bool) -> Result<usize, EndpointError> {
        let regs = Regs::active();
        if !first {
            regs.flip_r_tog(0);
        }
        regs.set_r_res(0, Res::Ack);
        let len = ep0_out_len().await?;
        if len > data.len() {
            return Err(EndpointError::BufferOverflow);
        }
        self.buf.read(&mut data[..len]);
        Ok(len)
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        if data.len() > EP0_SIZE {
            return Err(EndpointError::BufferOverflow);
        }
        let regs = Regs::active();
        self.buf.write(data);
        regs.write(uep_t_len(0), data.len() as u8);
        if !first {
            regs.flip_t_tog(0);
        }
        regs.set_t_res(0, Res::Ack);
        ep0_in_done().await?;

        if last {
            // status stage: zero-length OUT with DATA1
            regs.set_r_tog(0, true);
            regs.set_r_res(0, Res::Ack);
            ep0_out_len().await?;
        }
        Ok(())
    }

    async fn accept(&mut self) {
        // status stage: zero-length IN with DATA1
        let regs = Regs::active();
        regs.write(uep_t_len(0), 0);
        regs.set_t_tog(0, true);
        regs.set_t_res(0, Res::Ack);
        let _ = ep0_in_done().await;
    }

    async fn reject(&mut self) {
        let regs = Regs::active();
        regs.set_t_res(0, Res::Stall);
        regs.set_r_res(0, Res::Stall);
    }

    async fn accept_set_address(&mut self, addr: u8) {
        self.accept().await;
        // The new address applies after the status stage.
        Regs::active().modify(R8_USB_DEV_AD, |v| (v & 0x80) | (addr & 0x7F));
    }
}
