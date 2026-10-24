//! Mitsubishi HF-KP serial absolute encoder feed.
//!
//! Polls the 18-bit single-turn encoder over RS-485 back to back: each
//! receive DMA completion (one every ~50 us) hands the position to
//! [`crate::tmc_bus::feed`], which writes it to the TMC4671
//! `ABN_DECODER_COUNT`. The TMC4671 derives both the electrical angle and
//! the mechanical position from that count, so the rest of the firmware
//! treats the encoder as an ABN encoder with `encoder_ppr` lines (65536
//! gives the full resolution).
//!
//! Wiring (MAX485-style transceiver, RE tied low):
//! - PB10 (GP7) USART3 TX -> DI
//! - PA1 (GP5) UART4 RX <- RO
//! - PB4 (GP2) TIM3_CH1 -> DE
//!
//! The request leaves on USART3 while a TIM3 one-pulse holds DE for the
//! request byte, and the reply arrives on UART4 through DMA, so the CPU
//! never waits on the line.

use core::sync::atomic::{AtomicBool, Ordering::Relaxed};

use embassy_stm32::Peri;
use embassy_stm32::gpio::{AfType, Flex, OutputType, Pull, Speed};
use embassy_stm32::pac;
use embassy_stm32::peripherals;
use foci_firmware::absolute_encoder::hfkp;

/// APB1 clock from `board_init::rcc_config` (168 MHz / 4); its timers run
/// at twice that.
const APB1_HZ: u32 = 42_000_000;
const APB1_TIMER_HZ: u32 = 2 * APB1_HZ;

const RX_DMA_STREAM: usize = 2; // DMA1 stream 2 channel 4: UART4_RX
const RX_DMA_CHANNEL: u8 = 4;

/// DE pulse: the request byte (10 bits at 2.5 Mbaud = 4 us) plus margin.
const DE_PULSE_NS: u32 = 4_400;

/// TIM7 restarts the request chain if no reply completed in this long (a
/// lost frame never raises the DMA interrupt).
const WATCHDOG_US: u32 = 500;

static mut RX_BUF: [u8; hfkp::FRAME_LEN] = [0; hfkp::FRAME_LEN];

/// A reply completed since the last watchdog tick.
static REPLIED: AtomicBool = AtomicBool::new(false);

/// Peripherals the feed owns; taking them keeps the rest of the firmware
/// from claiming them.
pub(crate) struct HfkpPins {
    pub uart4: Peri<'static, peripherals::UART4>,
    pub usart3: Peri<'static, peripherals::USART3>,
    pub rx_dma: Peri<'static, peripherals::DMA1_CH2>,
    pub timer: Peri<'static, peripherals::TIM7>,
    pub tx: Peri<'static, peripherals::PB10>,
    pub rx: Peri<'static, peripherals::PA1>,
    pub de: Peri<'static, peripherals::PB4>,
    pub de_timer: Peri<'static, peripherals::TIM3>,
}

/// Configure the pins, both USARTs and the receive DMA stream.
pub(crate) fn init(pins: HfkpPins) {
    let HfkpPins {
        uart4,
        usart3,
        rx_dma,
        timer,
        tx,
        rx,
        de,
        de_timer,
    } = pins;

    // Driven through the PAC below; the handles only mark them as taken.
    let _ = (uart4, usart3, rx_dma, timer, de_timer);

    let rcc = pac::RCC;
    rcc.apb1enr().modify(|w| {
        w.set_uart4en(true);
        w.set_usart3en(true);
        w.set_tim7en(true);
        w.set_tim3en(true);
    });
    rcc.ahb1enr().modify(|w| w.set_dma1en(true));
    cortex_m::asm::delay(2);

    let mut tx = Flex::new(tx);
    tx.set_as_af_unchecked(7, AfType::output(OutputType::PushPull, Speed::VeryHigh));
    let mut rx = Flex::new(rx);
    rx.set_as_af_unchecked(8, AfType::input(Pull::Up));
    // Pins must stay configured -- don't drop the handles.
    core::mem::forget(tx);
    core::mem::forget(rx);
    let mut de = Flex::new(de);
    de.set_as_af_unchecked(2, AfType::output(OutputType::PushPull, Speed::VeryHigh));
    core::mem::forget(de);

    // One-pulse PWM mode 2: the output goes high one tick after CEN and low
    // again at the update event, where the counter stops at 0.
    let tim3 = pac::TIM3;
    let pulse_ticks = DE_PULSE_NS * (APB1_TIMER_HZ / 1_000_000) / 1_000;
    tim3.psc().write_value(0);
    tim3.ccr(0).write(|w| w.set_ccr(1));
    tim3.arr().write(|w| w.set_arr((1 + pulse_ticks) as u16));
    tim3.ccmr_output(0)
        .modify(|w| w.set_ocm(0, pac::timer::vals::Ocm::PWM_MODE2));
    tim3.ccer().modify(|w| w.set_cce(0, true));
    tim3.cr1().modify(|w| w.set_opm(true));
    tim3.egr().write(|w| w.set_ug(true));

    // OVER8: BRR holds USARTDIV in eighths, 3 fraction bits.
    // 42 MHz / 2.5 Mbaud rounds to 2.125, a 1.2% baud error.
    let div8 = (APB1_HZ + hfkp::BAUD / 2) / hfkp::BAUD;
    let brr = ((div8 >> 3) << 4) | (div8 & 7);
    for usart in [pac::UART4, pac::USART3] {
        usart.brr().write_value(pac::usart::regs::Brr(brr));
        usart.cr1().write(|w| {
            w.set_over8(pac::usart::vals::Over8::OVERSAMPLING8);
            w.set_ue(true);
        });
    }
    pac::USART3.cr1().modify(|w| w.set_te(true));
    pac::UART4.cr3().modify(|w| w.set_dmar(true));
    pac::UART4.cr1().modify(|w| w.set_re(true));

    let st = pac::DMA1.st(RX_DMA_STREAM);
    st.cr().write(|w| {
        w.set_chsel(RX_DMA_CHANNEL);
        w.set_dir(pac::dma::vals::Dir::PERIPHERAL_TO_MEMORY);
        w.set_minc(true);
        w.set_tcie(true);
    });
    st.par().write_value(pac::UART4.dr().as_ptr() as u32);
    st.m0ar()
        .write_value(core::ptr::addr_of_mut!(RX_BUF) as u32);

    // 1 MHz tick, update interrupt every watchdog period.
    let tim7 = pac::TIM7;
    tim7.psc()
        .write_value((APB1_TIMER_HZ / 1_000_000 - 1) as u16);
    tim7.arr().write(|w| w.set_arr((WATCHDOG_US - 1) as u16));
    tim7.egr().write(|w| w.set_ug(true));
    tim7.sr().write_value(Default::default());
    tim7.dier().write(|w| w.set_uie(true));
    tim7.cr1().modify(|w| w.set_cen(true));

    use embassy_stm32::interrupt::{Interrupt, InterruptExt};
    for irq in [Interrupt::TIM7, Interrupt::DMA1_STREAM2] {
        irq.set_priority(POLL_INTERRUPT_PRIORITY);
        // SAFETY: the handlers are bound below and only touch state owned
        // by this module and the SPI arbiter.
        unsafe { irq.enable() };
    }
    send_request();
}

/// Both handlers run at RTIC priority 2 (NVIC 0xE0), the level the SPI
/// arbiter masks while `tmc_control` holds the bus, so they never preempt
/// each other. STEP/DIR at RTIC priority 3 still preempts them.
const POLL_INTERRUPT_PRIORITY: embassy_stm32::interrupt::Priority =
    embassy_stm32::interrupt::Priority::P14;

embassy_stm32::bind_interrupts!(struct Irqs {
    DMA1_STREAM2 => ReplyHandler;
    TIM7 => WatchdogHandler;
});

struct ReplyHandler;

impl embassy_stm32::interrupt::typelevel::Handler<embassy_stm32::interrupt::typelevel::DMA1_STREAM2>
    for ReplyHandler
{
    unsafe fn on_interrupt() {
        on_reply();
    }
}

struct WatchdogHandler;

impl embassy_stm32::interrupt::typelevel::Handler<embassy_stm32::interrupt::typelevel::TIM7>
    for WatchdogHandler
{
    unsafe fn on_interrupt() {
        on_watchdog();
    }
}

/// Receive DMA complete: feed the reply to the TMC4671 and send the next
/// request.
fn on_reply() {
    clear_rx_dma_flags();
    // A corrupted frame is skipped; the next one is ~50 us away.
    if let Some(position) = take_frame() {
        crate::tmc_bus::feed(position, hfkp::POSITION_BITS);
    }
    REPLIED.store(true, Relaxed);
    send_request();
}

/// TIM7 update: restart the request chain after a lost reply.
fn on_watchdog() {
    pac::TIM7.sr().write_value(Default::default());
    if !REPLIED.swap(false, Relaxed) {
        send_request();
    }
}

/// Return the position from a complete, checksummed reply.
fn take_frame() -> Option<u32> {
    let st = pac::DMA1.st(RX_DMA_STREAM);
    if st.cr().read().en() || st.ndtr().read().ndt() != 0 {
        return None;
    }
    // SAFETY: the DMA stream is disabled and finished, so nothing else
    // touches RX_BUF until `send_request` re-arms it.
    let frame = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(RX_BUF)) };
    hfkp::decode_position(&frame)
}

/// Clear every receive DMA stream flag, including the completion that
/// raises `on_reply`.
fn clear_rx_dma_flags() {
    let flags = RX_DMA_STREAM % 4;
    pac::DMA1.ifcr(RX_DMA_STREAM / 4).write(|w| {
        w.set_tcif(flags, true);
        w.set_htif(flags, true);
        w.set_teif(flags, true);
        w.set_dmeif(flags, true);
        w.set_feif(flags, true);
    });
}

/// Re-arm the receive DMA and send one position request.
fn send_request() {
    let st = pac::DMA1.st(RX_DMA_STREAM);
    st.cr().modify(|w| w.set_en(false));
    while st.cr().read().en() {}
    clear_rx_dma_flags();
    // Reading SR then DR clears a stale byte or overrun.
    let uart4 = pac::UART4;
    let _ = uart4.sr().read();
    let _ = uart4.dr().read();
    st.ndtr().write(|w| w.set_ndt(hfkp::FRAME_LEN as u16));
    st.cr().modify(|w| w.set_en(true));

    // Keep the request byte and the DE pulse together.
    let usart3 = pac::USART3;
    critical_section::with(|_| {
        usart3
            .dr()
            .write(|w| w.set_dr(u16::from(hfkp::REQUEST_POSITION)));
        pac::TIM3.cr1().modify(|w| w.set_cen(true));
    });
}
