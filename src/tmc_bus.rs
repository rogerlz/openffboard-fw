//! SPI1 arbitration between `tmc_control` and the absolute encoder feed.
//!
//! The encoder feed writes `ABN_DECODER_COUNT` from a priority-2 interrupt
//! every ~50 us, so it cannot wait for `tmc_control` (priority 1) to reach
//! a point where it owns the bus. Instead the bus lives here and every
//! `tmc_control` transaction runs as one blocking transfer with priority 2
//! masked through BASEPRI. Between transactions the feed preempts freely.
//! Priority 3 (STEP/DIR, USB, STATUS) is never masked.
//!
//! The arbiter also passes every register write `tmc_control` sends to the
//! [`CountFeed`], which follows the count range and the alignment's zero.

use core::cell::UnsafeCell;
use core::mem::MaybeUninit;

use embedded_hal::spi::{ErrorType, Operation, SpiDevice};
use foci_firmware::absolute_encoder::CountFeed;

use crate::app_types::ExclusiveTmcSpiDevice;

/// BASEPRI value masking RTIC priority 2 and below on this 4-priority-bit
/// part (RTIC `cortex_logical2hw(2, 4)`).
const BASEPRI_MASK_P2: u8 = ((1 << 4) - 2) << 4;

const WRITE_FLAG: u8 = 0x80;
const ABN_DECODER_COUNT: u8 = foci_tmc4671::registers::ABN_DECODER_COUNT.addr();

struct Bus {
    /// Owns SPI1 and its pins; transfers go through `run_raw`.
    _device: ExclusiveTmcSpiDevice,
    feed: CountFeed,
}

struct BusCell(UnsafeCell<MaybeUninit<Bus>>);

// SAFETY: the bus is only touched from `tmc_control` (priority 1) with
// priority 2 masked, and from the feed interrupt (priority 2), which
// `tmc_control` cannot preempt. Nothing at priority 3 accesses it.
unsafe impl Sync for BusCell {}

static BUS: BusCell = BusCell(UnsafeCell::new(MaybeUninit::uninit()));

/// Move the SPI1 device into the arbiter and return the handle that
/// `tmc_control` drives the TMC4671 through. Call once from `init`.
pub(crate) fn install(device: ExclusiveTmcSpiDevice) -> ArbitratedSpi {
    // SAFETY: init runs once with interrupts masked, before any task.
    unsafe {
        (*BUS.0.get()).write(Bus {
            _device: device,
            feed: CountFeed::new(),
        });
    }
    ArbitratedSpi
}

/// Run `f` on the bus with priority 2 masked.
fn with_bus_masked<R>(f: impl FnOnce(&mut Bus) -> R) -> R {
    let saved = cortex_m::register::basepri::read();
    cortex_m::register::basepri_max::write(BASEPRI_MASK_P2);
    // SAFETY: `install` ran in init, and priority 2 is masked, so the feed
    // cannot hold a reference concurrently (see `BusCell`).
    let result = f(unsafe { (*BUS.0.get()).assume_init_mut() });
    // SAFETY: restores the caller's BASEPRI.
    unsafe { cortex_m::register::basepri::write(saved) };
    result
}

/// Publish a new absolute encoder reading and write it to
/// `ABN_DECODER_COUNT`. Call from the priority-2 feed interrupt only.
pub(crate) fn feed(position: u32, position_bits: u32) {
    // SAFETY: priority 2 is the highest priority touching the bus and
    // `tmc_control` only touches it with priority 2 masked.
    let bus = unsafe { (*BUS.0.get()).assume_init_mut() };
    let Some(count) = bus.feed.count(position, position_bits) else {
        return;
    };
    let bytes = count.to_be_bytes();
    let frame = [
        WRITE_FLAG | ABN_DECODER_COUNT,
        bytes[0],
        bytes[1],
        bytes[2],
        bytes[3],
    ];
    let mut operations = [Operation::Write(&frame)];
    run_raw(&mut operations);
}

/// SPI1 chip select (PA4); `ExclusiveDevice` only keeps the pins owned.
const TMC_CS_PIN: usize = 4;

/// Run a transaction straight through the SPI1 registers with CS held low.
/// The embassy blocking path costs ~17 us per 5-byte read against ~7 us
/// here, which matters with the encoder feed sharing the bus: the
/// gain-floor capture reads two registers every 60 us. SPI1 stays
/// configured (mode 3, 8 MHz) and enabled between transfers.
fn run_raw(operations: &mut [Operation<'_, u8>]) {
    let spi = embassy_stm32::pac::SPI1;
    let gpio = embassy_stm32::pac::GPIOA;
    let exchange = |byte: u8| -> u8 {
        while !spi.sr().read().txe() {}
        spi.dr().write(|w| w.set_dr(u16::from(byte)));
        while !spi.sr().read().rxne() {}
        spi.dr().read().dr() as u8
    };
    gpio.bsrr().write(|w| w.set_br(TMC_CS_PIN, true));
    for operation in operations.iter_mut() {
        match operation {
            Operation::Write(bytes) => {
                for &byte in bytes.iter() {
                    exchange(byte);
                }
            }
            Operation::Read(bytes) => {
                for byte in bytes.iter_mut() {
                    *byte = exchange(0);
                }
            }
            Operation::Transfer(read, write) => {
                for index in 0..read.len().max(write.len()) {
                    let received = exchange(write.get(index).copied().unwrap_or(0));
                    if let Some(slot) = read.get_mut(index) {
                        *slot = received;
                    }
                }
            }
            Operation::TransferInPlace(bytes) => {
                for byte in bytes.iter_mut() {
                    *byte = exchange(*byte);
                }
            }
            Operation::DelayNs(ns) => {
                while spi.sr().read().bsy() {}
                cortex_m::asm::delay(ns.saturating_mul(168) / 1_000 + 1);
            }
        }
    }
    while spi.sr().read().bsy() {}
    gpio.bsrr().write(|w| w.set_bs(TMC_CS_PIN, true));
}

/// Pass a register write from `tmc_control` to the count feed.
fn observe(bus: &mut Bus, operations: &[Operation<'_, u8>]) {
    if let [Operation::Write(frame)] = operations
        && let [address, b0, b1, b2, b3] = **frame
        && address & WRITE_FLAG != 0
    {
        bus.feed
            .observe_write(address & !WRITE_FLAG, u32::from_be_bytes([b0, b1, b2, b3]));
    }
}

/// `tmc_control`'s handle to the arbitrated SPI1 device.
pub(crate) struct ArbitratedSpi;

impl ErrorType for ArbitratedSpi {
    type Error = <ExclusiveTmcSpiDevice as ErrorType>::Error;
}

impl SpiDevice for ArbitratedSpi {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        with_bus_masked(|bus| {
            observe(bus, operations);
            run_raw(operations);
        });
        Ok(())
    }
}

impl embedded_hal_async::spi::SpiDevice for ArbitratedSpi {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        SpiDevice::transaction(self, operations)
    }
}
