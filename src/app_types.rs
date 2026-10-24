use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async as AsyncMode;
use embassy_stm32::spi::Spi;
use embedded_hal_bus::spi::ExclusiveDevice;

pub type Spi1Bus = Spi<'static, AsyncMode, embassy_stm32::spi::mode::Master>;

/// Busy-wait delay for the TMC4671 500 ns tSCKpause on SPI reads above 2 MHz.
pub struct CycleDelay;

impl embedded_hal_async::delay::DelayNs for CycleDelay {
    async fn delay_ns(&mut self, ns: u32) {
        let cycles = (ns as u64 * 168 / 1000) as u32;
        cortex_m::asm::delay(cycles);
    }
}

impl embedded_hal::delay::DelayNs for CycleDelay {
    fn delay_ns(&mut self, ns: u32) {
        let cycles = (ns as u64 * 168 / 1000) as u32;
        cortex_m::asm::delay(cycles);
    }
}

pub type ExclusiveTmcSpiDevice = ExclusiveDevice<Spi1Bus, Output<'static>, CycleDelay>;

#[cfg(not(feature = "hfkp-encoder"))]
pub type TmcSpiDevice = ExclusiveTmcSpiDevice;
/// The encoder feed shares SPI1 with `tmc_control` through the arbiter.
#[cfg(feature = "hfkp-encoder")]
pub type TmcSpiDevice = crate::tmc_bus::ArbitratedSpi;
