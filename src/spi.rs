//! Serial Peripheral Interface (SPI) bus

use core::ptr;

use embedded_hal::spi::{FullDuplex, Mode, Phase, Polarity};
use nb;
use crate::pac::{SPI1, SPI2, SPI3};

use crate::gpio::gpioa::{PA5, PA6, PA7};
use crate::gpio::gpiob::{PB10, PB15};
use crate::gpio::gpioc::{PC10, PC11, PC12, PC2};
use crate::gpio::{Alternate, AF5};
use crate::rcc::{APB1, APB2, Clocks};
use crate::time::Hertz;

/// SPI error
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
    #[doc(hidden)]
    _Extensible,
}

// FIXME these should be "closed" traits
/// SCK pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SckPin<SPI> {}

/// MISO pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait MisoPin<SPI> {}

/// MOSI pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait MosiPin<SPI> {}

// SPI1_SSEL - PA4
unsafe impl SckPin<SPI1> for PA5<Alternate<AF5>> {}
unsafe impl MisoPin<SPI1> for PA6<Alternate<AF5>> {}
unsafe impl MosiPin<SPI1> for PA7<Alternate<AF5>> {}

// SPI2_SSEL - PB4
unsafe impl SckPin<SPI2> for PB10<Alternate<AF5>> {}
unsafe impl MisoPin<SPI2> for PC2<Alternate<AF5>> {}
unsafe impl MosiPin<SPI2> for PB15<Alternate<AF5>> {}

// SPI3_SSEL - PA15
unsafe impl SckPin<SPI3> for PC10<Alternate<AF5>> {}
unsafe impl MisoPin<SPI3> for PC11<Alternate<AF5>> {}
unsafe impl MosiPin<SPI3> for PC12<Alternate<AF5>> {}

/// SPI peripheral operating in full duplex master mode
pub struct Spi<SPI, PINS> {
    spi: SPI,
    pins: PINS,
}

macro_rules! hal {
    ($($SPIX:ident: ($spiX:ident, $spiXen:ident, $spiXrst:ident, $APB:ident),)+) => {
        $(
            impl<SCK, MISO, MOSI> Spi<$SPIX, (SCK, MISO, MOSI)> {
                pub fn $spiX(
                    spi: $SPIX,
                    pins: (SCK, MISO, MOSI),
                    mode: Mode,
                    freq: Hertz,
                    clocks: Clocks,
                    apb: &mut $APB,
                ) -> Self
                where
                    SCK: SckPin<$SPIX>,
                    MISO: MisoPin<$SPIX>,
                    MOSI: MosiPin<$SPIX>,
                {
                    // enable or reset $SPIX
                    apb.enr().modify(|_, w| w.$spiXen().enabled());
                    apb.rstr().modify(|_, w| w.$spiXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$spiXrst().clear_bit());

                    // TODO - probably should do a proper shutdown, 35.5.9
                    // disable the SPI peripheral
                    spi.cr1.write(|w| w.spe().clear_bit());

                    // disable SS output
                    spi.cr2.write(|w| w.ssoe().clear_bit());

                    let br = match clocks.pclk2().0 / freq.0 {
                        0 => unreachable!(),
                        1...2 => 0b000, // pclk/2
                        3...5 => 0b001, // pclk/4
                        6...11 => 0b010,
                        12...23 => 0b011,
                        24...47 => 0b100,
                        48...95 => 0b101,
                        96...191 => 0b110, // pclk/128
                        _ => 0b111, // pclk/256
                    };

                    // mstr: master configuration
                    // lsbfirst: MSB first
                    // ssm: enable software slave management (NSS pin free for other uses)
                    // ssi: set nss high = master mode
                    // bidimode: 2-line unidirectional
                    // spe: enable the SPI bus
                    spi.cr1.write(|w| {
                        w.cpha().bit(mode.phase == Phase::CaptureOnSecondTransition)
                        .cpol().bit(mode.polarity == Polarity::IdleHigh)
                        .mstr().set_bit()
                        .br().bits(br)
                        .lsbfirst().clear_bit()
                        .ssm().set_bit()
                        .ssi().set_bit()
                        .rxonly().clear_bit()
                        //
                        // transit-only
                        .bidioe().set_bit()
                        .bidimode().set_bit()
                        //
                        .spe().set_bit()
                    });

                    // ds: 8 bit frames
                    spi.cr2.write(|w| { unsafe { w
                        .ds().bits(0b111)
                    }});

                    // TODO - forcing tx only until a proper half-duplex impl is done
                    Spi { spi, pins }
                }

                pub fn free(self) -> ($SPIX, (SCK, MISO, MOSI)) {
                    (self.spi, self.pins)
                }
            }

            impl<SCK, MISO, MOSI> FullDuplex<u8> for Spi<$SPIX, (SCK, MISO, MOSI)> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    let sr = self.spi.sr.read();

                    Err(if sr.ovr().bit_is_set() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.modf().bit_is_set() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().bit_is_set() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.rxne().bit_is_set() {
                        // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
                        // reading a half-word)
                        return Ok(unsafe {
                            ptr::read_volatile(&self.spi.dr as *const _ as *const u8)
                        });
                    } else {
                        nb::Error::WouldBlock
                    })
                }

                fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
                    let sr = self.spi.sr.read();

                    Err(if sr.ovr().bit_is_set() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.modf().bit_is_set() {
                        nb::Error::Other(Error::ModeFault)
                    } else if sr.crcerr().bit_is_set() {
                        nb::Error::Other(Error::Crc)
                    } else if sr.txe().bit_is_set() {
                        // NOTE(write_volatile) see note above
                        unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, byte) }
                        return Ok(());
                    } else {
                        nb::Error::WouldBlock
                    })
                }

            }

            impl<SCK, MISO, MOSI> crate::hal::blocking::spi::transfer::Default<u8> for Spi<$SPIX, (SCK, MISO, MOSI)> {}

            impl<SCK, MISO, MOSI> crate::hal::blocking::spi::write::Default<u8> for Spi<$SPIX, (SCK, MISO, MOSI)> {}
        )+
    }
}

hal! {
    SPI1: (spi1, spi1en, spi1rst, APB2),
    SPI2: (spi2, spi2en, spi2rst, APB1),
    SPI3: (spi3, spi3en, spi3rst, APB1),
}
