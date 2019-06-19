//! Serial
use core::fmt;
use core::marker::PhantomData;
use core::ptr;

use embedded_hal::serial;
use embedded_hal::prelude::*;
use nb::block;

use crate::hal::serial::Write;

use crate::stm32::{RCC, UART4, UART5, USART1, USART2, USART3, USART6, UART7, UART8};

use crate::gpio::gpioa::{PA0, PA1, PA2, PA3, PA8, PA9, PA10, PA11, PA12, PA15};
use crate::gpio::gpiob::{PB3, PB4, PB10, PB11, PB12, PB13, PB5, PB6, PB7, PB8, PB9};
use crate::gpio::gpioc::{PC5, PC10, PC11, PC12, PC6, PC7};
use crate::gpio::gpiod::{PD0, PD1, PD2, PD5, PD6, PD8, PD9, PD10};
use crate::gpio::gpioe::{PE0, PE1, PE7, PE8};
use crate::gpio::gpiof::{PF6, PF7, PF8, PF9};
use crate::gpio::gpiog::{PG0, PG1, PG11, PG12, PG14, PG9};
use crate::gpio::gpioh::{PH13, PH14};
use crate::gpio::{Alternate, AF7, AF8, AF11};
use crate::rcc::Clocks;
use crate::time::Bps;

/// Serial error
#[derive(Debug)]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
    #[doc(hidden)]
    _Extensible,
}

/// Interrupt event
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
    /// Idle line state detected
    Idle,
}

pub mod config {
    use crate::time::Bps;
    use crate::time::U32Ext;

    pub enum WordLength {
        DataBits8,
        DataBits9,
    }

    pub enum Parity {
        ParityNone,
        ParityEven,
        ParityOdd,
    }

    pub enum StopBits {
        #[doc = "1 stop bit"]
        STOP1,
        #[doc = "0.5 stop bits"]
        STOP0P5,
        #[doc = "2 stop bits"]
        STOP2,
        #[doc = "1.5 stop bits"]
        STOP1P5,
    }

    pub struct Config {
        pub baudrate: Bps,
        pub wordlength: WordLength,
        pub parity: Parity,
        pub stopbits: StopBits,
    }

    impl Config {
        pub fn baudrate(mut self, baudrate: Bps) -> Self {
            self.baudrate = baudrate;
            self
        }

        pub fn parity_none(mut self) -> Self {
            self.parity = Parity::ParityNone;
            self
        }

        pub fn parity_even(mut self) -> Self {
            self.parity = Parity::ParityEven;
            self
        }

        pub fn parity_odd(mut self) -> Self {
            self.parity = Parity::ParityOdd;
            self
        }

        pub fn wordlength_8(mut self) -> Self {
            self.wordlength = WordLength::DataBits8;
            self
        }

        pub fn wordlength_9(mut self) -> Self {
            self.wordlength = WordLength::DataBits9;
            self
        }

        pub fn stopbits(mut self, stopbits: StopBits) -> Self {
            self.stopbits = stopbits;
            self
        }
    }

    #[derive(Debug)]
    pub struct InvalidConfig;

    impl Default for Config {
        fn default() -> Config {
            let baudrate = 19_200_u32.bps();
            Config {
                baudrate,
                wordlength: WordLength::DataBits8,
                parity: Parity::ParityNone,
                stopbits: StopBits::STOP1,
            }
        }
    }
}

pub trait Pins<USART> {}
pub trait PinTx<USART> {}
pub trait PinRx<USART> {}

impl<USART, TX, RX> Pins<USART> for (TX, RX)
where
    TX: PinTx<USART>,
    RX: PinRx<USART>,
{}

/// A filler type for when the Tx pin is unnecessary
pub struct NoTx;
/// A filler type for when the Rx pin is unnecessary
pub struct NoRx;

impl PinTx<USART1> for NoTx {}
impl PinRx<USART1> for NoRx {}

impl PinTx<USART1> for PA9<Alternate<AF7>> {}
impl PinRx<USART1> for PA10<Alternate<AF7>> {}
impl PinTx<USART1> for PA15<Alternate<AF7>> {}
impl PinRx<USART1> for PB3<Alternate<AF7>> {}
impl PinTx<USART1> for PB6<Alternate<AF7>> {}
impl PinRx<USART1> for PB7<Alternate<AF7>> {}

impl PinTx<USART2> for NoTx {}
impl PinRx<USART2> for NoRx {}

impl PinTx<USART2> for PA2<Alternate<AF7>> {}
impl PinRx<USART2> for PA3<Alternate<AF7>> {}
impl PinTx<USART2> for PD5<Alternate<AF7>> {}
impl PinRx<USART2> for PD6<Alternate<AF7>> {}

impl PinTx<USART3> for NoTx {}
impl PinRx<USART3> for NoRx {}

impl PinTx<USART3> for PB10<Alternate<AF7>> {}
impl PinRx<USART3> for PB11<Alternate<AF7>> {}
impl PinRx<USART3> for PC5<Alternate<AF7>> {}
impl PinTx<USART3> for PC10<Alternate<AF7>> {}
impl PinRx<USART3> for PC11<Alternate<AF7>> {}
impl PinTx<USART3> for PD8<Alternate<AF7>> {}
impl PinRx<USART3> for PD9<Alternate<AF7>> {}

impl PinTx<UART4> for NoTx {}
impl PinRx<UART4> for NoRx {}

impl PinTx<UART4> for PA0<Alternate<AF8>> {}
impl PinRx<UART4> for PA1<Alternate<AF8>> {}
impl PinTx<UART4> for PA12<Alternate<AF11>> {}
impl PinRx<UART4> for PA11<Alternate<AF11>> {}
impl PinTx<UART4> for PC10<Alternate<AF8>> {}
impl PinRx<UART4> for PC11<Alternate<AF8>> {}
impl PinTx<UART4> for PD1<Alternate<AF11>> {}
impl PinRx<UART4> for PD0<Alternate<AF11>> {}
impl PinTx<UART4> for PD10<Alternate<AF8>> {}

impl PinTx<UART5> for NoTx {}
impl PinRx<UART5> for NoRx {}

impl PinTx<UART5> for PB6<Alternate<AF11>> {}
impl PinRx<UART5> for PB5<Alternate<AF11>> {}
impl PinTx<UART5> for PB9<Alternate<AF11>> {}
impl PinRx<UART5> for PB8<Alternate<AF11>> {}
impl PinTx<UART5> for PB13<Alternate<AF11>> {}
impl PinRx<UART5> for PB12<Alternate<AF11>> {}
impl PinTx<UART5> for PC12<Alternate<AF8>> {}
impl PinRx<UART5> for PD2<Alternate<AF8>> {}
impl PinTx<UART5> for PE8<Alternate<AF8>> {}
impl PinRx<UART5> for PE7<Alternate<AF8>> {}

impl PinTx<USART6> for NoTx {}
impl PinRx<USART6> for NoRx {}

impl PinTx<USART6> for PA11<Alternate<AF8>> {}
impl PinRx<USART6> for PA12<Alternate<AF8>> {}
impl PinTx<USART6> for PC6<Alternate<AF8>> {}
impl PinRx<USART6> for PC7<Alternate<AF8>> {}
impl PinTx<USART6> for PG14<Alternate<AF8>> {}
impl PinRx<USART6> for PG9<Alternate<AF8>> {}

impl PinTx<UART7> for NoTx {}
impl PinRx<UART7> for NoRx {}

impl PinTx<UART7> for PA15<Alternate<AF8>> {}
impl PinRx<UART7> for PA8<Alternate<AF8>> {}
impl PinTx<UART7> for PB4<Alternate<AF8>> {}
impl PinRx<UART7> for PB3<Alternate<AF8>> {}
impl PinTx<UART7> for PE8<Alternate<AF8>> {}
impl PinRx<UART7> for PE7<Alternate<AF8>> {}
impl PinTx<UART7> for PF7<Alternate<AF8>> {}
impl PinRx<UART7> for PF6<Alternate<AF8>> {}

impl PinTx<UART8> for NoTx {}
impl PinRx<UART8> for NoRx {}

impl PinTx<UART8> for PE1<Alternate<AF8>> {}
impl PinRx<UART8> for PE0<Alternate<AF8>> {}
impl PinTx<UART8> for PF9<Alternate<AF8>> {}
impl PinRx<UART8> for PF8<Alternate<AF8>> {}

/// Serial abstraction
pub struct Serial<USART, PINS> {
    usart: USART,
    pins: PINS,
}

/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

macro_rules! halUsartImpl {
    ($(
        $USARTX:ident: ($usartX:ident, $apbXenr:ident, $usartXen:ident,  $pclkX:ident),
    )+) => {
        $(
            impl<PINS> Serial<$USARTX, PINS> {
                pub fn $usartX(
                    usart: $USARTX,
                    pins: PINS,
                    config: config::Config,
                    clocks: Clocks,
                ) -> Result<Self, config::InvalidConfig>
                where
                    PINS: Pins<$USARTX>,
                {
                    use self::config::*;

                    // NOTE(unsafe) This executes only during initialisation
                    let rcc = unsafe { &(*RCC::ptr()) };

                    // Enable clock for USART
                    rcc.$apbXenr.modify(|_, w| w.$usartXen().set_bit());

                    // Calculate correct baudrate divisor on the fly
                    let div = (clocks.$pclkX().0 + config.baudrate.0 / 2)
                        / config.baudrate.0;
                    usart.brr.write(|w| unsafe { w.bits(div) });

                    // Reset other registers to disable advanced USART features
                    usart.cr2.reset();
                    usart.cr3.reset();

                    // Enable transmission and receiving
                    // and configure frame
                    usart.cr1.write(|w| {
                        w.ue()
                            .set_bit()
                            .te()
                            .set_bit()
                            .re()
                            .set_bit()
                            .m0()
                            .bit(match config.wordlength {
                                WordLength::DataBits8 => false,
                                WordLength::DataBits9 => true,
                            }).pce()
                            .bit(match config.parity {
                                Parity::ParityNone => false,
                                _ => true,
                            }).ps()
                            .bit(match config.parity {
                                Parity::ParityOdd => true,
                                _ => false,
                            })
                    });

                    Ok(Serial { usart, pins }.config_stop(config))
                }

                /// Starts listening for an interrupt event
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::Rxne => {
                            self.usart.cr1.modify(|_, w| w.rxneie().set_bit())
                        },
                        Event::Txe => {
                            self.usart.cr1.modify(|_, w| w.txeie().set_bit())
                        },
                        Event::Idle => {
                            self.usart.cr1.modify(|_, w| w.idleie().set_bit())
                        },
                    }
                }

                /// Stop listening for an interrupt event
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::Rxne => {
                            self.usart.cr1.modify(|_, w| w.rxneie().clear_bit())
                        },
                        Event::Txe => {
                            self.usart.cr1.modify(|_, w| w.txeie().clear_bit())
                        },
                        Event::Idle => {
                            self.usart.cr1.modify(|_, w| w.idleie().clear_bit())
                        },
                    }
                }

                /// Return true if the line idle status is set
                pub fn is_idle(& self) -> bool {
                    unsafe { (*$USARTX::ptr()).isr.read().idle().bit_is_set() }
                }

                /// Return true if the tx register is empty (and can accept data)
                pub fn is_txe(& self) -> bool {
                    unsafe { (*$USARTX::ptr()).isr.read().txe().bit_is_set() }
                }

                /// Return true if the rx register is not empty (and can be read)
                pub fn is_rxne(& self) -> bool {
                    unsafe { (*$USARTX::ptr()).isr.read().rxne().bit_is_set() }
                }

                pub fn split(self) -> (Tx<$USARTX>, Rx<$USARTX>) {
                    (
                        Tx {
                            _usart: PhantomData,
                        },
                        Rx {
                            _usart: PhantomData,
                        },
                    )
                }
                pub fn release(self) -> ($USARTX, PINS) {
                    (self.usart, self.pins)
                }
            }

            impl<PINS> serial::Read<u8> for Serial<$USARTX, PINS> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    let mut rx: Rx<$USARTX> = Rx {
                        _usart: PhantomData,
                    };
                    rx.read()
                }
            }

            impl serial::Read<u8> for Rx<$USARTX> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let sr = unsafe { (*$USARTX::ptr()).isr.read() };

                    // Any error requires the dr to be read to clear
                    if sr.pe().bit_is_set()
                        || sr.fe().bit_is_set()
                        || sr.nf().bit_is_set()
                        || sr.ore().bit_is_set()
                    {
                        unsafe { (*$USARTX::ptr()).rdr.read() };
                    }

                    Err(if sr.pe().bit_is_set() {
                        nb::Error::Other(Error::Parity)
                    } else if sr.fe().bit_is_set() {
                        nb::Error::Other(Error::Framing)
                    } else if sr.nf().bit_is_set() {
                        nb::Error::Other(Error::Noise)
                    } else if sr.ore().bit_is_set() {
                        nb::Error::Other(Error::Overrun)
                    } else if sr.rxne().bit_is_set() {
                        // NOTE(read_volatile) see `write_volatile` below
                        return Ok(unsafe { ptr::read_volatile(&(*$USARTX::ptr()).rdr as *const _ as *const _) });
                    } else {
                        nb::Error::WouldBlock
                    })
                }
            }

            impl<PINS> serial::Write<u8> for Serial<$USARTX, PINS> {
                type Error = Error;

                fn flush(&mut self) -> nb::Result<(), Self::Error> {
                    let mut tx: Tx<$USARTX> = Tx {
                        _usart: PhantomData,
                    };
                    tx.flush()
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                    let mut tx: Tx<$USARTX> = Tx {
                        _usart: PhantomData,
                    };
                    tx.write(byte)
                }
            }

            impl serial::Write<u8> for Tx<$USARTX> {
                type Error = Error;

                fn flush(&mut self) -> nb::Result<(), Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let sr = unsafe { (*$USARTX::ptr()).isr.read() };

                    if sr.tc().bit_is_set() {
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let sr = unsafe { (*$USARTX::ptr()).isr.read() };

                    if sr.txe().bit_is_set() {
                        // NOTE(unsafe) atomic write to stateless register
                        // NOTE(write_volatile) 8-bit write that's not possible through the svd2rust API
                        unsafe { ptr::write_volatile(&(*$USARTX::ptr()).tdr as *const _ as *mut _, byte) }
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }
            }
        )+
    }
}

macro_rules! halUsart {
    ($(
        $USARTX:ident: ($usartX:ident, $apbXenr:ident, $usartXen:ident, $pclkX:ident),
    )+) => {
        $(
        impl<PINS> Serial<$USARTX, PINS> {
            fn config_stop(self, config: config::Config) -> Self {
                use crate::stm32::usart1::cr2::STOPW;
                use self::config::*;

                self.usart.cr2.write(|w| {
                    w.stop().variant(match config.stopbits {
                        StopBits::STOP0P5 => STOPW::STOP0P5,
                        StopBits::STOP1 => STOPW::STOP1,
                        StopBits::STOP1P5 => STOPW::STOP1P5,
                        StopBits::STOP2 => STOPW::STOP2,
                    })
                });
                self
            }
        }
        )+

        halUsartImpl! {
            $( $USARTX: ($usartX, $apbXenr, $usartXen, $pclkX), )+
        }
    }
}

macro_rules! halUart {
    ($(
        $USARTX:ident: ($usartX:ident, $apbXenr:ident, $usartXen:ident, $pclkX:ident),
    )+) => {
        $(
        impl<PINS> Serial<$USARTX, PINS> {
            fn config_stop(self, config: config::Config) -> Self {
                use crate::stm32::usart1::cr2::STOPW;
                use self::config::*;

                self.usart.cr2.write(|w| {
                    w.stop().variant(match config.stopbits {
                        StopBits::STOP0P5 => STOPW::STOP0P5,
                        StopBits::STOP1 => STOPW::STOP1,
                        StopBits::STOP1P5 => STOPW::STOP1P5,
                        StopBits::STOP2 => STOPW::STOP2,
                    })
                });
                self
            }
        }
        )+

        halUsartImpl! {
            $( $USARTX: ($usartX, $apbXenr, $usartXen, $pclkX), )+
        }
    }
}

halUsart! {
    USART1: (usart1, apb2enr, usart1en, pclk2),
    USART2: (usart2, apb1enr, usart2en, pclk1),
    USART6: (usart6, apb2enr, usart6en, pclk2),
}


halUsart! {
    USART3: (usart3, apb1enr, usart3en, pclk1),
}

halUart! {
    UART4: (uart4, apb1enr, uart4en, pclk1),
    UART5: (uart5, apb1enr, uart5en, pclk1),
}

halUsart! {
    UART7: (uart7, apb1enr, uart7en, pclk1),
    UART8: (uart8, apb1enr, uart8en, pclk1),
}


impl<USART> fmt::Write for Tx<USART>
where
    Tx<USART>: serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let _ = s
            .as_bytes()
            .iter()
            .map(|c| block!(self.write(*c)))
            .last();
        Ok(())
    }
}

