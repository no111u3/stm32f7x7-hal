use core::marker::PhantomData;
use core::ptr;

use nb;
use void::Void;

use crate::hal;
use crate::hal::serial::Write;

use crate::pac::{RCC, UART4, UART5, USART1, USART2, USART3, USART6};

use crate::gpio::gpioa::{PA0, PA1, PA10, PA11, PA12, PA2, PA3, PA9};
use crate::gpio::gpiob::{PB10, PB11, PB12, PB13, PB14, PB15, PB5, PB6, PB7, PB8, PB9};
use crate::gpio::gpioc::{PC10, PC11, PC12, PC6, PC7};
use crate::gpio::gpiod::{PD0, PD1, PD2, PD5, PD6, PD8, PD9};
use crate::gpio::gpiog::{PG14, PG9};
use crate::gpio::gpioh::{PH13, PH14};
use crate::gpio::{Alternate, AF1, AF4, AF6, AF7, AF8};
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

pub trait Pins<USART> {}

impl Pins<USART1> for (PA9<Alternate<AF7>>, PA10<Alternate<AF7>>) {}
impl Pins<USART1> for (PB6<Alternate<AF7>>, PB7<Alternate<AF7>>) {}
impl Pins<USART1> for (PB14<Alternate<AF4>>, PB15<Alternate<AF4>>) {}

impl Pins<USART2> for (PA2<Alternate<AF7>>, PA3<Alternate<AF7>>) {}
impl Pins<USART2> for (PD5<Alternate<AF7>>, PD6<Alternate<AF7>>) {}

impl Pins<USART3> for (PB10<Alternate<AF7>>, PB11<Alternate<AF7>>) {}
impl Pins<USART3> for (PC10<Alternate<AF7>>, PC11<Alternate<AF7>>) {}
impl Pins<USART3> for (PD8<Alternate<AF7>>, PD9<Alternate<AF7>>) {}

impl Pins<UART4> for (PA0<Alternate<AF8>>, PA1<Alternate<AF8>>) {}
impl Pins<UART4> for (PA12<Alternate<AF6>>, PA11<Alternate<AF6>>) {}
impl Pins<UART4> for (PC10<Alternate<AF8>>, PC11<Alternate<AF8>>) {}
impl Pins<UART4> for (PD1<Alternate<AF8>>, PD0<Alternate<AF8>>) {}
impl Pins<UART4> for (PH13<Alternate<AF8>>, PH14<Alternate<AF8>>) {}

impl Pins<UART5> for (PB6<Alternate<AF1>>, PB5<Alternate<AF1>>) {}
impl Pins<UART5> for (PB9<Alternate<AF7>>, PB8<Alternate<AF7>>) {}
impl Pins<UART5> for (PB13<Alternate<AF8>>, PB12<Alternate<AF8>>) {}
impl Pins<UART5> for (PC12<Alternate<AF8>>, PD2<Alternate<AF8>>) {}

impl Pins<USART6> for (PC6<Alternate<AF8>>, PC7<Alternate<AF8>>) {}
impl Pins<USART6> for (PG14<Alternate<AF8>>, PG9<Alternate<AF8>>) {}

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

/// USART1
impl<PINS> Serial<USART1, PINS> {
    pub fn usart1(usart: USART1, pins: PINS, baud_rate: Bps, clocks: Clocks) -> Self
    where
        PINS: Pins<USART1>,
    {
        // NOTE(unsafe) This executes only during initialisation
        let rcc = unsafe { &(*RCC::ptr()) };

        // Use sysclock for baudrate
        rcc.dkcfgr2.modify(|_, w| unsafe { w.usart1sel().bits(1) });

        /* Enable clock for USART */
        rcc.apb2enr.modify(|_, w| w.usart1en().set_bit());

        // Calculate correct baudrate divisor on the fly
        let brr = clocks.sysclk().0 / baud_rate.0;
        usart.brr.write(|w| unsafe { w.bits(brr) });

        /* Reset other registers to disable advanced USART features */
        usart.cr2.reset();
        usart.cr3.reset();

        /* Enable transmission and receiving */
        usart.cr1.modify(|_, w| unsafe { w.bits(0xD) });

        Serial { usart, pins }
    }

    pub fn split(self) -> (Tx<USART1>, Rx<USART1>) {
        (
            Tx {
                _usart: PhantomData,
            },
            Rx {
                _usart: PhantomData,
            },
        )
    }
    pub fn release(self) -> (USART1, PINS) {
        (self.usart, self.pins)
    }
}

impl hal::serial::Read<u8> for Rx<USART1> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { (*USART1::ptr()).isr.read() };

        Err(if isr.pe().bit_is_set() {
            nb::Error::Other(Error::Parity)
        } else if isr.fe().bit_is_set() {
            nb::Error::Other(Error::Framing)
        } else if isr.nf().bit_is_set() {
            nb::Error::Other(Error::Noise)
        } else if isr.ore().bit_is_set() {
            nb::Error::Other(Error::Overrun)
        } else if isr.rxne().bit_is_set() {
            // NOTE(read_volatile) see `write_volatile` below
            return Ok(unsafe { ptr::read_volatile(&(*USART1::ptr()).rdr as *const _ as *const _) });
        } else {
            nb::Error::WouldBlock
        })
    }
}

impl hal::serial::Write<u8> for Tx<USART1> {
    type Error = Void;

    fn flush(&mut self) -> nb::Result<(), Void> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { (*USART1::ptr()).isr.read() };

        if isr.tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Void> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { (*USART1::ptr()).isr.read() };

        if isr.txe().bit_is_set() {
            // NOTE(unsafe) atomic write to stateless register
            // NOTE(write_volatile) 8-bit write that's not possible through the svd2rust API
            unsafe { ptr::write_volatile(&(*USART1::ptr()).tdr as *const _ as *mut _, byte) }
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

/// USART2
impl<PINS> Serial<USART2, PINS> {
    pub fn usart2(usart: USART2, pins: PINS, baud_rate: Bps, clocks: Clocks) -> Self
    where
        PINS: Pins<USART2>,
    {
        // NOTE(unsafe) This executes only during initialisation
        let rcc = unsafe { &(*RCC::ptr()) };

        // Use sysclock for baudrate
        rcc.dkcfgr2.modify(|_, w| unsafe { w.usart2sel().bits(1) });

        /* Enable clock for USART */
        rcc.apb1enr.modify(|_, w| w.usart2en().set_bit());

        // Calculate correct baudrate divisor on the fly
        let brr = clocks.sysclk().0 / baud_rate.0;
        usart.brr.write(|w| unsafe { w.bits(brr) });

        /* Reset other registers to disable advanced USART features */
        usart.cr2.reset();
        usart.cr3.reset();

        /* Enable transmission and receiving */
        usart.cr1.modify(|_, w| unsafe { w.bits(0xD) });

        Serial { usart, pins }
    }

    pub fn split(self) -> (Tx<USART2>, Rx<USART2>) {
        (
            Tx {
                _usart: PhantomData,
            },
            Rx {
                _usart: PhantomData,
            },
        )
    }
    pub fn release(self) -> (USART2, PINS) {
        (self.usart, self.pins)
    }
}

impl hal::serial::Read<u8> for Rx<USART2> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { (*USART2::ptr()).isr.read() };

        Err(if isr.pe().bit_is_set() {
            nb::Error::Other(Error::Parity)
        } else if isr.fe().bit_is_set() {
            nb::Error::Other(Error::Framing)
        } else if isr.nf().bit_is_set() {
            nb::Error::Other(Error::Noise)
        } else if isr.ore().bit_is_set() {
            nb::Error::Other(Error::Overrun)
        } else if isr.rxne().bit_is_set() {
            // NOTE(read_volatile) see `write_volatile` below
            return Ok(unsafe { ptr::read_volatile(&(*USART2::ptr()).rdr as *const _ as *const _) });
        } else {
            nb::Error::WouldBlock
        })
    }
}

impl hal::serial::Write<u8> for Tx<USART2> {
    type Error = Void;

    fn flush(&mut self) -> nb::Result<(), Void> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { (*USART2::ptr()).isr.read() };

        if isr.tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Void> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { (*USART2::ptr()).isr.read() };

        if isr.txe().bit_is_set() {
            // NOTE(unsafe) atomic write to stateless register
            // NOTE(write_volatile) 8-bit write that's not possible through the svd2rust API
            unsafe { ptr::write_volatile(&(*USART2::ptr()).tdr as *const _ as *mut _, byte) }
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

/// USART3
impl<PINS> Serial<USART3, PINS> {
    pub fn usart3(usart: USART3, pins: PINS, baud_rate: Bps, clocks: Clocks) -> Self
    where
        PINS: Pins<USART3>,
    {
        // NOTE(unsafe) This executes only during initialisation
        let rcc = unsafe { &(*RCC::ptr()) };

        // Use sysclock for baudrate
        rcc.dkcfgr2.modify(|_, w| unsafe { w.usart3sel().bits(1) });

        /* Enable clock for USART */
        rcc.apb1enr.modify(|_, w| w.usart3en().set_bit());

        // Calculate correct baudrate divisor on the fly
        let brr = clocks.sysclk().0 / baud_rate.0;
        usart.brr.write(|w| unsafe { w.bits(brr) });

        /* Reset other registers to disable advanced USART features */
        usart.cr2.reset();
        usart.cr3.reset();

        /* Enable transmission and receiving */
        usart.cr1.modify(|_, w| unsafe { w.bits(0xD) });

        Serial { usart, pins }
    }

    pub fn split(self) -> (Tx<USART3>, Rx<USART3>) {
        (
            Tx {
                _usart: PhantomData,
            },
            Rx {
                _usart: PhantomData,
            },
        )
    }
    pub fn release(self) -> (USART3, PINS) {
        (self.usart, self.pins)
    }
}

impl hal::serial::Read<u8> for Rx<USART3> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { (*USART3::ptr()).isr.read() };

        Err(if isr.pe().bit_is_set() {
            nb::Error::Other(Error::Parity)
        } else if isr.fe().bit_is_set() {
            nb::Error::Other(Error::Framing)
        } else if isr.nf().bit_is_set() {
            nb::Error::Other(Error::Noise)
        } else if isr.ore().bit_is_set() {
            nb::Error::Other(Error::Overrun)
        } else if isr.rxne().bit_is_set() {
            // NOTE(read_volatile) see `write_volatile` below
            return Ok(unsafe { ptr::read_volatile(&(*USART3::ptr()).rdr as *const _ as *const _) });
        } else {
            nb::Error::WouldBlock
        })
    }
}

impl hal::serial::Write<u8> for Tx<USART3> {
    type Error = Void;

    fn flush(&mut self) -> nb::Result<(), Void> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { (*USART3::ptr()).isr.read() };

        if isr.tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Void> {
        // NOTE(unsafe) atomic read with no side effects
        let isr = unsafe { (*USART3::ptr()).isr.read() };

        if isr.txe().bit_is_set() {
            // NOTE(unsafe) atomic write to stateless register
            // NOTE(write_volatile) 8-bit write that's not possible through the svd2rust API
            unsafe { ptr::write_volatile(&(*USART3::ptr()).tdr as *const _ as *mut _, byte) }
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<USART> core::fmt::Write for Tx<USART>
where
    Tx<USART>: hal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        s.as_bytes()
            .iter()
            .try_for_each(|c| nb::block!(self.write(*c)))
            .map_err(|_| core::fmt::Error)
    }
} 
