//! Inter-Integrated Circuit (I2C) bus
use core::ops::Deref;

use crate::stm32::i2c1;

use core::cmp;

use crate::stm32::{I2C1, I2C2, I2C3, RCC};

use crate::hal::blocking::i2c::{Read, Write, WriteRead};

use crate::gpio::{
    gpioa::PA8,
    gpiob::{PB10, PB11, PB3, PB4, PB6, PB7, PB8, PB9},
    gpioc::{PC12, PC9},
    gpiof::{PF0, PF1},
    gpioh::{PH4, PH5, PH7, PH8},
};
use crate::gpio::{Alternate, AF4, AF9};

use crate::rcc::Clocks;
use crate::time::{Hertz, KiloHertz, U32Ext};

/// I2C abstraction
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

pub trait Pins<I2c> {}
pub trait PinScl<I2c> {}
pub trait PinSda<I2c> {}

impl<I2c, SCL, SDA> Pins<I2c> for (SCL, SDA)
where
    SCL: PinScl<I2c>,
    SDA: PinSda<I2c>,
{
}

impl PinScl<I2C1> for PB6<Alternate<AF4>> {}
impl PinSda<I2C1> for PB7<Alternate<AF4>> {}
impl PinScl<I2C1> for PB8<Alternate<AF4>> {}
impl PinSda<I2C1> for PB9<Alternate<AF4>> {}
impl PinSda<I2C2> for PB3<Alternate<AF4>> {}
impl PinSda<I2C2> for PB3<Alternate<AF9>> {}
impl PinSda<I2C2> for PB9<Alternate<AF9>> {}
impl PinScl<I2C2> for PB10<Alternate<AF4>> {}
impl PinSda<I2C2> for PB11<Alternate<AF4>> {}
impl PinSda<I2C2> for PC12<Alternate<AF4>> {}
impl PinScl<I2C2> for PF1<Alternate<AF4>> {}
impl PinSda<I2C2> for PF0<Alternate<AF4>> {}
impl PinScl<I2C2> for PH4<Alternate<AF4>> {}
impl PinSda<I2C2> for PH5<Alternate<AF4>> {}
impl PinScl<I2C3> for PA8<Alternate<AF4>> {}
impl PinSda<I2C3> for PB4<Alternate<AF4>> {}
impl PinSda<I2C3> for PB4<Alternate<AF9>> {}
impl PinSda<I2C3> for PB8<Alternate<AF9>> {}
impl PinSda<I2C3> for PC9<Alternate<AF4>> {}
impl PinScl<I2C3> for PH7<Alternate<AF4>> {}
impl PinSda<I2C3> for PH8<Alternate<AF4>> {}

#[derive(Debug)]
pub enum Error {
    OVERRUN,
    NACK,
}

impl<PINS> I2c<I2C1, PINS> {
    pub fn i2c1(i2c: I2C1, pins: PINS, speed: KiloHertz, clocks: Clocks) -> Self
    where
        PINS: Pins<I2C1>,
    {
        // NOTE(unsafe) This executes only during initialisation
        let rcc = unsafe { &(*RCC::ptr()) };

        // Enable clock for I2C1
        rcc.apb1enr.modify(|_, w| w.i2c1en().set_bit());

        // Reset I2C1
        rcc.apb1rstr.modify(|_, w| w.i2c1rst().set_bit());
        rcc.apb1rstr.modify(|_, w| w.i2c1rst().clear_bit());

        let i2c = I2c { i2c, pins };
        i2c.i2c_init(speed, clocks.pclk1());
        i2c
    }
}

impl<PINS> I2c<I2C2, PINS> {
    pub fn i2c2(i2c: I2C2, pins: PINS, speed: KiloHertz, clocks: Clocks) -> Self
    where
        PINS: Pins<I2C2>,
    {
        // NOTE(unsafe) This executes only during initialisation
        let rcc = unsafe { &(*RCC::ptr()) };

        // Enable clock for I2C2
        rcc.apb1enr.modify(|_, w| w.i2c2en().set_bit());

        // Reset I2C2
        rcc.apb1rstr.modify(|_, w| w.i2c2rst().set_bit());
        rcc.apb1rstr.modify(|_, w| w.i2c2rst().clear_bit());

        let i2c = I2c { i2c, pins };
        i2c.i2c_init(speed, clocks.pclk1());
        i2c
    }
}

impl<PINS> I2c<I2C3, PINS> {
    pub fn i2c3(i2c: I2C3, pins: PINS, speed: KiloHertz, clocks: Clocks) -> Self
    where
        PINS: Pins<I2C3>,
    {
        // NOTE(unsafe) This executes only during initialisation
        let rcc = unsafe { &(*RCC::ptr()) };

        // Enable clock for I2C3
        rcc.apb1enr.modify(|_, w| w.i2c3en().set_bit());

        // Reset I2C3
        rcc.apb1rstr.modify(|_, w| w.i2c3rst().set_bit());
        rcc.apb1rstr.modify(|_, w| w.i2c3rst().clear_bit());

        let i2c = I2c { i2c, pins };
        i2c.i2c_init(speed, clocks.pclk1());
        i2c
    }
}

impl<I2C, PINS> I2c<I2C, PINS>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    fn i2c_init(&self, speed: KiloHertz, pclk: Hertz) {
        // TODO: update to STM32F7 family
        /*let speed: Hertz = speed.into();

        // Make sure the I2C unit is disabled so we can configure it
        self.i2c.cr1.modify(|_, w| w.pe().clear_bit());

        // Calculate settings for I2C speed modes
        let clock = pclk.0;
        let freq = clock / 1_000_000;
        assert!(freq >= 2 && freq <= 50);

        // Configure bus frequency into I2C peripheral
        /*self.i2c.cr2.write(|w| unsafe { w.freq().bits(freq as u8) });

        let trise = if speed <= 100.khz().into() {
            freq + 1
        } else {
            (freq * 300) / 1000 + 1
        };

        // Configure correct rise times
        self.i2c.trise.write(|w| w.trise().bits(trise as u8));*/

        // I2C clock control calculation
        if speed <= 100.khz().into() {
            let ccr = {
                let ccr = clock / (speed.0 * 2);
                if ccr < 4 {
                    4
                } else {
                    ccr
                }
            };

            // Set clock to standard mode with appropriate parameters for selected speed
            self.i2c.ccr.write(|w| unsafe {
                w.f_s()
                    .clear_bit()
                    .duty()
                    .clear_bit()
                    .ccr()
                    .bits(ccr as u16)
            });
        } else {
            const DUTYCYCLE: u8 = 0;
            if DUTYCYCLE == 0 {
                let ccr = clock / (speed.0 * 3);
                let ccr = if ccr < 1 { 1 } else { ccr };

                // Set clock to fast mode with appropriate parameters for selected speed (2:1 duty cycle)
                self.i2c.ccr.write(|w| unsafe {
                    w.f_s().set_bit().duty().clear_bit().ccr().bits(ccr as u16)
                });
            } else {
                let ccr = clock / (speed.0 * 25);
                let ccr = if ccr < 1 { 1 } else { ccr };

                // Set clock to fast mode with appropriate parameters for selected speed (16:9 duty cycle)
                self.i2c.ccr.write(|w| unsafe {
                    w.f_s().set_bit().duty().set_bit().ccr().bits(ccr as u16)
                });
            }
        }

        // Enable the I2C processing
        self.i2c.cr1.modify(|_, w| w.pe().set_bit());
        */
    }

    pub fn release(self) -> (I2C, PINS) {
        (self.i2c, self.pins)
    }
}

trait I2cCommon {
    fn send_byte(&self, byte: u8) -> Result<(), Error>;

    fn recv_byte(&self) -> Result<u8, Error>;
}

impl<I2C, PINS> I2cCommon for I2c<I2C, PINS>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    fn send_byte(&self, byte: u8) -> Result<(), Error> {
        /* Wait until we're ready for sending */
        while self.i2c.isr.read().txis().bit_is_clear() {}

        /* Push out a byte of data */
        self.i2c.txdr.write(|w| unsafe { w.bits(u32::from(byte)) });

        /* If we received a NACK, then this is an error */
        if self.i2c.isr.read().nackf().bit_is_set() {
            self.i2c
                .icr
                .write(|w| w.stopcf().set_bit().nackcf().set_bit());
            return Err(Error::NACK);
        }

        Ok(())
    }

    fn recv_byte(&self) -> Result<u8, Error> {
        while self.i2c.isr.read().rxne().bit_is_clear() {}
        let value = self.i2c.rxdr.read().bits() as u8;
        Ok(value)
    }
}

impl<I2C, PINS> WriteRead for I2c<I2C, PINS>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = Error;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.write(addr, bytes)?;
        self.read(addr, buffer)?;

        Ok(())
    }
}

impl<I2C, PINS> Write for I2c<I2C, PINS>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        /* Set up current address, we're trying a "read" command and not going to set anything
         * and make sure we end a non-NACKed read (i.e. if we found a device) properly */
        self.i2c.cr2.modify(|_, w| {
            w.sadd()
                .bits(addr.into())
                .nbytes()
                .bits(bytes.len() as u8)
                .rd_wrn()
                .clear_bit()
                .autoend()
                .set_bit()
        });

        /* Send a START condition */
        self.i2c.cr2.modify(|_, w| w.start().set_bit());

        for c in bytes {
            self.send_byte(*c)?;
        }

        /* Fallthrough is success */
        self.i2c
            .icr
            .write(|w| w.stopcf().set_bit().nackcf().set_bit());
        Ok(())
    }
}

impl<I2C, PINS> Read for I2c<I2C, PINS>
where
    I2C: Deref<Target = i2c1::RegisterBlock>,
{
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        if let Some((last, buffer)) = buffer.split_last_mut() {
            // TODO: update to STM32F7 family
            /*
            // Send a START condition and set ACK bit
            self.i2c.cr1.modify(|_, w| w.start().set_bit().ack().set_bit());

            // Wait until START condition was generated
            while self.i2c.isr.read().sb().bit_is_clear() {}

            // Also wait until signalled we're master and everything is waiting for us
            while {
                let sr2 = self.i2c.sr2.read();
                sr2.msl().bit_is_clear() && sr2.busy().bit_is_clear()
            } {}

            // Set up current address, we're trying to talk to
            self.i2c.txdr.write(|w| unsafe { w.bits((u32::from(addr) << 1) + 1) });

            // Wait until address was sent
            while self.i2c.isr.read().addr().bit_is_clear() {}

            // Clear condition by reading SR2
            self.i2c.sr2.read();

            // Receive bytes into buffer
            for c in buffer {
                *c = self.recv_byte()?;
            }

            // Prepare to send NACK then STOP after next byte
            self.i2c.cr1.modify(|_, w| w.ack().clear_bit().stop().set_bit());

            // Receive last byte
            *last = self.recv_byte()?;
            */

            // Fallthrough is success
            Ok(())
        } else {
            Err(Error::OVERRUN)
        }
    }
}
