#![no_std]
#![allow(non_camel_case_types)]
#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(deprecated)]

pub use embedded_hal as hal;

pub use stm32f7::stm32f7x7 as stm32;

pub use nb;
pub use nb::block;

#[cfg(feature = "rt")]
pub use crate::stm32::interrupt;

pub mod adc;
pub mod delay;
pub mod gpio;
pub mod i2c;
pub mod prelude;
pub mod qei;
pub mod rcc;
pub mod serial;
pub mod signature;
pub mod spi;
pub mod time;
pub mod timer;
pub mod watchdog;
