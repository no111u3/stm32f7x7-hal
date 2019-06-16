#![no_std]

use embedded_hal as hal;

pub use stm32f7::stm32f7x7 as pac;

pub use crate::pac as device;
pub use crate::pac as stm32;

pub mod delay;
pub mod gpio;
pub mod i2c;
pub mod prelude;
pub mod rcc;
pub mod serial;
pub mod time;
pub mod timer; 
