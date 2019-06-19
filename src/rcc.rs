//! Reset & Control Clock

use crate::stm32::{RCC, FLASH};

use crate::time::Hertz;

/// LSI clock frequency is approximately 32 KHz.
///
/// Note - this is not very accurate.
/// It is recommended to use TIM5 to measure the LSI frequency
/// for accurate
pub const LSI: u32 = 32_000_000;

/// HSI default clock speed is 16 MHz
pub const HSI: u32 = 16_000_000;

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            cfgr: CFGR {
                hclk: None,
                pclk1: None,
                pclk2: None,
                sysclk: None,
            },
        }
    }
}

/// Constrained RCC peripheral
pub struct Rcc {
    /// Clock configuration
    pub cfgr: CFGR,
}


/// Clock configuration
pub struct CFGR {
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
}

impl CFGR {
    /// Sets a frequency for the AHB bus
    pub fn hclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.hclk = Some(freq.into().0);
        self
    }

    /// Sets a frequency for the APB1 bus
    pub fn pclk1<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk1 = Some(freq.into().0);
        self
    }

    /// Sets a frequency for the APB2 bus
    pub fn pclk2<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk2 = Some(freq.into().0);
        self
    }

    /// Sets the system (core) frequency
    pub fn sysclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.sysclk = Some(freq.into().0);
        self
    }

    // @brief  System Clock Configuration
    //         The system Clock is configured as follow :
    //            System Clock source            = PLL (HSE)
    //            SYSCLK(Hz)                     = 216000000
    //            HCLK(Hz)                       = 216000000
    //            AHB Prescaler                  = 1
    //            APB1 Prescaler                 = 4
    //            APB2 Prescaler                 = 2
    //            HSE Frequency(Hz)              = 25000000
    //            PLL_M                          = 8
    //            PLL_N                          = 432
    //            PLL_P                          = 2
    //            PLL_Q                          = 9
    //            PLL_R                          = 7
    //            VDD(V)                         = 3.3
    //            Main regulator output voltage  = Scale1 mode
    //            Flash Latency(WS)              = 7
    //
    // TODO - configs/timeout/result?
    /*
    pub fn freeze_max(self, acr: &mut ACR) -> Clocks {
        let rcc = unsafe { &*RCC::ptr() };
        let pll_m = 8;
        let pll_n = 432;
        let pll_q = 9;
        let pll_r = 7;

        // enable power control clock
        rcc.apb1enr.modify(|_, w| w.pwren().set_bit());

        // TODO - needed?
        // enable voltage scaling

        // enable HSE oscillator and activate PLL with HSE as source
        rcc.cr.modify(|_, w| w.hseon().set_bit());

        // wait until HSE is ready
        while rcc.cr.read().hserdy().bit() == false {}

        // if the PLL is not used as system clock
        if rcc.cfgr.read().sws().bits() != 0b10 {
            // disable main PLL
            rcc.cr.modify(|_, w| w.pllon().clear_bit());

            // configure main PLL clock source
            rcc.pllcfgr.modify(|_, w| unsafe {
                w
                    // HSE PLL source
                    .pllsrc()
                    .set_bit()
                    .pllm()
                    .bits(pll_m)
                    .plln()
                    .bits(pll_n)
                    // PLLP_DIV2
                    .pllp()
                    .bits(0b00)
                    .pllq()
                    .bits(pll_q)
                    .pllr()
                    .bits(pll_r)
            });

            // enable main PLL
            rcc.cr.modify(|_, w| w.pllon().set_bit());

            // wait until PLL is ready
            while rcc.cr.read().pllrdy().bit() == false {}
        }

        // TODO - not neede for voltage scale 1?
        // activate OverDrive
        // HAL_PWREx_EnableOverDrive

        // set flash latency wait states
        acr.acr().modify(|_, w| w.latency().bits(7));

        // TODO - should read back out and check?

        // HCLK config
        // no prescaler
        rcc.cfgr.modify(|_, w| unsafe { w.hpre().bits(0b0000) });

        // SYSCLK config
        // wait for PLL ready
        while rcc.cr.read().pllrdy().bit() == false {}
        // set clock source, PLL
        rcc.cfgr.modify(|_, w| unsafe { w.sw().bits(0b10) });

        // wait for it
        while rcc.cfgr.read().sws().bits() != 0b10 {}

        rcc.cfgr.modify(|_, w| unsafe {
            w
                // PCLK1, DIV4
                .ppre1()
                .bits(0b101)
                // PCLK2, DIV2
                .ppre2()
                .bits(0b100)
        });

        // TODO
        let sysclk = 216_000_000;
        let hclk = sysclk;
        let ppre1: u32 = 4;
        let ppre2: u32 = 2;

        Clocks {
            hclk: Hertz(hclk),
            pclk1: Hertz(hclk / ppre1),
            pclk2: Hertz(hclk / ppre2),
            ppre1: ppre1 as _,
            ppre2: ppre2 as _,
            sysclk: Hertz(sysclk),
        }
    }*/

    /// Freezes the clock configuration, making it effective
    pub fn freeze(self) -> Clocks {
        let rcc = unsafe { &*RCC::ptr() };
        let flash = unsafe { &*FLASH::ptr() };

        let sysclk = self.sysclk.unwrap_or(HSI);
        let hclk = self.hclk.unwrap_or(HSI);

        assert!(sysclk >= HSI);
        assert!(hclk <= sysclk);

        if sysclk == HSI && hclk == sysclk {
            // use HSI as source and run everything at the same speed
            rcc.cfgr.modify(|_, w| unsafe {
                w.ppre2().bits(0).ppre1().bits(0).hpre().bits(0).sw().hsi()
            });

            Clocks {
                hclk: Hertz(hclk),
                pclk1: Hertz(hclk),
                pclk2: Hertz(hclk),
                ppre1: 1,
                ppre2: 1,
                sysclk: Hertz(sysclk),
            }
        } else if sysclk == HSI && hclk < sysclk {
            let hpre_bits = match sysclk / hclk {
                0 => unreachable!(),
                1 => 0b0111,
                2 => 0b1000,
                3...5 => 0b1001,
                6...11 => 0b1010,
                12...39 => 0b1011,
                40...95 => 0b1100,
                96...191 => 0b1101,
                192...383 => 0b1110,
                _ => 0b1111,
            };

            // Use HSI as source and run everything at the same speed
            rcc.cfgr.modify(|_, w| unsafe {
                w.ppre2()
                    .bits(0)
                    .ppre1()
                    .bits(0)
                    .hpre()
                    .bits(hpre_bits)
                    .sw()
                    .hsi()
            });

            Clocks {
                hclk: Hertz(hclk),
                pclk1: Hertz(hclk),
                pclk2: Hertz(hclk),
                ppre1: 1,
                ppre2: 1,
                sysclk: Hertz(sysclk),
            }
        } else {
            assert!(sysclk <= 216_000_000 && sysclk >= 24_000_000);

            // We're not diving down the hclk so it'll be the same as sysclk
            let hclk = sysclk;

            let (pllm, plln, pllp) = if sysclk >= 96_000_000 {
                // Input divisor from HSI clock, must result in less than 2MHz
                let pllm = 16;

                // Main scaler, must result in >= 192MHz and <= 432MHz, min 50, max 432
                let plln = (sysclk / 1_000_000) * 2;

                // Sysclk output divisor, must result in >= 24MHz and <= 216MHz
                // needs to be the equivalent of 2, 4, 6 or 8
                let pllp = 0;

                (pllm, plln, pllp)
            } else if sysclk <= 54_000_000 {
                // Input divisor from HSI clock, must result in less than 2MHz
                let pllm = 16;

                // Main scaler, must result in >= 192MHz and <= 432MHz, min 50, max 432
                let plln = (sysclk / 1_000_000) * 8;

                // Sysclk output divisor, must result in >= 24MHz and <= 216MHz
                // needs to be the equivalent of 2, 4, 6 or 8
                let pllp = 0b11;

                (pllm, plln, pllp)
            } else {
                // Input divisor from HSI clock, must result in less than 2MHz
                let pllm = 16;

                // Main scaler, must result in >= 192MHz and <= 432MHz, min 50, max 432
                let plln = (sysclk / 1_000_000) * 4;

                // Sysclk output divisor, must result in >= 24MHz and <= 216MHz
                // needs to be the equivalent of 2, 4, 6 or 8
                let pllp = 0b1;

                (pllm, plln, pllp)
            };

            let ppre2_bits = if sysclk > 108_000_000 { 0b100 } else { 0 };
            let ppre1_bits = if sysclk > 108_000_000 {
                0b101
            } else if sysclk > 54_000_000 {
                0b100
            } else {
                0
            };

            // Calculate real divisor
            let ppre1 = 1 << (ppre1_bits - 0b011);
            let ppre2 = 1 << (ppre2_bits - 0b011);

            // Calculate new bus clocks
            let pclk1 = hclk / u32::from(ppre1);
            let pclk2 = hclk / u32::from(ppre2);

            // Adjust flash wait states
            flash.acr.write(|w| {
                w.latency().bits(if sysclk <= 30_000_000 {
                    0b0000
                } else if sysclk <= 60_000_000 {
                    0b0001
                } else if sysclk <= 90_000_000 {
                    0b0010
                } else if sysclk <= 120_000_000 {
                    0b0011
                } else if sysclk <= 150_000_000 {
                    0b0100
                } else if sysclk <= 180_000_000 {
                    0b0101
                } else if sysclk <= 210_000_000 {
                    0b0110
                } else {
                    0b0111
                })
            });

            // use PLL as source
            rcc.pllcfgr.write(|w| unsafe {
                w.pllm()
                    .bits(pllm)
                    .plln()
                    .bits(plln as u16)
                    .pllp()
                    .bits(pllp)
            });

            // Enable PLL
            rcc.cr.write(|w| w.pllon().set_bit());

            // Wait for PLL to stabilise
            while rcc.cr.read().pllrdy().bit_is_clear() {}

            // Set scaling factors and switch clock to PLL
            rcc.cfgr.modify(|_, w| unsafe {
                w.ppre2()
                    .bits(ppre2_bits)
                    .ppre1()
                    .bits(ppre1_bits)
                    .hpre()
                    .bits(0)
                    .sw()
                    .pll()
            });

            Clocks {
                hclk: Hertz(hclk),
                pclk1: Hertz(pclk1),
                pclk2: Hertz(pclk2),
                ppre1,
                ppre2,
                sysclk: Hertz(sysclk),
            }
        }
    }
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
#[derive(Clone, Copy)]
pub struct Clocks {
    hclk: Hertz,
    pclk1: Hertz,
    pclk2: Hertz,
    ppre1: u8,
    ppre2: u8,
    sysclk: Hertz,
}

impl Clocks {
    /// Returns the frequency of the AHB1
    pub fn hclk(&self) -> Hertz {
        self.hclk
    }

    /// Returns the frequency of the APB1
    pub fn pclk1(&self) -> Hertz {
        self.pclk1
    }

    /// Returns the frequency of the APB2
    pub fn pclk2(&self) -> Hertz {
        self.pclk2
    }

    /// Returns the prescaler of the APB1
    pub fn ppre1(&self) -> u8 {
        self.ppre1
    }

    /// Returns the prescaler of the APB2
    pub fn ppre2(&self) -> u8 {
        self.ppre2
    }

    /// Returns the system (core) frequency
    pub fn sysclk(&self) -> Hertz {
        self.sysclk
    }
} 

/// TODO
#[derive(Copy, Clone, Debug)]
pub struct ResetConditions {
    pub low_power: bool,
    pub window_watchdog: bool,
    pub independent_watchdog: bool,
    pub software: bool,
    pub por_pdr: bool,
    pub pin: bool,
    pub bor: bool,
}

impl ResetConditions {
    pub fn read_and_clear() -> Self {
        let csr = unsafe { &(*RCC::ptr()).csr };
        let rc = ResetConditions {
            low_power: csr.read().lpwrrstf().bit(),
            window_watchdog: csr.read().wwdgrstf().bit(),
            independent_watchdog: csr.read().wdgrstf().bit(),
            software: csr.read().sftrstf().bit(),
            por_pdr: csr.read().porrstf().bit(),
            pin: csr.read().padrstf().bit(),
            bor: csr.read().borrstf().bit(),
        };

        // clear the reset flags
        csr.modify(|_, w| w.rmvf().set_bit());

        rc
    }
}
