#![no_std]
#![no_main]


extern crate panic_itm; // logs messages over ITM; requires ITM support

use cortex_m::{asm, iprintln, iprint, Peripherals as core_peripherals};
use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
use stm32f3::stm32f303;

mod my_api;
mod sensor;

#[entry]
fn main() -> ! {

    let periph = stm32f303::Peripherals::take().unwrap();
    let mut core_p = core_peripherals::take().unwrap();

    let itm = &mut core_p.ITM.stim[0];
    

    let gpioa = periph.GPIOA;
    let gpioe = periph.GPIOE;
    let rcc = periph.RCC;
    let flash = periph.FLASH;
    let spi1 = periph.SPI1;

    set_to_72MHz(&rcc, &flash);

    let spi = my_api::spi_mod::spi_func::new(&rcc, &gpioa, &gpioe, &spi1);

    let mut gyro = sensor::Gyro::gyro::new(&spi, &spi1);
    
    loop {
    
        iprintln!(itm, "{:?}", gyro.read_temp(&spi, &spi1, &gpioe));
        
    }
}



fn set_to_72MHz(rcc: &stm32f3::stm32f303::RCC, flash: &stm32f3::stm32f303::FLASH){
    rcc.cr.modify(|_,w| w.hseon().set_bit()); //turns on the external oscillator
    while rcc.cr.read().hserdy().bit_is_clear(){} //wait for it to be ready
    flash.acr.modify(|_,w| {
        w.prftbe().set_bit(); //turn on prefetch
        unsafe{w.latency().bits(0b010)} //set latency for 72MHz
    }); 

    rcc.cfgr.modify(|_,w| {
        w.pllxtpre().clear_bit();
        unsafe{w.pllsrc().bits(0b10)};
        w.pllmul().bits(0b0111);
        unsafe{w.hpre().bits(0b0000);
        w.ppre1().bits(0b100);
        w.ppre2().bits(0b000)}
    });

    rcc.cr.modify(|_,w| w.pllon().set_bit());
    while rcc.cr.read().pllrdy().bit_is_clear(){}

    rcc.cfgr.modify(|_,w| unsafe{w.sw().bits(0b10)});
    while rcc.cfgr.read().sws().bits() != 0b10{}
}
