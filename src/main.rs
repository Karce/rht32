#![no_main]
#![no_std]
#[allow(unused_extern_crates)]
extern crate panic_halt;

pub use cortex_m::asm::nop;
pub use cortex_m_rt::entry;

use f3::hal::delay::Delay;
use f3::hal::prelude::*;
use f3::hal::stm32f30x::{self};

#[entry]
fn main() -> ! {
    let max_time = 32_000;
    let cm = cortex_m::Peripherals::take().unwrap();
    // peri is a very basic mapping for rust onto the hardware.
    // This is useful for abstracting away physical addresses.
    // The basic format is show below.
    let peri = stm32f30x::Peripherals::take().unwrap();

    // Power on the LED peripheral (GPIOE).
    peri.RCC.ahbenr.modify(|_, w| w.iopeen().set_bit());

    // Power on the GPIOC pins.
    peri.RCC.ahbenr.modify(|_, w| w.iopcen().set_bit());

    // Configure the led pin for output mode.
    peri.GPIOE.moder.write(|w| w.moder8().output());

    // Turn the blue LED at pin PE8 on.
    peri.GPIOE.odr.write(|w| w.odr8().set_bit());

    // Using a HAL delay abstraction here to setup the clock/delay.
    let mut flash = peri.FLASH.constrain();
    let rcc = peri.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = Delay::new(cm.SYST, clocks);

    let mut count = 0;
    let mut error = false;
    
    // TIMING CRITICAL SECTION
    //
    // Configure pin PC1 for output mode.
    peri.GPIOC.moder.write(|w| w.moder1().output());

    // Write pin low.
    peri.GPIOC.odr.write(|w| w.odr1().clear_bit());

    // This delay should produce a blocking wait for 500 microseconds.
    delay.delay_us(500_u16);

    // Write pin high.
    peri.GPIOC.odr.write(|w| w.odr1().set_bit());
    delay.delay_us(30_u16);

    // Change pin PC1 into input mode.
    // Should I change any other configuration like PUPDR or SPEED or anything?
    peri.GPIOC.moder.write(|w| w.moder1().input());

    // This delay should give the sensor enough time to take over.
    delay.delay_us(40_u16);

    /*
    // Perform the first read of the bit.
    if !peri.GPIOC.idr.read().idr1().bit() {
        delay.delay_us(80_u16);
        if peri.GPIOC.idr.read().idr1().bit() {
            // This is good.
            error = false;
        }
    }
    */
    
    while peri.GPIOC.idr.read().idr1().bit() {
        count += 1;
        if count >= max_time {
            error = true;
            break;
        }
    }
    // Collect the time it takes between lows and highs.
    let mut lows: [u32; 41] = [0; 41];
    let mut highs: [u32; 41] = [0; 41];
    for i in 0..41 {
        while !peri.GPIOC.idr.read().idr1().bit() {
            lows[i] += 1;
            if lows[i] >= max_time {
                error = true;
                break;
            }
        }
        while peri.GPIOC.idr.read().idr1().bit() {
            highs[i] += 1;
            if highs[i] >= max_time {
                error = true;
                break;
            }
        }
    }
    // Calculate the average time of the lows to determine a threshold
    // between 0s and 1s.
    let mut threshold: u32 = 0;
    for i in 1..41 {
        threshold += lows[i];
    }
    threshold /= 40;

    // Interpret the response, setting bits based on time.
    // The array [0, 0, 0, 0, 0] as unsigned 8-bit integers.
    let mut reading: [u8; 5] = [0; 5];
    for i in 1..41 {
        let index = (i-1)/16;    
        reading[index] <<= 1;
        if highs[i] > threshold {
            reading[index] |= 1;
        }
    }
    // Turn the LED at pin PE8 off.
    peri.GPIOE.odr.write(|w| w.odr8().clear_bit());

    // Get the readings from the response.
    let total = (reading[0] + reading[1] + reading[2] + reading[3]) & 0xff;
    if total == reading[4] {
        let rh = reading[0] as f32 + (reading[1] as f32 / 10.0_f32);
        let temp = reading[2] as f32 + (reading[3] as f32 / 10.0_f32);
    }
    else {
        // There was some error with the checksum.
    }

    // Wait 2 seconds.
    delay.delay_ms(2_000_u16);

    // Loop infinitely but do nothing.
    loop {}
}
