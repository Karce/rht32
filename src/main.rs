#![no_main]
#![no_std]
#[allow(unused_extern_crates)] // NOTE(allow) bug rust-lang/rust#53964
//extern crate panic_itm; // panic handler
extern crate panic_halt; //

pub use cortex_m::asm::nop;
pub use cortex_m_rt::entry;

// use embedded_hal::digital::*;
use f3::hal::delay::Delay;
use f3::hal::prelude::*;
use f3::hal::prelude::_embedded_hal_digital_OutputPin;
use f3::hal::prelude::_embedded_hal_digital_InputPin;
//use f3::hal::digital::v2::InputPin;
use f3::led::Led;
use f3::hal::stm32f30x::{self, GPIOC};
/*
#[inline(never)]
fn delay(p: &stm32f30x::Peripherals, ms: u16) {
    // Set the timer to go off in `ms` ticks
    // 1 tick = 1 ms
    p.TIM6.arr.write(|w| w.arr().bits(ms));
    
    // CEN: Enable the counter
    p.TIM6.cr1.modify(|_, w| w.cen().set_bit());
    
    // Wait until the alarm goes off (until the update event occurs)
    while !p.TIM6.sr.read().uif().bit_is_set() {}
    
    // Clear the update event flag
    p.TIM6.sr.modify(|_, w| w.uif().clear_bit());
}
*/

#[entry]
fn main() -> ! {
    let MAX_TIME = 32_000;
    let mut reading: [u8; 5] = [0; 5];
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32f30x::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);
    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut led: Led = gpioe
        .pe8
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper)
        .into();
    let mut delay = Delay::new(cp.SYST, clocks);
    // let mut pc1 = gpioc.pc1;

    led.on();
    let mut count = 0;
    let mut error = false;
    let mut pc1_out = gpioc.pc1.into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
    // pc1_out.set_high();
    // delay.delay_ms(500_u16);
    pc1_out.set_low();
    delay.delay_us(500_u16);
    pc1_out.set_high();
    delay.delay_us(30_u16);
    let mut pc1_in = pc1_out.into_floating_input(&mut gpioc.moder, &mut gpioc.pupdr);
    delay.delay_us(40_u16);

    if !unsafe {(*GPIOC::ptr()).idr.read().idr1().bit()} {
        delay.delay_us(80_u16);
        if unsafe {(*GPIOC::ptr()).idr.read().idr1().bit()} {
            // This is good.
            error = false;
        }
    }
    
    while unsafe {(*GPIOC::ptr()).idr.read().idr1().bit()} {
        count += 1;
        if count >= MAX_TIME {
            error = true;
            break;
        }
    }
    let mut lows: [u32; 41] = [0; 41];
    let mut highs: [u32; 41] = [0; 41];
    for i in 0..41 {
        // 41 * 2
        while !unsafe {(*GPIOC::ptr()).idr.read().idr1().bit()} {
            lows[i] += 1;
            if lows[i] >= MAX_TIME {
                error = true;
                break;
            }
        }
        while unsafe {(*GPIOC::ptr()).idr.read().idr1().bit()} {
            highs[i] += 1;
            if highs[i] >= MAX_TIME {
                error = true;
                break;
            }
        }
        
    }
    let mut threshold: u32 = 0;
    for i in 1..41 {
        threshold += lows[i];
    }
    threshold /= 40;

    for i in 1..41 {
        let index = (i-1)/16;    
        reading[index] <<= 1;
        if highs[i] > threshold {
            reading[index] |= 1;
        }
    }
    led.off();
    delay.delay_ms(2_000_u16);

    loop {}

    // peripherals.RCC.apb1enr.modify(|_, w| w.tim6en().set_bit());
    // peripherals.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit());
    // peripherals.RCC.ahbenr.modify(|_, w| w.iopeen().set_bit());
    // Power on IO Port C.
    // peripherals.RCC.ahbenr.modify(|_, w| w.iopcen().set_bit());
    // peripherals.GPIOC.moder.modify(|_, w| w.moder1().output());
    // peripherals.GPIOC.odr.write(|w| w.odr1().set_bit());

    //peripherals.TIM6.cr1.write(|w| w.opm().set_bit().cen().clear_bit());
    // Configure LED to output mode.
    /*
    peripherals.GPIOE.moder.modify(|_, w| {
        w.moder8().output()
    });
    */

    // (7_999 + 1) is 1 ms.
    //peripherals.TIM6.psc.write(|w| w.psc().bits(7_999));

    // Configure PA0 to pull up resister mode.
    // peripherals.GPIOC.pupdr.write(|w| unsafe { w.pupdr1().bits(1) });
    // Configure PA0 to high speed.
    // peripherals.GPIOC.ospeedr.write(|w| unsafe { w.ospeedr1().bits(3) });

    /*
    loop {
        // Turn on an LED
        peripherals.GPIOE.odr.write(|w| w.odr8().set_bit());
        // Configure PC1 to output mode first.
        peripherals.GPIOC.moder.modify(|_, w| w.moder1().output());
        peripherals.GPIOC.odr.write(|w| w.odr1().clear_bit());
        //peripherals.TIM2.arr.write(|w| w.arrh().bits(255));
        // Wait for 18 ms.
        delay(&peripherals, 18);
        peripherals.GPIOC.odr.write(|w| w.odr1().set_bit());
        // The delay ticks in increments of 5us.
        delay(&peripherals, 1);
        peripherals.GPIOC.moder.modify(|_, w| w.moder1().input());

        // at 69 iterations: bit turns on in input mode and connection dies.
        // at 68 iterations: bit is still off and count == 0.
        //for _ in 0..500 {}

        let mut count = 0;
        while peripherals.GPIOC.idr.read().idr1().bit() {
            count += 1;
            if count >= 64000 {
                break;
            }
        }
        let mut pulses: [u32; 82] = [0; 82];

        for i in 0..82 {
            // 41 * 2
            if i % 2 == 0 {
                while !peripherals.GPIOC.idr.read().idr1().bit() {
                    pulses[i] += 1;
                    if pulses[i] >= 32000 {
                        break;
                    }
                }
            }
            else {
                while peripherals.GPIOC.idr.read().idr1().bit() {
                    pulses[i] += 1;
                    if pulses[i] >= 32000 {
                        break;
                    }
                }
            }
        }
        let mut threshold: u32 = 0;
        for i in 2..82 {
            if i % 2 == 0 {
                threshold += pulses[i];
            }
        }
        threshold /= 40;


        for i in 3..82 {
            if i % 2 == 1 {
                let index = (i-3)/16;    
                reading[index] <<= 1;
                if pulses[i] > threshold {
                    reading[index] |= 1;
                }
            }
        }
        // Turn off an LED
        peripherals.GPIOE.odr.write(|w| w.odr8().clear_bit());
        
        let total = (reading[0] + reading[1] + reading[2] + reading[3]) & 0xff;
        if total == reading[4] {
            let rh = (reading[0] as f32 * 256_f32 + reading[1] as f32) / 10.0_f32;
            let temp = ((reading[2] & 0x7F) as f32 * 256_f32 + reading[3] as f32) / 10.0_f32;
        } else {
            // Somethings gone wrong.
        }

        // Delay for 2 seconds.
        delay(&peripherals, 2_000);
    }
    */
}
