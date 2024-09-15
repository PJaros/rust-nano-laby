#![no_std]
#![no_main]

// use panic_halt as _;
use arduino_hal::prelude::*;
use arduino_hal::adc;
// use arduino_hal::delay_ms;
// use atmega_hal::port::mode::Output;
// use atmega_hal::port::{Dynamic, Pin};
// use avr_hal_generic::avr_device;
// use core::{fmt::Write, panic::PanicInfo};
// use embassy_executor::Spawner;
// use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
// use ufmt::uwriteln;

// Links:
// Modules: https://rust-classes.com/chapter_4_3
// OOP: https://doc.rust-lang.org/book/ch17-01-what-is-oo.html
// Linuxday: https://www.linuxday.at/

// Links - RNG:
// std RNG: https://stackoverflow.com/a/37017052/406423
// rahix-avr: https://github.com/Rahix/avr-hal
// arduino: https://rahix.github.io/avr-hal/arduino_hal/index.html
// embedded-hal: https://crates.io/crates/embedded-hal/0.2.3
// embedded-hal, rng: https://github.com/rust-embedded/embedded-hal/issues/128
// rand::RngCore: https://docs.rs/rand/0.6.5/rand/trait.RngCore.html
// small_rng example: https://stackoverflow.com/a/67652214/406423
// crates.io randCore: https://crates.io/crates/rand_core
// micro_rand: https://crates.io/crates/micro_rand
// LCG: https://en.wikipedia.org/wiki/Linear_congruential_generator
// CLCG: https://en.wikipedia.org/wiki/Combined_linear_congruential_generator

// use crate::laby::Laby;
// mod laby;

use rand::{Rng, SeedableRng};
use rand::rngs::SmallRng;

// Panic handler per https://github.com/Rahix/avr-hal/blob/main/examples/arduino-nano/src/bin/nano-panic.rs
#[cfg(not(doc))]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    avr_device::interrupt::disable();

    // Get the peripherals so we can access serial and the LED.
    // Because the interrupt is disabled and main has called and main is in panic, we know this is safe.
    let dp = unsafe { arduino_hal::Peripherals::steal() };
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    // Print out panic location
    ufmt::uwriteln!(&mut serial, "Firmware panic!\r").unwrap_infallible();
    if let Some(loc) = info.location() {
        ufmt::uwriteln!(
            &mut serial,
            "  At {}:{}:{}\r",
            loc.file(),
            loc.line(),
            loc.column(),
        )
        .unwrap_infallible();
    }

    // Blink LED rapidly
    let mut led = pins.d13.into_output();
    loop {
        led.toggle();
        arduino_hal::delay_ms(100);
    }
}
const MAX_RX: usize = 25;
const MAX_RY: usize = 25;
const MAX_ARR_SIZE: usize = MAX_RX * MAX_RY;
const MAX_POS_SIZE: usize =
      (MAX_RX - 3) / 2
    * (MAX_RY - 3) / 2;

struct Laby {
    size_x: isize,
    size_y: isize,
    real_x: isize,
    _real_y: isize,
    arr: [u8; MAX_ARR_SIZE],
    dirs: [isize; 4],
}

impl Laby {
    pub fn new(size_x: isize, size_y: isize) -> Self {
        let real_x: isize = size_x + 2;
        let real_y: isize = size_y + 2;
        let li = Self {
            size_x: size_x,
            size_y: size_y,
            real_x: real_x,
            _real_y: real_y,
            arr: [0; MAX_ARR_SIZE],
            dirs: [-real_x, -1_isize, 1_isize, real_x],
        };
        li
    }

    pub fn generate(&mut self, rng: &mut impl Rng) {
        for i in 0..MAX_ARR_SIZE {
            self.arr[i] = 0;
        }
        for y in 1..self.size_y + 1 {
            for x in 1..self.size_x + 1 {
                self.arr[(x + y * self.real_x) as usize] = 1; 
            }
        }
        let mut jump_pos = [0_usize; MAX_POS_SIZE];
        let mut jump_num = 0_i32;
        let mut pos: isize = 2 + 2 * self.real_x;     
        self.arr[pos as usize] = 0;
    
        loop {
            loop {
                let mut avai_dir = [0_isize; 4];
                let mut avai_found = 0_usize;
                for i in 0..self.dirs.len() {
                    let dir = self.dirs[i];
                    let look_pos = (pos + 2 * dir) as usize;
                    if self.arr[look_pos] != 0 {
                        avai_dir[avai_found] = dir;
                        avai_found += 1;
                    }
                }
    
                let dir = match avai_found {
                    0 => {break;},
                    1 => avai_dir[0],
                    _ => {
                        let slot = rng.gen_range(0..=avai_found);
                        avai_dir[slot]
                    }
                };
                jump_pos[jump_num as usize] = pos as usize;
                jump_num += 1;
    
                for _ in 0..2 {
                    pos += dir;
                    self.arr[pos as usize] = 0;
                }
            }
            match jump_num {
                0 => {break;}
                _ => {
                    jump_num -= 1;
                    pos = jump_pos[jump_num as usize] as isize;
                }
            }
        }
        self.arr[(self.size_x - 1 + self.real_x * self.size_y) as usize] = 0;    
    }
}

// fn get_type_of<T>(_: &T) -> &'static str where T: ?Sized, {
//     core::any::type_name::<T>()
// }

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());

    let mut led = pins.d13.into_output();
    let mut led_n = pins.d7.into_output();
    let mut led_w = pins.d6.into_output();
    let mut led_e = pins.d9.into_output();
    let mut led_s = pins.d8.into_output();
    let btn_n = pins.d2.into_pull_up_input();
    let btn_w = pins.d3.into_pull_up_input();
    let btn_e = pins.d4.into_pull_up_input();
    let btn_s = pins.d5.into_pull_up_input();
    let a_pin = pins.a0.into_analog_input(&mut adc);

    // Run adc blocking read to certain that arduino is ready
    _ = adc.read_blocking(&adc::channel::Vbg);
    ufmt::uwriteln!(&mut serial, "Hello Ruum42!\r").unwrap_infallible();
    ufmt::uwriteln!(&mut serial, "Laybrinth: {} Positions: {}\r", MAX_ARR_SIZE, MAX_POS_SIZE).unwrap_infallible();
    // ufmt::uwriteln!(&mut serial, "{}\r", get_type_of(&mut serial)).unwrap_infallible();
    let seed = a_pin.analog_read(&mut adc);
    ufmt::uwriteln!(&mut serial, "Seed: {}\r", seed).unwrap_infallible();
    let mut rng = SmallRng::seed_from_u64(seed as u64);
    let mut li = Laby::new(19, 19);
    ufmt::uwriteln!(&mut serial, "Generating...\r").unwrap_infallible();
    li.generate(&mut rng);
    
    for y in 1..li.size_y + 1 {
        for x in 1..li.size_x + 1 {
            let num = li.arr[(x + y * li.real_x) as usize];
            let c = match num {
                0 => ' ',
                _ => '#',
            };
            ufmt::uwrite!(&mut serial, "{}", c).unwrap_infallible(); 
        }
        ufmt::uwriteln!(&mut serial, "\r").unwrap_infallible(); 
    }
    ufmt::uwriteln!(&mut serial, "Laby generated!\r").unwrap_infallible();

    loop {
        if btn_n.is_low() {ufmt::uwriteln!(&mut serial, "btn_n\r").unwrap_infallible(); }
        if btn_w.is_low() {ufmt::uwriteln!(&mut serial, "btn_w\r").unwrap_infallible(); }
        if btn_e.is_low() {ufmt::uwriteln!(&mut serial, "btn_e\r").unwrap_infallible(); }
        if btn_s.is_low() {ufmt::uwriteln!(&mut serial, "btn_s\r").unwrap_infallible(); }
        if btn_n.is_low() {led_n.set_high();} else {led_n.set_low();}
        if btn_w.is_low() {led_w.set_high();} else {led_w.set_low();}
        if btn_e.is_low() {led_e.set_high();} else {led_e.set_low();}
        if btn_s.is_low() {led_s.set_high();} else {led_s.set_low();}
        led.toggle();
        arduino_hal::delay_ms(1000);
        // panic!();<
    }
}