#![no_std]
#![no_main]
#![feature(stmt_expr_attributes)]

// use panic_halt as _;
use arduino_hal::adc;
use arduino_hal::prelude::*;
// use arduino_hal::delay_ms;
// use atmega_hal::port::mode::Output;
// use atmega_hal::port::{Dynamic, Pin};
// use avr_hal_generic::avr_device;
// use core::{fmt::Write, panic::PanicInfo};
// use embassy_executor::Spawner;
// use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
// use ufmt::uwriteln;

// Commands:
// Build: RAVEDUDE_PORT=/dev/ttyUSB0 cargo build --release
// Detect USB-Port: journalctl -k -f

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

use rand::rngs::SmallRng;
use rand::{Rng, SeedableRng};

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
const MAX_RX: usize = 35;
const MAX_RY: usize = 35;
// const MAX_RX: usize = 25;
// const MAX_RY: usize = 25;
const MAX_ARR_SIZE: usize = (MAX_RX * MAX_RY).div_ceil(8);
const MAX_POS_SIZE: usize = (MAX_RX - 3) / 2 * (MAX_RY - 3) / 2;

struct Laby {
    size_x: isize,
    size_y: isize,
    real_x: isize,
    _real_y: isize,
    arr: [u8; MAX_ARR_SIZE],
    dirs: [isize; 4],
    // max_used_jump: i32,
}

impl Laby {
    pub fn new(size_x: isize, size_y: isize) -> Self {
        let real_x: isize = size_x + 2;
        let real_y: isize = size_y + 2;

        Self {
            size_x,
            size_y,
            real_x,
            _real_y: real_y,
            arr: [0; MAX_ARR_SIZE],
            dirs: [-real_x, -1_isize, 1_isize, real_x],
            // max_used_jump: 0,
        }
    }

    pub fn set_0(&mut self, pos: usize) {
        let byte_pos = pos / 8;
        let bit_pos = pos % 8;
        let mut byte_value = self.arr[byte_pos];
        byte_value &= !(1 << bit_pos);
        self.arr[byte_pos] = byte_value;
    }

    pub fn set_1(&mut self, pos: usize) {
        let byte_pos = pos / 8;
        let bit_pos = pos % 8;
        let mut byte_value = self.arr[byte_pos];
        byte_value |= 1 << bit_pos;
        self.arr[byte_pos] = byte_value;
    }

    pub fn read(&mut self, pos: usize) -> bool {
        let byte_pos = pos / 8;
        let bit_pos = pos % 8;
        let mut byte_value = self.arr[byte_pos];
        byte_value &= 1 << bit_pos;
        byte_value > 0
    }

    pub fn generate(&mut self, rng: &mut impl Rng) {
        for i in 0..MAX_ARR_SIZE {
            self.arr[i] = 0;
        }
        for y in 1..self.size_y + 1 {
            for x in 1..self.size_x + 1 {
                let pos = (x + y * self.real_x) as usize;
                self.set_1(pos);
            }
        }
        let mut jump_pos = [0_usize; MAX_POS_SIZE];
        let mut jump_num = 0_usize;
        let mut pos: isize = 2 + 2 * self.real_x;
        self.set_0(pos as usize);

        loop {
            loop {
                let mut avai_dir = [0_isize; 4];
                let mut avai_found = 0_usize;
                for i in 0..self.dirs.len() {
                    let dir = self.dirs[i];
                    let look_pos = (pos + 2 * dir) as usize;

                    if self.read(look_pos) == true {
                        avai_dir[avai_found] = dir;
                        avai_found += 1;
                    }
                }

                #[rustfmt::skip]
                let dir = match avai_found {
                    0 => {break;},
                    1 => avai_dir[0],
                    _ => {
                        let slot = rng.gen_range(0..avai_found);
                        jump_pos[jump_num] = pos as usize;
                        jump_num += 1;
                        avai_dir[slot]
                    }
                };

                for _ in 0..2 {
                    pos += dir;
                    self.set_0(pos as usize);
                }
            }
            #[rustfmt::skip]
            match jump_num {
                0 => {break;}
                _ => {
                    jump_num -= 1;
                    pos = jump_pos[jump_num] as isize;
                }
            }
            // self.max_used_jump = (jump_num as i32).max(self.max_used_jump);
        }
        let pos = self.size_x - 1 + self.real_x * self.size_y;
        self.set_0(pos as usize);
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
    ufmt::uwriteln!(
        &mut serial,
        "Laybrinth: {} Positions: {}\r",
        MAX_ARR_SIZE,
        MAX_POS_SIZE
    )
    .unwrap_infallible();
    // ufmt::uwriteln!(&mut serial, "{}\r", get_type_of(&mut serial)).unwrap_infallible();
    // ufmt::uwriteln!(&mut serial, "{}\r", get_type_of(&mut MAX_POS_SIZE)).unwrap_infallible();
    // ufmt::uwriteln!(&mut serial, "usize::MAX: {}\r", usize::MAX).unwrap_infallible();
    let seed = a_pin.analog_read(&mut adc);
    ufmt::uwriteln!(&mut serial, "Seed: {}\r", seed).unwrap_infallible();
    let mut rng = SmallRng::seed_from_u64(seed as u64);
    let mut li = Laby::new(33, 33);
    // let mut li = Laby::new(49, 49);
    // let mut li = Laby::new(19, 19);
    ufmt::uwriteln!(&mut serial, "Generating...\r").unwrap_infallible();

    // const TEST_NUM: usize = 500;
    // let mut max_all = 0;
    // let mut max_sum = 0;
    // for i in 1..=TEST_NUM {
    //     li.generate(&mut rng);
    //     if i % 100 == 0 {
    //         ufmt::uwriteln!(&mut serial, "i: {}\r", i).unwrap_infallible();
    //     }
    //     max_sum += li.max_used_jump;
    //     max_all = max_all.max(li.max_used_jump);
    //     li.max_used_jump = 0;
    // }
    // let max_avg = (max_sum as f32) / (TEST_NUM as f32);
    // ufmt::uwriteln!(&mut serial, "max_all: {}, max_avg: {}\r", max_all, max_avg as u32).unwrap_infallible();

    // ufmt::uwriteln!(&mut serial, "max_all: {}, 0: {}, 1: {}, 2: {}\r", max_all, stat_jump[0], stat_jump[1], stat_jump[2], ).unwrap_infallible();
    // let mut container = [0; 3];
    // for _ in 0..10000 {
    //     let c = rng.gen_range(0..3);
    //     container[c] += 1;
    // }
    // ufmt::uwriteln!(&mut serial, "0: {}, 1: {}, 2: {}\r", container[0], container[1], container[2], ).unwrap_infallible();

    li.generate(&mut rng);

    for y in 1..li.size_y + 1 {
        for x in 1..li.size_x + 1 {
            let pos = (x + y * li.real_x) as usize;
            let v = li.read(pos);
            let c = match v {
                false => ' ',
                true => '#',
            };
            ufmt::uwrite!(&mut serial, "{}", c).unwrap_infallible();
        }
        ufmt::uwriteln!(&mut serial, "\r").unwrap_infallible();
    }
    ufmt::uwriteln!(&mut serial, "Laby generated!\r").unwrap_infallible();

    #[rustfmt::skip]
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
