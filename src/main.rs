#![no_std]
#![no_main]

use panic_halt as _;
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
use rand::RngCore;

const MAX_RX: usize = 25;
const MAX_RY: usize = 25;
// const MAX_RX: usize = 21;
// const MAX_RY: usize = 21;
// const MAX_RX: usize = 101;
// const MAX_RY: usize = 101;
const MAX_ARR_SIZE: usize = MAX_RX * MAX_RY;
const MAX_POS_SIZE: usize =
      (MAX_RX - 3) / 2
    * (MAX_RY - 3) / 2;

// struct Laby {
//     size_x: i8,
//     size_y: i8,
//     real_x: i8,
//     real_y: i8,
//     arr: [u8; MAX_ARR_SIZE],
//     dirs: [i8; 4],
// }
struct Laby {
    size_x: isize,
    size_y: isize,
    real_x: isize,
    real_y: isize,
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
            real_y: real_y,
            arr: [0; MAX_ARR_SIZE],
            dirs: [-real_x, -1_isize, 1_isize, real_x],
        };
        li
    }
}
#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    arduino_hal::delay_ms(1000);

    let mut led = pins.d13.into_output();
    let mut led_n = pins.d7.into_output();
    let mut led_w = pins.d6.into_output();
    let mut led_e = pins.d9.into_output();
    let mut led_s = pins.d8.into_output();
    let btn_n = pins.d3.into_pull_up_input();
    let btn_w = pins.d2.into_pull_up_input();
    let btn_e = pins.d4.into_pull_up_input();
    let btn_s = pins.d5.into_pull_up_input();

    let mut rng = SmallRng::seed_from_u64(1234_u64);
    let rand_num = rng.next_u64();
    _ = ufmt::uwriteln!(&mut serial, "Random: {}\r", rand_num);
    ufmt::uwriteln!(&mut serial, "Hello Ruum42!\r").unwrap();
    let mut li = Laby::new(19, 19);
    for i in 0..MAX_ARR_SIZE {
        li.arr[i] = 0;
    }
    for y in 1..li.size_y + 1 {
        for x in 1..li.size_x + 1 {
            li.arr[(x + y * li.real_x) as usize] = 1; 
        }
    }
    let mut jump_pos = [0_usize; MAX_POS_SIZE];
    let mut jump_num = 0_i32;
    let mut pos: isize = 2 + 2 * li.real_x;     
    li.arr[pos as usize] = 0;

    loop {
        loop {
            let mut avai_dir = [0_isize; 4];
            let mut avai_found = 0_usize;
            for i in 0..li.dirs.len() {
                let dir = li.dirs[i];
                let look_pos = (pos + 2 * dir) as usize;
                if li.arr[look_pos] != 0 {
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
                li.arr[pos as usize] = 0;
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

    for y in 1..li.size_y + 1 {
        for x in 1..li.size_x + 1 {
            let num = li.arr[(x + y * li.real_x) as usize];
            let c = match num {
                0 => ' ',
                _ => '#',
            };
            ufmt::uwrite!(&mut serial, "{}", c).unwrap(); 
        }
        ufmt::uwriteln!(&mut serial, "\r").unwrap(); 
    }
    // let mut dxy = [0_u8; 4];
    // for y in (1..li.size_y + 1).step_by(2) {
    //     for x in (1..li.size_x + 1).step_by(2) {
    //         let pos = (x + y * li.real_x) as isize;
    //         for i in 0..li.dirs.len() {
    //             dxy[i] = li.arr[(pos + li.dirs[i]) as usize];
    //         }
    //         let c = match dxy {
    //             [0, 0, 0, 0] => ' ',
    //             [0, 0, 0, _] => '╷',
    //             [0, 0, _, 0] => '╶',
    //             [0, 0, _, _] => '┌',
    //             [0, _, 0, 0] => '╴',
    //             [0, _, 0, _] => '┐',
    //             [0, _, _, 0] => '─',
    //             [0, _, _, _] => '┬',
    //             [_, 0, 0, 0] => '╵',
    //             [_, 0, 0, _] => '│',
    //             [_, 0, _, 0] => '└',
    //             [_, 0, _, _] => '├',
    //             [_, _, 0, 0] => '┘',
    //             [_, _, 0, _] => '┤',
    //             [_, _, _, 0] => '┴',
    //             [_, _, _, _] => '┼',
    //         };
    //         ufmt::uwrite!(&mut serial, "{}", c).unwrap(); 
    //     }
    //     ufmt::uwriteln!(&mut serial, "\r").unwrap(); 
    // }
    // ufmt::uwriteln!(&mut serial, "Laby, real: {}x{}, size: {}x{}\r", li.real_x, li.real_y, li.size_x, li.size_y).unwrap();

    ufmt::uwriteln!(&mut serial, "Generating...\r").unwrap();
    ufmt::uwriteln!(&mut serial, "Laby generated!\r").unwrap();

    loop {
        if btn_n.is_low() {ufmt::uwriteln!(&mut serial, "btn_n\r").unwrap(); }
        if btn_w.is_low() {ufmt::uwriteln!(&mut serial, "btn_w\r").unwrap(); }
        if btn_e.is_low() {ufmt::uwriteln!(&mut serial, "btn_e\r").unwrap(); }
        if btn_s.is_low() {ufmt::uwriteln!(&mut serial, "btn_s\r").unwrap(); }
        if btn_n.is_low() {led_n.set_high();} else {led_n.set_low();}
        if btn_w.is_low() {led_w.set_high();} else {led_w.set_low();}
        if btn_e.is_low() {led_e.set_high();} else {led_e.set_low();}
        if btn_s.is_low() {led_s.set_high();} else {led_s.set_low();}
        led.toggle();
        arduino_hal::delay_ms(1000);
    }
}