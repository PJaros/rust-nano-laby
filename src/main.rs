#![no_std]
#![no_main]
#![feature(stmt_expr_attributes)]

// use panic_halt as _;
use arduino_hal::adc;
use arduino_hal::port::mode::Output;
use arduino_hal::port::Pin;
use arduino_hal::prelude::*;

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
const START_SIZE_X: isize = 7;
const START_SIZE_Y: isize = 7;
const MAX_RX: usize = 35;
const MAX_RY: usize = 35;
const MAX_ARR_SIZE: usize = (MAX_RX * MAX_RY).div_ceil(8);
const MAX_POS_SIZE: usize = (MAX_RX - 3) / 2 * (MAX_RY - 3) / 2;

struct ButtonState {
    btn_n: bool,
    btn_w: bool,
    btn_e: bool,
    btn_s: bool,
    // btn_reset: bool,
}

struct Wall {
    wall_n: bool,
    wall_w: bool,
    wall_e: bool,
    wall_s: bool,
}

fn set_wall(li: &Laby, pos: &isize, w: &mut Wall) {
    w.wall_n = li.read((pos + li.dirs[0]) as usize);
    w.wall_w = li.read((pos + li.dirs[1]) as usize);
    w.wall_e = li.read((pos + li.dirs[2]) as usize);
    w.wall_s = li.read((pos + li.dirs[3]) as usize);
}

fn led_show_wall(leds: &mut [&mut Pin<Output>; 4], w: &mut Wall) {
    led_set_state(leds[0], w.wall_n);
    led_set_state(leds[1], w.wall_w);
    led_set_state(leds[2], w.wall_e);
    led_set_state(leds[3], w.wall_s);
}

fn blink(leds: &mut [&mut Pin<Output>; 4], num: usize) {
    for _ in 0..num {
        for state in [true, false] {
            led_all(leds, state);
            arduino_hal::delay_ms(100);
        }
    }
}

fn led_all(leds: &mut [&mut Pin<Output>; 4], state: bool) {
    for l in leds {
        led_set_state(l, state);
    }
}

#[rustfmt::skip]
fn led_set_state(l: &mut Pin<Output>, state: bool) {
    match state {
        false => {l.set_low();},
        true => {l.set_high();}
    }
}

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

    pub fn change_size(&mut self, size_x: isize, size_y: isize) {
        let real_x: isize = size_x + 2;
        let real_y: isize = size_y + 2;

        self.size_x = size_x;
        self.size_y = size_y;
        self.real_x = real_x;
        self._real_y = real_y;
        for i in 0..MAX_ARR_SIZE {
            self.arr[i] = 0;
        }
        self.dirs = [-real_x, -1_isize, 1_isize, real_x];
    }

    #[inline]
    pub fn get_byte_bit_pos(&self, pos: usize) -> (usize, usize) {
        (pos / 8, pos % 8)
    }

    pub fn set_0(&mut self, pos: usize) {
        let (byte_pos, bit_pos) = self.get_byte_bit_pos(pos);
        let mut byte_value = self.arr[byte_pos];
        byte_value &= !(1 << bit_pos);
        self.arr[byte_pos] = byte_value;
    }

    pub fn set_1(&mut self, pos: usize) {
        let (byte_pos, bit_pos) = self.get_byte_bit_pos(pos);
        let mut byte_value = self.arr[byte_pos];
        byte_value |= 1 << bit_pos;
        self.arr[byte_pos] = byte_value;
    }

    pub fn read(&self, pos: usize) -> bool {
        let (byte_pos, bit_pos) = self.get_byte_bit_pos(pos);
        let mut byte_value = self.arr[byte_pos];
        byte_value &= 1 << bit_pos;
        byte_value != 0
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

#[rustfmt::skip]
fn _get_type_of<T>(_: &T) -> &'static str where T: ?Sized, {
    core::any::type_name::<T>()
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());

    let mut led_n = pins.d7.into_output().downgrade();
    let mut led_w = pins.d6.into_output().downgrade();
    let mut led_e = pins.d9.into_output().downgrade();
    let mut led_s = pins.d8.into_output().downgrade();
    let btn_n = pins.d2.into_pull_up_input();
    let btn_w = pins.d3.into_pull_up_input();
    let btn_e = pins.d4.into_pull_up_input();
    let btn_s = pins.d5.into_pull_up_input();
    // let btn_reset = pins.a2.into_pull_up_input().downgrade();
    let a_pin = pins.a0.into_analog_input(&mut adc);

    // Run adc blocking read to ensure that arduino is ready
    _ = adc.read_blocking(&adc::channel::Vbg);
    ufmt::uwriteln!(&mut serial, "Hello Ruum42!\r").unwrap_infallible();
    ufmt::uwriteln!(
        &mut serial,
        "Laybrinth: {} Positions: {}\r",
        MAX_ARR_SIZE,
        MAX_POS_SIZE
    )
    .unwrap_infallible();
    // ufmt::uwriteln!(&mut serial, "{}\r", _get_type_of(&mut serial)).unwrap_infallible();
    // ufmt::uwriteln!(&mut serial, "{}\r", _get_type_of(&mut MAX_POS_SIZE)).unwrap_infallible();
    // ufmt::uwriteln!(&mut serial, "usize::MAX: {}\r", usize::MAX).unwrap_infallible();
    let seed = a_pin.analog_read(&mut adc);
    ufmt::uwriteln!(&mut serial, "Seed: {}\r", seed).unwrap_infallible();
    let mut rng = SmallRng::seed_from_u64(seed as u64);
    let mut level = 1;
    let mut size_x = START_SIZE_X;
    let mut size_y = START_SIZE_Y;
    let mut li = Laby::new(size_x, size_y);
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

    let mut pos: isize = 2 + 2 * li.real_x;
    let mut w = Wall {
        wall_n: false,
        wall_w: false,
        wall_e: false,
        wall_s: false,
    };
    // ufmt::uwriteln!(&mut serial, "wall, n: {}, w: {}, e: {}, s: {}\r", w.wall_n, w.wall_w, w.wall_e, w.wall_s).unwrap_infallible();
    let mut led_arr = &mut [&mut led_n, &mut led_w, &mut led_e, &mut led_s];
    set_wall(&li, &pos, &mut w);
    led_show_wall(&mut led_arr, &mut w);
    let mut last_pressed_buttons = ButtonState {
        btn_n: false,
        btn_w: false,
        btn_e: false,
        btn_s: false,
        // btn_reset: false,
    };
    let mut buttons_were_pressed = false;
    let mut regenerate_laby_next_lvl = false;
    // let mut regenerate_laby_reset = false;

    #[rustfmt::skip]
    loop {    
        let buttons = ButtonState {
            btn_n: btn_n.is_low(),
            btn_w: btn_w.is_low(),
            btn_e: btn_e.is_low(),
            btn_s: btn_s.is_low(),
            // btn_reset: btn_reset.is_low(),
        };
        if  buttons.btn_n == true ||
            buttons.btn_w == true ||
            buttons.btn_e == true ||
            buttons.btn_s == true {
              last_pressed_buttons = buttons;
              buttons_were_pressed = true;
        } else if buttons_were_pressed == true {
            // led_all(&mut led_arr, false);

            if      last_pressed_buttons.btn_n == true && w.wall_n == false {pos += 2 * li.dirs[0];}
            else if last_pressed_buttons.btn_w == true && w.wall_w == false {pos += 2 * li.dirs[1];}
            else if last_pressed_buttons.btn_e == true && w.wall_e == false {pos += 2 * li.dirs[2];}
            else if last_pressed_buttons.btn_s == true && w.wall_s == false {pos += 2 * li.dirs[3];}
            else if last_pressed_buttons.btn_n == true && w.wall_n == true  {blink(led_arr, 2);}
            else if last_pressed_buttons.btn_w == true && w.wall_w == true  {blink(led_arr, 2);}
            else if last_pressed_buttons.btn_e == true && w.wall_e == true  {blink(led_arr, 2);}
            else if last_pressed_buttons.btn_s == true && w.wall_s == true  {blink(led_arr, 2);}
            // else if last_pressed_buttons.btn_reset == true                  {regenerate_laby_reset = true;}        
            buttons_were_pressed = false;

            if pos == li.size_x - 1 + li.real_x * (li.size_y + 1) {
                regenerate_laby_next_lvl = true;
            }

            // if regenerate_laby_next_lvl || regenerate_laby_reset {
            // if regenerate_laby_next_lvl {
                if regenerate_laby_next_lvl {
                    if level > 1 {
                        ufmt::uwriteln!(&mut serial, "Exit found! Labyrinth was:\r").unwrap_infallible();
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
                    }
                    if (size_x + 2 + 2) as usize <= MAX_RX || (size_y + 2 + 2) as usize <= MAX_RY {
                        level += 1;
                        size_x += 2;
                        size_y += 2;
                        li.change_size(size_x, size_y);    
                    }
                    ufmt::uwriteln!(&mut serial, "Level {}. Generating a labyrinth: {} x {}\r", level, size_x, size_y).unwrap_infallible();
                // } else { // regenerate_laby_reset
                //     level = 1;
                //     size_x = START_SIZE_X;
                //     size_y = START_SIZE_Y;
                //     li.change_size(size_x, size_y);    
                //     ufmt::uwriteln!(&mut serial, "Reset to Level {}. Generating a labyrinth: {} x {}\r", level, size_x, size_y).unwrap_infallible();
                // }
                pos = 2 + 2 * li.real_x;
                blink(led_arr, 4);
                li.generate(&mut rng);
                regenerate_laby_next_lvl = false;
                // regenerate_laby_reset = false;
            }
            set_wall(&li, &pos, &mut w);
            led_show_wall(&mut led_arr, &mut w);
        }
        arduino_hal::delay_ms(50);
    }
}
