#![no_std]
#![no_main]
#![feature(stmt_expr_attributes)]

use panic_halt as _;

use arduino_hal::prelude::*;

use crate::ws2812::Ws2812;
use arduino_hal::{
    adc,
    port::mode::{Input, PullUp},
    port::Pin,
    spi, Spi,
};
use smart_leds::{
    brightness,
    colors::{BLUE, CYAN, GREEN, MAGENTA, RED, YELLOW},
    SmartLedsWrite, RGB8,
};
use ws2812_spi as ws2812;

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

use crate::laby::{Laby, MAX_RX, MAX_RY};
mod laby;

use rand::rngs::SmallRng;
use rand::SeedableRng;

// Panic handler from https://github.com/Rahix/avr-hal/blob/main/examples/arduino-nano/src/bin/nano-panic.rs
// #[cfg(not(doc))]
// #[panic_handler]
// fn panic(info: &core::panic::PanicInfo) -> ! {
//     avr_device::interrupt::disable();

//     // Get the peripherals so we can access serial and the LED.
//     // Because the interrupt is disabled and main has called and main is in panic, we know this is safe.
//     let dp = unsafe { arduino_hal::Peripherals::steal() };
//     let pins = arduino_hal::pins!(dp);
//     let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

//     // Print out panic location
//     ufmt::uwriteln!(&mut serial, "Firmware panic!\r").unwrap_infallible();
//     if let Some(loc) = info.location() {
//         ufmt::uwriteln!(
//             &mut serial,
//             "  At {}:{}:{}\r",
//             loc.file(),
//             loc.line(),
//             loc.column(),
//         )
//         .unwrap_infallible();
//     }

//     // Blink LED rapidly
//     let mut led = pins.d13.into_output();
//     loop {
//         led.toggle();
//         arduino_hal::delay_ms(100);
//     }
// }

const START_SIZE_X: isize = 7;
const START_SIZE_Y: isize = 7;

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
struct Btn {
    north: Pin<Input<PullUp>>,
    east: Pin<Input<PullUp>>,
    south: Pin<Input<PullUp>>,
    west: Pin<Input<PullUp>>,
}

const NUM_LEDS: usize = 87;
const PIXEL_N: (usize, usize) = (0, 18);
const PIXEL_W: (usize, usize) = (23, 41);
const PIXEL_S: (usize, usize) = (46, 64);
const PIXEL_E: (usize, usize) = (69, 87);

fn set_wall(li: &Laby, pos: &isize, w: &mut Wall) {
    w.wall_n = li.read((pos + li.dirs[0]) as usize);
    w.wall_w = li.read((pos + li.dirs[1]) as usize);
    w.wall_e = li.read((pos + li.dirs[2]) as usize);
    w.wall_s = li.read((pos + li.dirs[3]) as usize);
}

// fn led_show_wall(leds: &mut [&mut Pin<Output>; 4], w: &mut Wall) {
//     led_set_state(leds[0], w.wall_n);
//     led_set_state(leds[1], w.wall_w);
//     led_set_state(leds[2], w.wall_e);
//     led_set_state(leds[3], w.wall_s);
// }

// fn blink(leds: &mut [&mut Pin<Output>; 4], num: usize) {
//     for _ in 0..num {
//         for state in [true, false] {
//             led_all(leds, state);
//             arduino_hal::delay_ms(100);
//         }
//     }
// }

// fn led_all(leds: &mut [&mut Pin<Output>; 4], state: bool) {
//     for l in leds {
//         led_set_state(l, state);
//     }
// }

// #[rustfmt::skip]
// fn led_set_state(l: &mut Pin<Output>, state: bool) {
//     match state {
//         false => {l.set_low();},
//         true => {l.set_high();}
//     }
// }

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

    // let mut led_n = pins.d7.into_output().downgrade();
    // let mut led_w = pins.d6.into_output().downgrade();
    // let mut led_e = pins.d9.into_output().downgrade();
    // let mut led_s = pins.d8.into_output().downgrade();

    let btn = Btn {
        north: pins.d2.into_pull_up_input().downgrade(),
        east: pins.d3.into_pull_up_input().downgrade(),
        south: pins.d4.into_pull_up_input().downgrade(),
        west: pins.d5.into_pull_up_input().downgrade(),
    };

    // let btn_reset = pins.a2.into_pull_up_input().downgrade();
    let a_pin = pins.a0.into_analog_input(&mut adc);

    // pins and stuff used to control ws2812 LEDs
    let sck = pins.d13.into_output();
    let mosi = pins.d11.into_output();
    let miso = pins.d12.into_pull_up_input();
    let cs = pins.d10.into_output();
    let settings = spi::Settings::default();
    let (spi, _) = spi::Spi::new(dp.SPI, sck, mosi, miso, cs, settings);

    let mut ws = Ws2812::new(spi);

    // Run adc blocking read to ensure that arduino is ready
    _ = adc.read_blocking(&adc::channel::Vbg);
    ufmt::uwriteln!(&mut serial, "Hello Ruum42!\r").unwrap_infallible();

    // Testing some variable types
    //
    // ufmt::uwriteln!(&mut serial, "Type: {}\r", _get_type_of(&mut serial)).unwrap_infallible();
    // ufmt::uwriteln!(&mut serial, "{}\r", _get_type_of(&mut MAX_POS_SIZE)).unwrap_infallible();
    // ufmt::uwriteln!(&mut serial, "usize::MAX: {}\r", usize::MAX).unwrap_infallible();

    let seed = a_pin.analog_read(&mut adc);
    ufmt::uwriteln!(&mut serial, "Current seed: {}\r", seed).unwrap_infallible();
    let mut rng = SmallRng::seed_from_u64(seed as u64);
    let mut level = 1;
    let mut size_x = START_SIZE_X;
    let mut size_y = START_SIZE_Y;
    let mut li = Laby::new(size_x, size_y);

    // Statistics: Running a few hundred labys and calculate the max and avg jump_pos.
    // Also some code to test the rng
    //
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
    li.print(&mut serial);

    if btn.north.is_low() || btn.west.is_low() || btn.east.is_low() || btn.south.is_low() {
        find_btn_orientation(&btn, &mut ws);
    }

    ws_celebrate(&mut ws);

    let mut pos: isize = 2 + 2 * li.real_x;
    let mut wall_state = Wall {
        wall_n: false,
        wall_w: false,
        wall_e: false,
        wall_s: false,
    };
    // let led_arr = &mut [&mut led_n, &mut led_w, &mut led_e, &mut led_s];
    set_wall(&li, &pos, &mut wall_state);
    // led_show_wall(led_arr, &wall_state);
    ws_show_walls(&mut ws, &wall_state);

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
            btn_n: btn.north.is_low(),
            btn_w: btn.west.is_low(),
            btn_e: btn.east.is_low(),
            btn_s: btn.south.is_low(),
            // btn_reset: btn_reset.is_low(),
        };
        if  buttons.btn_n ||
            buttons.btn_w ||
            buttons.btn_e ||
            buttons.btn_s {
              last_pressed_buttons = buttons;
              buttons_were_pressed = true;
        } else if buttons_were_pressed {
            // led_all(&mut led_arr, false);

            if      last_pressed_buttons.btn_n && !wall_state.wall_n {pos += 2 * li.dirs[0];}
            else if last_pressed_buttons.btn_w && !wall_state.wall_w {pos += 2 * li.dirs[1];}
            else if last_pressed_buttons.btn_e && !wall_state.wall_e {pos += 2 * li.dirs[2];}
            else if last_pressed_buttons.btn_s && !wall_state.wall_s {pos += 2 * li.dirs[3];}
            else if last_pressed_buttons.btn_n && wall_state.wall_n  {ws_blink_wall(&mut ws, &wall_state, PIXEL_N);}
            else if last_pressed_buttons.btn_w && wall_state.wall_w  {ws_blink_wall(&mut ws, &wall_state, PIXEL_W);}
            else if last_pressed_buttons.btn_e && wall_state.wall_e  {ws_blink_wall(&mut ws, &wall_state, PIXEL_E);}
            else if last_pressed_buttons.btn_s && wall_state.wall_s  {ws_blink_wall(&mut ws, &wall_state, PIXEL_S);}
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
                    li.print(&mut serial);
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
                ws_celebrate(&mut ws);
                li.generate(&mut rng);
                regenerate_laby_next_lvl = false;
                // regenerate_laby_reset = false;
            }
            set_wall(&li, &pos, &mut wall_state);
            ws_show_walls(&mut ws, &wall_state);
            // led_show_wall(led_arr, &mut wall_state);
        }
        arduino_hal::delay_ms(50);
    }
}

fn off_if_low(button: &Pin<Input<PullUp>>, leds: &mut [RGB8], (from, to): (usize, usize)) {
    if button.is_low() {
        for i in from..to {
            leds[i] = RGB8::default();
        }
    }
}

fn find_btn_orientation(btn: &Btn, ws: &mut Ws2812<Spi>) -> ! {
    let mut data: [RGB8; NUM_LEDS] = [RGB8::default(); NUM_LEDS];
    let colors = [RED, YELLOW, GREEN, CYAN, BLUE, MAGENTA];

    let mut pos: u8 = 0;
    loop {
        for i in 0_u8..(NUM_LEDS as u8) {
            let color_index = (pos + i) as usize % colors.len();
            data[i as usize] = colors[color_index];
        }

        pos += 1;
        if pos >= NUM_LEDS as u8 {
            pos = 0;
        }

        off_if_low(&btn.north, &mut data, PIXEL_N);
        off_if_low(&btn.east, &mut data, PIXEL_E);
        off_if_low(&btn.south, &mut data, PIXEL_S);
        off_if_low(&btn.west, &mut data, PIXEL_W);

        ws.write(brightness(data.iter().cloned(), 25)).unwrap();
        arduino_hal::delay_ms(500);
    }
}

fn green_walls(state: &Wall, data: &mut [RGB8]) {
    if state.wall_n {
        for i in PIXEL_N.0..PIXEL_N.1 {
            data[i] = GREEN;
        }
    }
    if state.wall_e {
        for i in PIXEL_E.0..PIXEL_E.1 {
            data[i] = GREEN;
        }
    }
    if state.wall_s {
        for i in PIXEL_S.0..PIXEL_S.1 {
            data[i] = GREEN;
        }
    }
    if state.wall_w {
        for i in PIXEL_W.0..PIXEL_W.1 {
            data[i] = GREEN;
        }
    }
}

fn ws_show_walls(ws: &mut Ws2812<Spi>, state: &Wall) {
    let mut data: [RGB8; NUM_LEDS] = [RGB8::default(); NUM_LEDS];
    green_walls(&state, &mut data);
    ws.write(brightness(data.iter().cloned(), 25)).unwrap();
}

fn ws_blink_wall(ws: &mut Ws2812<Spi>, state: &Wall, (from, to): (usize, usize)) {
    let mut data: [RGB8; NUM_LEDS] = [RGB8::default(); NUM_LEDS];

    green_walls(&state, &mut data);
    for i in from..to {
        data[i] = RED;
    }
    ws.write(brightness(data.iter().cloned(), 25)).unwrap();
    arduino_hal::delay_ms(100);

    green_walls(&state, &mut data);
    ws.write(brightness(data.iter().cloned(), 25)).unwrap();
    arduino_hal::delay_ms(100);

    for i in from..to {
        data[i] = RED;
    }
    ws.write(brightness(data.iter().cloned(), 25)).unwrap();
    arduino_hal::delay_ms(100);

    green_walls(&state, &mut data);
    ws.write(brightness(data.iter().cloned(), 25)).unwrap();
}

fn ws_celebrate(ws: &mut Ws2812<Spi>) {
    let mut data: [RGB8; NUM_LEDS] = [RGB8::default(); NUM_LEDS];
    // let colors = [RED, YELLOW, GREEN, CYAN, BLUE, MAGENTA];

    for i in 0..NUM_LEDS {
        data[i] = BLUE;
    }
    ws.write(brightness(data.iter().cloned(), 25)).unwrap();
    arduino_hal::delay_ms(2000);

    // for pos in 0..colors.len() {
    //     for i in 0..NUM_LEDS {
    //         let color_index = pos + i % colors.len();
    //         data[i] = colors[color_index];
    //     }

    //     ws.write(brightness(data.iter().cloned(), 25)).unwrap();
    //     arduino_hal::delay_ms(250);
    // }
}
