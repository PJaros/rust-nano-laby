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


#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    arduino_hal::delay_ms(1000);

    /*
     * For examples (and inspiration), head to
     *
     *     https://github.com/Rahix/avr-hal/tree/main/examples
     *
     * NOTE: Not all examples were ported to all boards!  There is a good chance though, that code
     * for a different board can be adapted for yours.  The Arduino Uno currently has the most
     * examples available.
     */

    let mut led = pins.d13.into_output();
    let mut led_n = pins.d7.into_output();
    let mut led_w = pins.d6.into_output();
    let mut led_e = pins.d9.into_output();
    let mut led_s = pins.d8.into_output();
    let btn_n = pins.d3.into_pull_up_input();
    let btn_w = pins.d2.into_pull_up_input();
    let btn_e = pins.d4.into_pull_up_input();
    let btn_s = pins.d5.into_pull_up_input();

    _ = ufmt::uwriteln!(&mut serial, "Hello Ruum42!\r");

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