# Links

Information found in the web past searches and/or future investigations:

## Rust
- Modules: https://rust-classes.com/chapter_4_3
- OOP: https://doc.rust-lang.org/book/ch17-01-what-is-oo.html

## Rust on Arduino
- rahix-avr: https://github.com/Rahix/avr-hal
- arduino: https://rahix.github.io/avr-hal/arduino_hal/index.html
- embedded-hal: https://crates.io/crates/embedded-hal/0.2.3

## Optimize Compile
- Cargo.toml Profile: https://doc.rust-lang.org/cargo/reference/profiles.html

## Analyse Compile
- cargo-bloat: https://crates.io/crates/cargo-bloat/
- Rust Unstable Book, emit-stack-sizes: https://doc.rust-lang.org/stable/unstable-book/compiler-flags/emit-stack-sizes.html 

## Memory layout with emulation
- Embedonomicon, QEMU: https://docs.rust-embedded.org/embedonomicon/memory-layout.html#testing-it
- AVR System emulator, QEMU Project: https://qemu-project.gitlab.io/qemu/system/target-avr.html
- Rahix, QEMU example: https://github.com/Rahix/avr-hal/discussions/280

## Arduino LED 
- https://github.com/SIMULATAN/arduino-led-rs/tree/main

## Social Media
- Rust Programming Discord: https://discord.com/invite/rust-lang-community
- Wokwi Discord: https://wokwi.com/discord

## Random number generator
- std RNG: https://stackoverflow.com/a/37017052/406423
- embedded-hal, rng: https://github.com/rust-embedded/embedded-hal/issues/128
- rand::RngCore: https://docs.rs/rand/0.6.5/rand/trait.RngCore.html
- crates.io randCore: https://crates.io/crates/rand_core

## Alternative RNG implementation (unused)
- small_rng example: https://stackoverflow.com/a/67652214/406423
- micro_rand: https://crates.io/crates/micro_rand
- LCG: https://en.wikipedia.org/wiki/Linear_congruential_generator
- CLCG: https://en.wikipedia.org/wiki/Combined_linear_congruential_generator

## Live Event
- Linuxday: https://www.linuxday.at/

## Useful Linux commands:
- Detect USB-Port: journalctl -k -f
- Build: RAVEDUDE_PORT=/dev/ttyUSB0 cargo build --release
