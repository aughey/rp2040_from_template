//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use defmt::*;
use rp2040_project_template::{sine_wave, toggle};
use rp_pico::entry;

#[entry]
fn main() -> ! {
    info!("Program start");

    let (mut read, mut write, mut led) =
        rp2040_project_template::rp2040::initialize_pio_state_machines();

    let mut sinewave = sine_wave(441.0, 44100);
    let mut buf = [0u32; 400];
    for value in buf.iter_mut() {
        *value = sinewave();
    }

    led(true);

    let mut led_flash = toggle(44100);

    // push 0's into tx for the first sample so we
    // have one in the buffer
    _ = read();
    _ = read();
    write(0u32);
    write(0u32);
    write(0u32);
    write(0u32);
    loop {
        // // Toggle LED
        if let Some(on) = led_flash() {
            led(on);
        }

        // write(read());
        // write(read());
        //  _ = read();
        //  _ = read();
        // let value = sinewave();
        // let value = buf[bufindex];
        // bufindex += 1;
        // if bufindex == buf.len() {
        //     bufindex = 0;
        // }

        let valuesl = read();
        write(0);
        let valuesr = read();

        let sum = valuesl[0] as i64
            + valuesl[1] as i64
            + valuesl[2] as i64
            + valuesr[0] as i64
            + valuesr[1] as i64
            + valuesr[2] as i64;
        //  let sum = sum + valuesr[0] as i64; // + valuesr[1] as i64 + valuesr[2] as i64;

        const MAX: i64 = 2147483647;
        let sum = if sum > MAX {
            MAX
        } else if sum < -MAX {
            -MAX
        } else {
            sum
        };

        let l = sum as u32;
        //  write(l);
        write(l);

        // let sum = sum + values.iter().fold(sum, |acc, x| acc + (*x as i64));

        // // clamp sum to +/- 2^31-1

        //  write(l);
        // let value = sinewave();
        // write(value);
        // write(value);

        continue;
    }
}

// End of file
