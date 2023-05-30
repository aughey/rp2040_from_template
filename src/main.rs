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

    let mut read8 = move || {
        let mut outvalues: [u32; 8] = [0; 8];
        for read_num in 0..8 {
            let values = read();
            // Brute force this to show the pattern.
            let shift = read_num * 4;

            let pin0 = 0
                | ((values & 0b00000000000000000000000000000001) >> (0 + 0 - 0 + shift))
                | ((values & 0b00000000000000000000000100000000) >> (0 + 8 - 1 + shift))
                | ((values & 0b00000000000000010000000000000000) >> (0 + 16 - 2 + shift))
                | ((values & 0b00000001000000000000000000000000) >> (0 + 24 - 3 + shift));
            let pin1 = 0
                | ((values & 0b00000000000000000000000000000010) >> (1 + 0 - 0 + shift))
                | ((values & 0b00000000000000000000001000000000) >> (1 + 8 - 1 + shift))
                | ((values & 0b00000000000000100000000000000000) >> (1 + 16 - 2 + shift))
                | ((values & 0b00000010000000000000000000000000) >> (1 + 24 - 3 + shift));
            let pin2 = 0
                | ((values & 0b00000000000000000000000000000100) >> (2 + 0 - 0 + shift))
                | ((values & 0b00000000000000000000010000000000) >> (2 + 8 - 1 + shift))
                | ((values & 0b00000000000001000000000000000000) >> (2 + 16 - 2 + shift))
                | ((values & 0b00000100000000000000000000000000) >> (2 + 24 - 3 + shift));
            let pin3 = 0
                | ((values & 0b00000000000000000000000000001000) >> (3 + 0 - 0 + shift))
                | ((values & 0b00000000000000000000100000000000) >> (3 + 8 - 1 + shift))
                | ((values & 0b00000000000010000000000000000000) >> (3 + 16 - 2 + shift))
                | ((values & 0b00001000000000000000000000000000) >> (3 + 24 - 3 + shift));
            let pin4 = 0
                | ((values & 0b00000000000000000000000000010000) >> (4 + 0 - 0 + shift))
                | ((values & 0b00000000000000000001000000000000) >> (4 + 8 - 1 + shift))
                | ((values & 0b00000000000100000000000000000000) >> (4 + 16 - 2 + shift))
                | ((values & 0b00010000000000000000000000000000) >> (4 + 24 - 3 + shift));
            let pin5 = 0
                | ((values & 0b00000000000000000000000000100000) >> (5 + 0 - 0 + shift))
                | ((values & 0b00000000000000000010000000000000) >> (5 + 8 - 1 + shift))
                | ((values & 0b00000000001000000000000000000000) >> (5 + 16 - 2 + shift))
                | ((values & 0b00100000000000000000000000000000) >> (5 + 24 - 3 + shift));
            let pin6 = 0
                | ((values & 0b00000000000000000000000001000000) >> (6 + 0 - 0 + shift))
                | ((values & 0b00000000000000000100000000000000) >> (6 + 8 - 1 + shift))
                | ((values & 0b00000000010000000000000000000000) >> (6 + 16 - 2 + shift))
                | ((values & 0b01000000000000000000000000000000) >> (6 + 24 - 3 + shift));
            let pin7 = 0
                | ((values & 0b00000000000000000000000010000000) >> (7 + 0 - 0 + shift))
                | ((values & 0b00000000000000001000000000000000) >> (7 + 8 - 1 + shift))
                | ((values & 0b00000000100000000000000000000000) >> (7 + 16 - 2 + shift))
                | ((values & 0b10000000000000000000000000000000) >> (7 + 24 - 3 + shift));

            outvalues[0] |= pin0;
            outvalues[1] |= pin1;
            outvalues[2] |= pin2;
            outvalues[3] |= pin3;
            outvalues[4] |= pin4;
            outvalues[5] |= pin5;
            outvalues[6] |= pin6;
            outvalues[7] |= pin7;
        }
        outvalues
    };

    let mut sinewave = sine_wave(441.0, 44100);
    let mut buf = [0u32; 400];
    for value in buf.iter_mut() {
        *value = sinewave();
    }

    led(true);

    let mut led_flash = toggle(44100);

    // push 0's into tx for the first sample so we
    // have one in the buffer
    _ = read8();
    _ = read8();
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

        let sum = {
            // each read gets 4 values from 8 inputs
            let valuesl = read8();
            let valuesr = read8();

            // let valuesl = read();
            // let valuesr = read();

            let to_i64 = |value| Into::<i64>::into(value as i32);

            let sum = 0
                + to_i64(valuesl[0])
                // + to_i64(valuesl[1])
                // + to_i64(valuesl[2])
                + to_i64(valuesr[0])
                // + to_i64(valuesr[1])
                // + to_i64(valuesr[2])
                ;

            const MAX: i64 = 2147483647;
            let sum = if sum > MAX {
                MAX
            } else if sum < -MAX {
                -MAX
            } else {
                sum
            };
            sum as u32
        };

        //let sum = sinewave();

        write(sum);
        write(sum);

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
