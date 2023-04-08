#![no_std]

use core::f32::consts::PI;

pub mod rp2040;

/// Toggle will return a closure that will toggle a boolean value every `count` times it is called.
pub fn toggle(count: usize) -> impl FnMut() -> Option<bool> {
    let mut curcount = count;
    let mut toggle_value = false;
    move || {
        curcount -= 1;
        if curcount == 0 {
            curcount = count;
            toggle_value = !toggle_value;
            Some(toggle_value)
        } else {
            None
        }
    }
}

/// sine_wave will return an iterator that will generate a sine wave at the given frequency and sample rate.
pub fn sine_wave(_freq: f32, sample_rate: usize) -> impl FnMut() -> u32 {
    let mut time = 0.0f32;
    let mut buf = [0u32; 400];
    let freq = 441.0f32;
    let sample_rate = sample_rate as f32;
    // Generate the 441hz sine wave in this buf

    let mut next_value = || {
        let y = libm::sinf(time * freq * 2.0 * PI);
        let y = y * 1.0;
        let y = (y * 8388608.0) as i32;
        // Convert i to u without changing the bit pattern
        time += 1.0 / sample_rate;
        if time > PI * 2.0 {
            time -= PI * 2.0;
        }

        y as u32
    };

    for value in buf.iter_mut() {
        *value = next_value();
    }

    let mut bufindex = 0usize;
    return move || {
        let value = buf[bufindex];
        bufindex += 1;
        if bufindex == buf.len() {
            bufindex = 0;
        }
        value
    };

    // repeat the buffer endlessly
    // buf.into_iter().cycle()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
