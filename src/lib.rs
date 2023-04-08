#![no_std]

use core::f32::consts::PI;

pub mod rp2040;

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

pub fn sine_wave(_freq: f32, sample_rate: usize) -> impl Iterator<Item = u32> {
    let mut time = 0.0f32;
    let mut buf = [0u32; 400 * 2];
    let freq = 441.0f32;
    let sample_rate = sample_rate as f32;
    // Generate the 441hz sine wave in this buf
    let len = buf.len();

    for i in 0..len {
        let compute_value = |f| {
            let y = libm::sinf(time * f * 2.0 * PI);
            let y = y * 0.1;
            let y = (y * 2147483648.0) as i32;
            // Convert i to u without changing the bit pattern
            y as u32
        };

        let y = compute_value(freq);
        buf[i] = y;

        time += 1.0 / sample_rate;
        if time > PI * 2.0 {
            time -= PI * 2.0;
        }

        //let y = y as u32;
        //let y = y << 16 | y;
    }

    // repeat the buffer endlessly
    buf.into_iter().cycle()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
