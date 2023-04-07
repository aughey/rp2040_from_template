//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use core::f32::consts::PI;

use bsp::{entry, hal::prelude::_rphal_pio_PIOExt};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use micromath::F32Ext;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    //let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = {
        // External high-speed crystal on the pico board is 12Mhz
        const EXTERNAL_XTAL_FREQ_HZ: u32 = 12_000_000u32;
        init_clocks_and_plls(
            EXTERNAL_XTAL_FREQ_HZ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap()
    };

    //let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut tx = {
        // Define some simple PIO program.
        let program = {
            let program_with_defines = pio_proc::pio_asm!(
                // "set pindirs, 1",
                // ".wrap_target",

                // "set pins, 0 [10]",
                // "set pins, 1 [10]",
                // ".wrap",
                ".side_set 2",
                "                    ;        /--- LRCLK",
                "                    ;        |/-- BCLK",
                ".wrap_target        ;        ||",
                "bitloop1:           ;        ||",
                "    out pins, 1       side 0b10",
                "    jmp x-- bitloop1  side 0b11",
                "    out pins, 1       side 0b00",
               // "    set x, 14         side 0b01",
               "    set x, 30         side 0b01",
                "",
                "bitloop0:",
                "    out pins, 1       side 0b00",
                "    jmp x-- bitloop0  side 0b01",
                "    out pins, 1       side 0b10",
                "public entry_point:",
                // "    set x, 14         side 0b11",
                "    set x, 30         side 0b11",
                ".wrap"
                options(max_program_size = 32) // Optional, defaults to 32
            );
            program_with_defines.program
        };

        let _pin9 = pins.gpio9.into_mode::<bsp::hal::gpio::FunctionPio0>();
        let _pin10 = pins.gpio10.into_mode::<bsp::hal::gpio::FunctionPio0>();
        let _pin11 = pins.gpio11.into_mode::<bsp::hal::gpio::FunctionPio0>();

        // Initialize and start PIO
        let (mut pio, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);
        let installed = pio.install(&program).unwrap();

        let (int, frac) = {
            let system_clock: f64 = clocks.system_clock.freq().to_Hz() as f64;
            let bit_freq: f64 = 44100.0 * 32.0 * 2.0 * 2.0;
            let int: u16 = (system_clock / bit_freq) as u16;
            let frac: u8 = ((system_clock / bit_freq - int as f64) * 256.0) as u8;
            (int, frac)
        };

        let (mut sm, _, tx) = bsp::hal::pio::PIOBuilder::from_program(installed)
            .out_pins(9, 3) // I2S data pin
            .side_set_pin_base(10) // I2S Clock Pin
            .autopull(true)
            .clock_divisor_fixed_point(int, frac)
            .build(sm0);
        sm.set_pindirs(
            [
                (9, bsp::hal::pio::PinDir::Output),
                (10, bsp::hal::pio::PinDir::Output),
                (11, bsp::hal::pio::PinDir::Output),
            ]
            .into_iter(),
        );
        sm.start();

        // Load up a new state machine to run the 256fs clock
        // Define some simple PIO program.
        let program = {
            let program_with_defines = pio_proc::pio_asm!(
                ".wrap_target          ",
                "    set pins, 1       ",
                "    set pins, 0       ",
                ".wrap"
                options(max_program_size = 32) // Optional, defaults to 32
            );
            program_with_defines.program
        };

        // 256fs clock
        let _pin8 = pins.gpio8.into_mode::<bsp::hal::gpio::FunctionPio1>();

        let (mut pio, sm0, _, _, _) = pac.PIO1.split(&mut pac.RESETS);
        let installed = pio.install(&program).unwrap();

        let (int, frac) = {
            let system_clock: f64 = clocks.system_clock.freq().to_Hz() as f64;
            let fsclock: f64 = 256.0 * 44100.0 * 2.0;
            let int: u16 = (system_clock / fsclock) as u16;
            let frac: u8 = ((system_clock / fsclock - int as f64) * 256.0) as u8;
            (int, frac)
        };

        let (mut sm, _, _) = bsp::hal::pio::PIOBuilder::from_program(installed)
            // .out_pins(8, 1) // High Frequency I2S Sys Clock
            //.side_set_pin_base(10) // I2S Clock Pin
            //.autopull(true)
            .set_pins(8, 1)
            .clock_divisor_fixed_point(int, frac)
            .build(sm0);
        sm.set_pindirs([(8, bsp::hal::pio::PinDir::Output)].into_iter());
        sm.start();

        tx
    };

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here.
    //let mut led_pin = pins.gpio10.into_push_pull_output();
    let mut led_pin = pins.led.into_push_pull_output();
    let mut flip = false;

    let mut time = 0.0f32;
    let mut buf = [0u32; 400 * 2];
    let freq = 441.0f32;
    // Generate the 441hz sine wave in this buf
    let len = buf.len() / 2;
    for i in 0..len {
        let compute_value = |f| {
            let y = libm::sinf(time * f * 2.0 * PI);
            let y = y * 0.1;
            let y = (y * 2147483648.0) as i32;
            // Convert i to u without changing the bit pattern
            let y = unsafe { core::mem::transmute::<i32, u32>(y) };
            y
        };
        let y = compute_value(freq);
        buf[i * 2 + 0] = y;
        let y = compute_value(freq * 2.0);
        buf[i * 2 + 1] = y;
        time += 1.0 / 44100.0;
        if time > PI * 2.0 {
            time -= PI * 2.0;
        }

        //let y = y as u32;
        //let y = y << 16 | y;
    }
    led_pin.set_high().unwrap();

    let mut bufindex = 0;
    time = 0.0;
    loop {
        // // Toggle LED
        flip = !flip;
        match flip {
            true => led_pin.set_high().unwrap(),
            false => led_pin.set_low().unwrap(),
        }

        // 440.0hz wave at 44100.0hz sample rate
        // let y = libm::sinf(time * 440.0 * 2.0 * PI);
        // let y = y / 2.0 + 0.5;
        // let y = (y * 32767.0) as i32;
        // time += 1.0 / 44100.0;
        // if time > PI * 2.0 {
        //     time -= PI * 2.0;
        // }
        // // Convert i32 to u32 without changing the bit pattern
        // let y = unsafe { core::mem::transmute::<i32, u32>(y) };
        // let y = y << 16 | y;
        // let y = 0xff00_ff00;

        let y = buf[bufindex];
        bufindex += 1;
        if bufindex >= buf.len() {
            bufindex = 0;
        }
        // let y = (time * 440.0 * 2.0 * PI).sin();
        // let y = y * 0.1;
        // let y = (y * 2147483648.0) as i32;
        // // Convert i to u without changing the bit pattern
        // let y = unsafe { core::mem::transmute::<i32, u32>(y) };

        // while !tx.write(y) {
        //     cortex_m::asm::nop();
        // }

        while !tx.write(y) {
            cortex_m::asm::nop();
        }

        time += 1.0 / 44100.0;
        if time > PI * 2.0 {
            time -= PI * 2.0;
        }
    }
}

// End of file
