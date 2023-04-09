use bsp::hal::prelude::_rphal_pio_PIOExt;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
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

pub fn initialize_pio_state_machines(
) -> (impl FnMut() -> [u32; 3], impl FnMut(u32), impl FnMut(bool)) {
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

    // Load and generate the tx,rx channels to the FIFOs
    let (mut tx, rx) = {
        // Define some simple PIO program.

        // Load up a new state machine to run the 256fs clock
        // Define some simple PIO program.

        // All state machines will run at this 256fs system clock
        // and do delays of [3].
        let (system_clock_int, system_clock_frac) = {
            let system_clock: f64 = clocks.system_clock.freq().to_Hz() as f64;
            let fsclock: f64 = 256.0 * 44100.0 * 2.0;
            let int: u16 = (system_clock / fsclock) as u16;
            let frac: u8 = ((system_clock / fsclock - int as f64) * 256.0) as u8;
            (int, frac)
        };

        // PIO0
        let tx = {
            // Initialize and start PIO
            let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

            let _pin9 = pins.gpio9.into_mode::<bsp::hal::gpio::FunctionPio0>();
            let _pin10 = pins.gpio10.into_mode::<bsp::hal::gpio::FunctionPio0>();
            let _pin11 = pins.gpio11.into_mode::<bsp::hal::gpio::FunctionPio0>();

            let installed_writer = {
                let program_output = {
                    let program_with_defines = pio_proc::pio_asm!(
                        ".side_set 2",
                        "                    ;       /--- LRCLK",
                        "                    ;       |/-- BCLK",
                        "public entry_point: ;       ||",
                        "    irq wait 0       side 0b11 [3]", // wait to be signaled by the system clock
                        "    set x, 29        side 0b00 [3]", // Last LSB of right channel
                        "    nop              side 0b01 [3]",
                        ".wrap_target        ;        ",
                        "bitloop0:",
                        "    out pins, 1      side 0b00 [3]", // 31 times
                        "    jmp x-- bitloop0 side 0b01 [3]",
                        "    out pins, 1      side 0b10 [3]", // 32nd time (LSB)
                        "    set x, 30        side 0b11 [3]",
                        "bitloop1:           ;        ",
                        "    out pins, 1      side 0b10 [3]", // 31 times
                        "    jmp x-- bitloop1 side 0b11 [3]",
                        "    out pins, 1      side 0b00 [3]", // 32nd time (LSB)
                        "    set x, 30        side 0b01 [3]",
                        ".wrap"
                        options(max_program_size = 32) // Optional, defaults to 32
                    );
                    program_with_defines.program
                };
                pio.install(&program_output).unwrap()
            };

            // let (int, frac) = {
            //     let system_clock: f64 = clocks.system_clock.freq().to_Hz() as f64;
            //     let bit_freq: f64 = 44100.0 * 32.0 * 2.0 * 2.0;
            //     let int: u16 = (system_clock / bit_freq) as u16;
            //     let frac: u8 = ((system_clock / bit_freq - int as f64) * 256.0) as u8;
            //     (int, frac)
            // };

            let (mut sm, _, tx) = bsp::hal::pio::PIOBuilder::from_program(installed_writer)
                .out_pins(9, 3) // I2S data pin
                .side_set_pin_base(10) // I2S Clock Pin
                .autopull(true)
                .pull_threshold(32)
                .clock_divisor_fixed_point(system_clock_int, system_clock_frac)
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
            tx
        };

        // PIO1
        let rx = {
            let (mut pio, sm0, sm1, sm2, sm3) = pac.PIO1.split(&mut pac.RESETS);

            // Receive
            let rx = {
                let _pin6 = pins.gpio6.into_mode::<bsp::hal::gpio::FunctionPio1>();
                let _pin7 = pins.gpio7.into_mode::<bsp::hal::gpio::FunctionPio1>();
                let _pin8 = pins.gpio8.into_mode::<bsp::hal::gpio::FunctionPio1>();

                let installed_reader = {
                    // Define some simple PIO program.
                    let program_input = {
                        let program_with_defines = pio_proc::pio_asm!(
                            ".side_set 2",
                            "                    ;       /--- LRCLK",
                            "                    ;       |/-- BCLK",
                            "public entry_point: ;       ||",
                            "    irq wait 0       side 0b11 [3]", // wait to be signaled by the system clock
                            "    set x, 29        side 0b00 [3]",
                            "    nop              side 0b01 [3]", // Last LSB of right channel
                            "    nop              side 0b00 [3]",
                            ".wrap_target        ;        ",
                            "bitloop0:",
                            "    in pins, 1       side 0b01 [3]", // 30 times
                            "    jmp x-- bitloop0 side 0b00 [3]",
                            "    in pins, 1       side 0b01 [3]", // 31st time
                            "    nop              side 0b10 [3]", // (raise LRCLK)
                            "    in pins, 1       side 0b11 [3]", // 32nd time (LSB)
                            "    set x, 29        side 0b10 [3]",
                            "bitloop1:           ;        ",
                            "    in pins, 1       side 0b11 [3]", // 30 times
                            "    jmp x-- bitloop1 side 0b10 [3]",
                            "    in pins, 1       side 0b11 [3]", // 31st time
                            "    nop              side 0b00 [3]", // (lower LRCLK)
                            "    in pins, 1       side 0b01 [3]", // 32nd time (LSB)
                            "    set x, 29        side 0b00 [3]",
                            ".wrap"
                            options(max_program_size = 32) // Optional, defaults to 32
                        );
                        program_with_defines.program
                    };

                    pio.install(&program_input).unwrap()
                };

                let (mut sm, mut rx0, _) =
                    bsp::hal::pio::PIOBuilder::from_program(unsafe { installed_reader.share() })
                        .in_pin_base(6) // I2S data pin
                        .side_set_pin_base(7) // I2S Clock Pin
                        .autopush(true)
                        .push_threshold(32)
                        .clock_divisor_fixed_point(system_clock_int, system_clock_frac)
                        .build(sm0);

                sm.set_pindirs(
                    [
                        (6, bsp::hal::pio::PinDir::Input),
                        (7, bsp::hal::pio::PinDir::Output),
                        (8, bsp::hal::pio::PinDir::Output),
                    ]
                    .into_iter(),
                );
                sm.start();

                // SECOND READER
                // install the same thing on sm1 (expecting another one there, but not actually using it)
                let _pin4 = pins.gpio4.into_mode::<bsp::hal::gpio::FunctionPio1>();
                let (mut sm, mut rx1, _) =
                    bsp::hal::pio::PIOBuilder::from_program(unsafe { installed_reader.share() })
                        .in_pin_base(4) // I2S data pin
                        //.side_set_pin_base(7) // I2S Clock Pin
                        .autopush(true)
                        .push_threshold(32)
                        .clock_divisor_fixed_point(system_clock_int, system_clock_frac)
                        .build(sm1);
                sm.set_pindirs([(4, bsp::hal::pio::PinDir::Input)].into_iter());
                sm.start();

                // THIRD READER
                // install the same thing on sm2 (expecting another one there, but not actually using it)
                let _pin3 = pins.gpio3.into_mode::<bsp::hal::gpio::FunctionPio1>();
                let (mut sm, mut rx2, _) =
                    bsp::hal::pio::PIOBuilder::from_program(installed_reader)
                        .in_pin_base(3) // I2S data pin
                        //.side_set_pin_base(7) // I2S Clock Pin
                        .autopush(true)
                        .push_threshold(32)
                        .clock_divisor_fixed_point(system_clock_int, system_clock_frac)
                        .build(sm2);
                sm.set_pindirs([(3, bsp::hal::pio::PinDir::Input)].into_iter());
                sm.start();

                move || {
                    let value0 = loop {
                        if let Some(value) = rx0.read() {
                            break value;
                        }
                    };
                    let value1 = loop {
                        if let Some(value) = rx1.read() {
                            break value;
                        }
                    };
                    let value2 = loop {
                        if let Some(value) = rx2.read() {
                            break value;
                        }
                    };
                    [value0, value1, value2]
                }
            };

            // 256fs clock
            {
                let installed_system_clock = {
                    let program_system_clock = {
                        let program_with_defines = pio_proc::pio_asm!(
                            "irq clear 0           ", // Launch all programs waiting
                            ".wrap_target          ",
                            "    set pins, 1       ",
                            "    set pins, 0       ",
                            ".wrap"
                            options(max_program_size = 32) // Optional, defaults to 32
                        );
                        program_with_defines.program
                    };
                    pio.install(&program_system_clock).unwrap()
                };

                let _pin5 = pins.gpio5.into_mode::<bsp::hal::gpio::FunctionPio1>();

                let (mut sm, _, _) =
                    bsp::hal::pio::PIOBuilder::from_program(installed_system_clock)
                        // .out_pins(8, 1) // High Frequency I2S Sys Clock
                        //.side_set_pin_base(10) // I2S Clock Pin
                        //.autopull(true)
                        .set_pins(5, 1)
                        .clock_divisor_fixed_point(system_clock_int, system_clock_frac)
                        .build(sm3);
                sm.set_pindirs([(5, bsp::hal::pio::PinDir::Output)].into_iter());
                sm.start();
            }

            rx
        };

        (tx, rx)
    };

    let write = move |value| {
        while !tx.write(value) {
            cortex_m::asm::nop();
        }
    };

    let mut led_pin = pins.led.into_push_pull_output();

    let led_on = move |on| match on {
        true => led_pin.set_high().unwrap(),
        false => led_pin.set_low().unwrap(),
    };

    (rx, write, led_on)
}
