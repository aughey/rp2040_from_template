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

pub fn initialize_pio_state_machines() -> (impl FnMut() -> u32, impl FnMut(u32), impl FnMut(bool)) {
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
    let (mut tx, mut rx) = {
        // Define some simple PIO program.

        // Load up a new state machine to run the 256fs clock
        // Define some simple PIO program.

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
                        "                    ;        /--- LRCLK",
                        "                    ;        |/-- BCLK",
                        ".wrap_target        ;        ||",
                        "bitloop1:           ;        ||",
                        "    out pins, 1       side 0b10",
                        "    jmp x-- bitloop1  side 0b11",
                        "    out pins, 1       side 0b00",
                       "    set x, 30         side 0b01",
                        "",
                        "bitloop0:",
                        "    out pins, 1       side 0b00",
                        "    jmp x-- bitloop0  side 0b01",
                        "    out pins, 1       side 0b10",
                        "public entry_point:",
                        "    set x, 30         side 0b11",
                        ".wrap"
                        options(max_program_size = 32) // Optional, defaults to 32
                    );
                    program_with_defines.program
                };
                pio.install(&program_output).unwrap()
            };

            let (int, frac) = {
                let system_clock: f64 = clocks.system_clock.freq().to_Hz() as f64;
                let bit_freq: f64 = 44100.0 * 32.0 * 2.0 * 2.0;
                let int: u16 = (system_clock / bit_freq) as u16;
                let frac: u8 = ((system_clock / bit_freq - int as f64) * 256.0) as u8;
                (int, frac)
            };

            let (mut sm, _, tx) = bsp::hal::pio::PIOBuilder::from_program(installed_writer)
                .out_pins(9, 3) // I2S data pin
                .side_set_pin_base(10) // I2S Clock Pin
                .autopull(true)
                .pull_threshold(32)
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
            tx
        };

        // PIO1
        let rx = {
            let (mut pio, sm0, sm1, _, _) = pac.PIO1.split(&mut pac.RESETS);
            // 256fs clock
            // Both state machines share the same clock because we want the
            // system clock and the i2s master (reader) to be synchronized
            let (int, frac) = {
                let installed_system_clock = {
                    let program_system_clock = {
                        let program_with_defines = pio_proc::pio_asm!(
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

                let (int, frac) = {
                    let system_clock: f64 = clocks.system_clock.freq().to_Hz() as f64;
                    let fsclock: f64 = 256.0 * 44100.0 * 2.0;
                    let int: u16 = (system_clock / fsclock) as u16;
                    let frac: u8 = ((system_clock / fsclock - int as f64) * 256.0) as u8;
                    (int, frac)
                };

                let _pin5 = pins.gpio5.into_mode::<bsp::hal::gpio::FunctionPio1>();

                let (mut sm, _, _) =
                    bsp::hal::pio::PIOBuilder::from_program(installed_system_clock)
                        // .out_pins(8, 1) // High Frequency I2S Sys Clock
                        //.side_set_pin_base(10) // I2S Clock Pin
                        //.autopull(true)
                        .set_pins(5, 1)
                        .clock_divisor_fixed_point(int, frac)
                        .build(sm0);
                sm.set_pindirs([(5, bsp::hal::pio::PinDir::Output)].into_iter());
                sm.start();

                (int, frac)
            };

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
                            "                    ;        /--- LRCLK",
                            "                    ;        |/-- BCLK",
                            ".wrap_target        ;        ||",
                            "bitloop1:           ;        ||",
                            // A delay of 4 throughout because we run on the same clock as the system clock
                            "    in pins, 1       side 0b10 [4]",
                            "    jmp x-- bitloop1 side 0b11 [4]",
                            "    in pins, 1       side 0b00 [4]",
                           "    set x, 30         side 0b01 [4]",
                            "",
                            "bitloop0:",
                            "    in pins, 1       side 0b00 [4]",
                            "    jmp x-- bitloop0 side 0b01 [4]",
                            "    in pins, 1       side 0b10 [4]",
                            "public entry_point:",
                            "    set x, 30        side 0b11 [4]",
                            ".wrap"
                            options(max_program_size = 32) // Optional, defaults to 32
                        );
                        program_with_defines.program
                    };

                    pio.install(&program_input).unwrap()
                };

                let (mut sm, rx, _) = bsp::hal::pio::PIOBuilder::from_program(installed_reader)
                    .in_pin_base(6) // I2S data pin
                    .side_set_pin_base(7) // I2S Clock Pin
                    .autopush(true)
                    .push_threshold(32)
                    .clock_divisor_fixed_point(int, frac)
                    .build(sm1);

                sm.set_pindirs(
                    [
                        (6, bsp::hal::pio::PinDir::Input),
                        (7, bsp::hal::pio::PinDir::Output),
                        (8, bsp::hal::pio::PinDir::Output),
                    ]
                    .into_iter(),
                );
                sm.start();
                rx
            };
            rx
        };

        (tx, rx)
    };

    let write = move |value| {
        while !tx.write(value) {
            cortex_m::asm::nop();
        }
    };

    let read = move || loop {
        let value = rx.read();
        if let Some(value) = value {
            return value;
        }
        cortex_m::asm::nop();
    };
    let mut led_pin = pins.led.into_push_pull_output();

    let led_on = move |on| match on {
        true => led_pin.set_high().unwrap(),
        false => led_pin.set_low().unwrap(),
    };

    (read, write, led_on)
}
