//! Rainbow effect color wheel using the onboard NeoPixel on an Adafruit Feather RP2040 board
//!
//! This flows smoothly through various colors on the onboard NeoPixel.
//! Uses the `ws2812_pio` driver to control the NeoPixel, which in turns uses the
//! RP2040's PIO block.
#![no_std]
#![no_main]

use adafruit_feather_rp2040::entry;
use adafruit_feather_rp2040::{
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio,
        i2c::I2C,
        pac,
        pio::PIOExt,
        pwm,
        timer::Timer,
        watchdog::Watchdog,
        Sio,
    },
    Pins, XOSC_CRYSTAL_FREQ,
};
use core::iter::once;
use cortex_m::delay::Delay;
use ds1307::{Ds1307, Rtcc, Timelike};
use embedded_hal::timer::CountDown;
use embedded_hal::PwmPin;
use fugit::{ExtU32, RateExtU32};
use hd44780_driver::{Cursor, CursorBlink, Display, DisplayMode, HD44780};
use heapless::String;
use panic_halt as _;
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ufmt::uwrite;
use ws2812_pio::Ws2812;

const LCD_I2C_ADDRESS: u8 = 0x20;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sda_pin = pins.sda.into_mode::<gpio::FunctionI2C>();
    let scl_pin = pins.scl.into_mode::<gpio::FunctionI2C>();

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let i2c = I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let bus = shared_bus::BusManagerSimple::new(i2c);

    let mut rtc_proxy = bus.acquire_i2c();
    let mut lcd_proxy = bus.acquire_i2c();

    // DS1307 RTC
    let mut rtc = Ds1307::new(rtc_proxy);

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut neopixel_delay = timer.count_down();

    // Configure the addressable LED
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        // The onboard NeoPixel is attached to GPIO pin #16 on the Feather RP2040.
        pins.neopixel.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    // Infinite colour wheel loop
    let mut n: u8 = 128;

    // LCD setup
    let mut lcd_delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // PWM for the LCD backlight
    // Right now we're plugged into PWM slice 6, using both channels
    let pwm = &mut pwm_slices.pwm6;
    pwm.set_ph_correct();
    pwm.enable();

    let channel_a = &mut pwm.channel_a;
    channel_a.output_to(pins.a2);
    channel_a.set_duty(10000);

    let channel_b = &mut pwm.channel_b;
    channel_b.output_to(pins.a3);
    channel_b.set_duty(10000);

    // The LCD itself
    let mut lcd = HD44780::new_i2c_mcp23008(lcd_proxy, LCD_I2C_ADDRESS, &mut lcd_delay).unwrap();
    lcd.reset(&mut lcd_delay).expect("Could not reset display");
    lcd.clear(&mut lcd_delay).expect("Could not clear display");
    lcd.set_display_mode(
        DisplayMode {
            display: Display::On,
            cursor_visibility: Cursor::Invisible,
            cursor_blink: CursorBlink::Off,
        },
        &mut lcd_delay,
    )
    .expect("Could not set display mode");
    lcd.write_str("Hello, world!", &mut lcd_delay)
        .expect("Could not write to LCD.");
    lcd.set_cursor_xy((0, 1), &mut lcd_delay)
        .expect("Could not update cursor position.");

    loop {
        let mut time_display: String<16> = String::new();
        let time = rtc.time().expect("Could not get time from RTC.");
        let (pm, hour) = time.hour12();

        if hour < 10 {
            uwrite!(time_display, "{}", " ");
        };
        uwrite!(time_display, "{}:", hour);

        if time.minute() < 10 {
            uwrite!(time_display, "{}", "0");
        };
        uwrite!(time_display, "{} ", time.minute());

        uwrite!(
            time_display,
            "{}",
            match pm {
                true => "PM",
                false => "AM",
            }
        );

        lcd.clear(&mut lcd_delay).expect("Could not clear display");

        lcd.write_str(&time_display, &mut lcd_delay);
        ws.write(brightness(once(wheel(n)), 32)).unwrap();
        n = n.wrapping_add(1);

        neopixel_delay.start(25.millis());
        let _ = nb::block!(neopixel_delay.wait());
    }
}

/// Convert a number from `0..=255` to an RGB color triplet.
///
/// The colours are a transition from red, to green, to blue and back to red.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        // No green in this sector - red and blue only
        (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
    } else if wheel_pos < 170 {
        // No red in this sector - green and blue only
        wheel_pos -= 85;
        (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
    } else {
        // No blue in this sector - red and green only
        wheel_pos -= 170;
        (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
    }
}
