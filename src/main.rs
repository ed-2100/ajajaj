#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use rtic_monotonics::rp2040::prelude::*;

rp2040_timer_monotonic!(Mono);

use rp2040_hal as hal;

#[rtic::app(device = hal::pac)]
mod app {
    use super::*;

    use hal::{
        clocks,
        gpio::{
            self,
            bank0::{Gpio15, Gpio2, Gpio25, Gpio3},
            FunctionI2C, FunctionSio, Pin, PullNone, SioOutput,
        },
        i2c, pac,
        sio::Sio,
        watchdog::Watchdog,
    };

    use core::mem::MaybeUninit;
    use embedded_hal::digital::{OutputPin as _, StatefulOutputPin as _};
    use panic_probe as _;

    type I2CBus = hal::I2C<
        pac::I2C1,
        (
            Pin<Gpio2, FunctionI2C, PullNone>,
            Pin<Gpio3, FunctionI2C, PullNone>,
        ),
        i2c::Peripheral,
    >;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        onboard_led: Pin<Gpio25, FunctionSio<SioOutput>, PullNone>,
        i2c: &'static mut I2CBus,
        aux_led: Pin<Gpio15, FunctionSio<SioOutput>, PullNone>,
    }

    #[init(local=[
        i2c_ctx: MaybeUninit<I2CBus> = MaybeUninit::uninit()
    ])]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        Mono::start(ctx.device.TIMER, &mut ctx.device.RESETS);

        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);

        let _clocks = clocks::init_clocks_and_plls(
            12_000_000,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(ctx.device.SIO);

        let gpioa = gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );

        let mut onboard_led: Pin<_, FunctionSio<SioOutput>, PullNone> = gpioa.gpio25.reconfigure();
        let mut aux_led: Pin<_, FunctionSio<SioOutput>, PullNone> = gpioa.gpio15.reconfigure();

        onboard_led.set_high().unwrap();
        aux_led.set_high().unwrap();

        let sda_pin: Pin<_, FunctionI2C, _> = gpioa.gpio2.reconfigure();
        let scl_pin: Pin<_, FunctionI2C, _> = gpioa.gpio3.reconfigure();

        let i2c: &'static mut _ = ctx
            .local
            .i2c_ctx
            .write(hal::I2C::new_peripheral_event_iterator(
                ctx.device.I2C1,
                sda_pin,
                scl_pin,
                &mut ctx.device.RESETS,
                0x17u8,
            ));

        heartbeat::spawn().ok();

        (
            Shared {},
            Local {
                onboard_led,
                i2c,
                aux_led,
            },
        )
    }

    #[task(local = [onboard_led])]
    async fn heartbeat(ctx: heartbeat::Context) {
        loop {
            ctx.local.onboard_led.toggle().unwrap();
            Mono::delay(1.secs()).await;
        }
    }

    #[task(local = [i2c, aux_led])]
    async fn on_i2c_event(ctx: on_i2c_event::Context) {
        use i2c::peripheral::Event;

        loop {
            let event = ctx.local.i2c.wait_next().await;

            match event {
                Event::Start => {}
                Event::Restart => {}
                Event::TransferRead => {}
                Event::TransferWrite => {}
                Event::Stop => {}
            }
        }
    }
}
