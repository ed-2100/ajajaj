#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use rtic_monotonics::rp2040::prelude::*;
rp2040_timer_monotonic!(Mono);

use embassy_rp::{bind_interrupts, i2c, peripherals::I2C1};
bind_interrupts!(struct Irqs {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

#[rtic::app(device = embassy_rp, peripherals=true, dispatchers = [SWI_IRQ_0, SWI_IRQ_1, SWI_IRQ_2])]
mod app {
    use super::*;

    use embassy_rp::{gpio, i2c_slave, peripherals};
    use panic_probe as _;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        onboard_led_pin: peripherals::PIN_25,
        i2c: i2c_slave::I2cSlave<'static, I2C1>,
        aux_led_pin: peripherals::PIN_15,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        let p = embassy_rp::init(Default::default());

        let mut config = embassy_rp::i2c_slave::Config::default();
        config.addr = 0x17;
        config.general_call = false;
        let i2c = embassy_rp::i2c_slave::I2cSlave::new(p.I2C1, p.PIN_3, p.PIN_2, Irqs, config);

        // SAFETY: The `steal()` method used here is safe, provided that:
        // We assume that the TIMER peripheral isn't being used by anything else and that RESETS is already properly configured.
        unsafe {
            Mono::start(
                rtic_monotonics::rp2040::TIMER::steal(),
                &rtic_monotonics::rp2040::RESETS::steal(),
            );
        }

        on_i2c_event::spawn().ok();
        heartbeat::spawn().ok();

        cx.core.SCB.set_sleepdeep();

        (
            Shared {},
            Local {
                onboard_led_pin: p.PIN_25,
                i2c,
                aux_led_pin: p.PIN_15,
            },
        )
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }

    #[task(local = [onboard_led_pin], priority = 1)]
    async fn heartbeat(cx: heartbeat::Context) {
        let mut led = gpio::Output::new(cx.local.onboard_led_pin, gpio::Level::High);

        loop {
            Mono::delay(100.millis()).await;
            led.set_level(match led.get_output_level() {
                gpio::Level::Low => gpio::Level::High,
                gpio::Level::High => gpio::Level::Low,
            });
        }
    }

    #[task(local = [i2c, aux_led_pin], priority = 2)]
    async fn on_i2c_event(ctx: on_i2c_event::Context) {
        let mut aux_led = gpio::Output::new(ctx.local.aux_led_pin, gpio::Level::High);

        loop {
            let mut buf = [0u8; 128];

            match ctx.local.i2c.listen(&mut buf).await {
                Ok(i2c_slave::Command::GeneralCall(_)) => {}
                Ok(i2c_slave::Command::Read) => loop {
                    match ctx
                        .local
                        .i2c
                        .respond_to_read(&[aux_led.get_output_level() as u8])
                        .await
                    {
                        Ok(x) => match x {
                            i2c_slave::ReadStatus::Done => break,
                            i2c_slave::ReadStatus::NeedMoreBytes => {}
                            i2c_slave::ReadStatus::LeftoverBytes(_) => break,
                        },
                        Err(_) => {}
                    }
                },
                Ok(i2c_slave::Command::Write(len)) => aux_led.set_level((buf[len - 1] != 0).into()),
                Ok(i2c_slave::Command::WriteRead(len)) => loop {
                    match ctx
                        .local
                        .i2c
                        .respond_and_fill(&[aux_led.get_output_level() as u8], 0)
                        .await
                    {
                        Ok(x) => {
                            aux_led.set_level((buf[len - 1] != 0).into());

                            match x {
                                i2c_slave::ReadStatus::Done => break,
                                i2c_slave::ReadStatus::NeedMoreBytes => {}
                                i2c_slave::ReadStatus::LeftoverBytes(_) => break,
                            }
                        }
                        Err(_) => {}
                    }
                },
                Err(_) => {}
            }
        }
    }
}
