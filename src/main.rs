#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

// extern crate alloc;
// use embedded_alloc::Heap;
// #[global_allocator]
// static HEAP: Heap = Heap::empty();

use panic_probe as _;

use fixed::types::{U16F16, U28F4};
use rtic_monotonics::rp2040::prelude::*;
rp2040_timer_monotonic!(Mono);

use embassy_rp::{bind_interrupts, i2c, peripherals::I2C1};
bind_interrupts!(struct Irqs {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

const fn top_duty_to_comp(top: u16, duty: U16F16) -> u16 {
    ((((top + 1) as u32) * duty.to_bits()) >> 16) as _
}

#[derive(Debug)]
enum SetFreqDutyError {
    FrequencyTooLow,
    FrequencyTooHigh,
}

/// Configures `top`, `divider`, `compare_a`, and `compare_b` given frequency and duty cycles.
/// # Arguments:
/// * `f_pwm`: from `sys_clk / (65535 * (256 + 15 / 16) * (config.phase_correct + 1))` to `sys_clk / 2`
/// * `duty_a`: from `0.0` to `1.0`
/// * `duty_b`: from `0.0` to `1.0`
fn set_freq_duty(
    config: &mut embassy_rp::pwm::Config,
    f_pwm: U28F4,
    duty_a: U16F16,
    duty_b: U16F16,
) -> Result<(), SetFreqDutyError> {
    let f_pwm_bits = f_pwm.to_bits();
    let f_sys = embassy_rp::clocks::clk_sys_freq();

    let mut div = (((f_sys as u64) << if config.phase_correct { 7 } else { 8 })
        / (65535 * f_pwm_bits as u64)) as u32;

    if div < 1 << 4 {
        div = 1 << 4;
    }

    if div > 0xFFF {
        // format!("{}Hz is too low of a frequency for a sys_clk of {}Hz. Raise sys_clk, or lower the frequency!", f_pwm, f_sys)
        return Err(SetFreqDutyError::FrequencyTooLow);
    }

    let mut top = ((((f_sys as u64) << if config.phase_correct { 7 } else { 8 })
        / (div as u64 * f_pwm_bits as u64)) as u32)
        .saturating_sub(1);

    if top == 0 {
        // format!("{}Hz is too high of a frequency for a sys_clk of {}Hz. Raise sys_clk, or lower the frequency!", f_pwm, f_sys)
        return Err(SetFreqDutyError::FrequencyTooHigh);
    }

    if top > 65534 {
        top = 65534;
    }

    config.top = top as u16;
    config.divider = fixed::FixedU16::<fixed::types::extra::U4>::from_bits(div as u16);
    config.compare_a = top_duty_to_comp(config.top, duty_a);
    config.compare_b = top_duty_to_comp(config.top, duty_b);

    Ok(())
}

mod pac {
    pub use embassy_rp::pac::*;
    pub use embassy_rp::Peripherals;
}

#[rtic::app(device = crate::pac, dispatchers = [SWI_IRQ_0, SWI_IRQ_1, SWI_IRQ_2])]
mod app {

    use embassy_rp::{
        clocks::clk_sys_freq,
        gpio::{Level, Output},
        i2c_slave::{self, I2cSlave, ReadStatus},
        peripherals::I2C1,
    };
    use fixed::traits::ToFixed as _;
    use num_traits::Float;
    use rtic_monotonics::rp2040::prelude::*;
    use rtt_target::rprintln;

    use crate::{set_freq_duty, top_duty_to_comp, Irqs, Mono};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        let config = embassy_rp::config::Config::default();
        let p = embassy_rp::init(config);

        unsafe {
            Mono::start(
                rtic_monotonics::rp2040::TIMER::steal(),
                &rtic_monotonics::rp2040::RESETS::steal(),
            );
        }

        // {
        //     use core::mem::MaybeUninit;
        //     const HEAP_SIZE: usize = 4096;
        //     static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        //     unsafe { super::HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
        // }

        rtt_target::rtt_init_print!();

        let dev = {
            let mut config = i2c_slave::Config::default();
            config.general_call = false;
            config.addr = 0x17;
            I2cSlave::new(p.I2C1, p.PIN_3, p.PIN_2, Irqs, config)
        };

        cortex_m::asm::delay(clk_sys_freq());

        heartbeat::spawn(Output::new(p.PIN_25, Level::High)).ok();
        led_dimmer::spawn(p.PWM_SLICE7, p.PIN_14, p.PIN_15).ok();
        on_i2c_event::spawn(Output::new(p.PIN_16, Level::High), dev).ok();

        (Shared {}, Local {})
    }

    #[idle]
    fn idle_function(_: idle_function::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(priority = 1)]
    async fn led_dimmer(
        _: led_dimmer::Context,
        slice: embassy_rp::peripherals::PWM_SLICE7,
        pina: embassy_rp::peripherals::PIN_14,
        pinb: embassy_rp::peripherals::PIN_15,
    ) {
        let mut config = embassy_rp::pwm::Config::default();
        config.phase_correct = true;
        set_freq_duty(
            &mut config,
            600.to_fixed(),
            0.75.to_fixed(),
            0.25.to_fixed(),
        )
        .unwrap();
        let mut led = embassy_rp::pwm::Pwm::new_output_ab(slice, pina, pinb, config.clone());

        use core::f32::consts::TAU;

        loop {
            let x = (Mono::now().ticks() % 1_000_000) as f32 / 1_000_000 as f32 * TAU;
            config.compare_a = top_duty_to_comp(config.top, ((x.sin() + 1.) / 2.).to_fixed());
            config.compare_b = top_duty_to_comp(config.top, ((x.cos() + 1.) / 2.).to_fixed());
            led.set_config(&config);
            Mono::delay((1_000_000 / 30).micros()).await;
        }
    }

    #[task(priority = 2)]
    async fn on_i2c_event(
        _: on_i2c_event::Context,
        mut led: Output<'static>,
        mut i2c: i2c_slave::I2cSlave<'static, I2C1>,
    ) {
        let mut buf = [0u8; 128];

        loop {
            match i2c.listen(&mut buf).await {
                Ok(i2c_slave::Command::GeneralCall(_)) => {}
                Ok(i2c_slave::Command::Read) => loop {
                    match i2c.respond_to_read(&[led.get_output_level() as u8]).await {
                        Ok(ReadStatus::NeedMoreBytes) => {
                            rprintln!("wanted more bytes")
                        }
                        Ok(_) => break,
                        Err(_) => {
                            rprintln!("error on read")
                        }
                    }
                },
                Ok(i2c_slave::Command::Write(len)) => {
                    if len > 0 {
                        led.set_level((buf[len - 1] != 0).into())
                    }
                }
                Ok(i2c_slave::Command::WriteRead(_)) => panic!("Didn't expect read+write!"),
                Err(_) => {}
            }
        }
    }

    #[task(priority = 1)]
    async fn heartbeat(_: heartbeat::Context, mut led: Output<'static>) {
        for i in 1.. {
            led.toggle();
            rprintln!("{}", i);
            Mono::delay(1.secs()).await;
        }
    }
}
