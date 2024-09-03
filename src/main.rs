#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use alloc::format;
use core::mem::MaybeUninit;
use embassy_executor::{Executor, InterruptExecutor};
use embassy_rp::i2c_slave::ReadStatus;
use embassy_rp::interrupt;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    i2c, i2c_slave,
    interrupt::{InterruptExt as _, Priority},
    peripherals::{I2C1, UART0},
    uart,
};
use embassy_time::{Duration, Instant, Ticker, Timer, TICK_HZ};

use fixed::traits::ToFixed;
use fixed::types::{U16F16, U28F4};
use panic_probe as _;
use rtt_target::{rprint, rprintln};

// #[no_mangle]
// static RTIC_ASYNC_MAX_LOGICAL_PRIO: u8 = 3; // 1 << embassy_rp::NVIC_PRIO_BITS;

// use rtic_monotonics::rp2040::prelude::*;

// rp2040_timer_monotonic!(Mono);

#[allow(unused_imports)]
use num_traits::*;
// use num_traits::float::FloatCore;

use embedded_alloc::Heap;
extern crate alloc;

#[global_allocator]
static HEAP: Heap = Heap::empty();

bind_interrupts!(struct Irqs {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
    UART0_IRQ => uart::InterruptHandler<UART0>;
});

// static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MED: InterruptExecutor = InterruptExecutor::new();
static mut EXECUTOR_LOW: MaybeUninit<Executor> = MaybeUninit::uninit();

// static CHANNEL: Channel<CriticalSectionRawMutex, Box<str>, 64> = Channel::new();

// #[cortex_m_rt::interrupt]
// unsafe fn SWI_IRQ_0() {
//     EXECUTOR_HIGH.on_interrupt()
// }

#[cortex_m_rt::interrupt]
unsafe fn SWI_IRQ_0() {
    EXECUTOR_MED.on_interrupt()
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());

    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 4096;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    // unsafe {
    //     Mono::start(
    //         rtic_monotonics::rp2040::TIMER::steal(),
    //         &rtic_monotonics::rp2040::RESETS::steal(),
    //     );
    // }

    let i2c = {
        let mut config = embassy_rp::i2c_slave::Config::default();
        config.addr = 0x17;
        config.general_call = false;
        embassy_rp::i2c_slave::I2cSlave::new(p.I2C1, p.PIN_3, p.PIN_2, Irqs, config)
    };

    // let uart = uart::Uart::new(
    //     p.UART0,
    //     p.PIN_0,
    //     p.PIN_1,
    //     Irqs,
    //     p.DMA_CH1,
    //     p.DMA_CH0,
    //     Default::default(),
    // );

    rtt_target::rtt_init_print!();

    // let (tx, rx) = uart.split();

    // interrupt::SWI_IRQ_0.set_priority(Priority::P1);
    // let high_spawner = EXECUTOR_HIGH.start(interrupt::SWI_IRQ_0);
    // spawner.spawn(run_high()).unwrap();
    // interrupt::I2C0_IRQ.set_priority(Priority::P1);
    // interrupt::TIMER_IRQ_0.set_priority(Priority::P2);
    // interrupt::TIMER_IRQ_1.set_priority(Priority::P2);
    // interrupt::TIMER_IRQ_2.set_priority(Priority::P2);
    // interrupt::TIMER_IRQ_3.set_priority(Priority::P2);

    interrupt::SWI_IRQ_0.set_priority(Priority::P3);
    let med_spawner = EXECUTOR_MED.start(interrupt::SWI_IRQ_0);
    med_spawner
        .spawn(on_i2c_event(Output::new(p.PIN_16, Level::High), i2c))
        .unwrap();

    let low_executor = unsafe { EXECUTOR_LOW.write(Executor::new()) };
    low_executor.run(|spawner| {
        spawner
            .spawn(heartbeat(
                p.PIN_25,
                // CHANNEL.sender(),
            ))
            .unwrap();
        // spawner.spawn(tx_handler(tx, CHANNEL.receiver())).unwrap()
        spawner
            .spawn(led_dimmer(p.PWM_SLICE7, p.PIN_14, p.PIN_15))
            .unwrap()
    });
}

// use num_traits::Float;

#[embassy_executor::task]
async fn heartbeat(
    pin: impl embassy_rp::gpio::Pin,
    // sender: Sender<'static, CriticalSectionRawMutex, Box<str>, 64>,
) {
    let mut led = Output::new(pin, Level::Low);

    let mut ticker = Ticker::every(Duration::from_secs(1));
    for i in 1.. {
        led.toggle();
        // let mut config = embassy_rp::pwm::Config::default();
        // config.phase_correct = true;

        // let duration = {
        //     // let start = Mono::now();
        //     let start = Instant::now();
        //     set_freq_duty(
        //         &mut config,
        //         3.75.to_fixed(),
        //         0.75.to_fixed(),
        //         0.25.to_fixed(),
        //     )
        //     .unwrap();
        //     // Mono::now() - start
        //     Instant::now() - start
        // };

        // block_for(Duration::from_millis(100));

        // sender.send(format!("Your count is: {}\n", i).into()).await;
        rprint!(
            "{}\n",
            i,
            // config.top,
            // config.divider,
            // config.compare_a,
            // config.compare_b,
            // (embassy_rp::clocks::clk_sys_freq() as f32)
            //     / ((if config.phase_correct { 2f32 } else { 1f32 })
            //         * ((config.top + 1) as f32)
            //         * config.divider.to_num::<f32>()),
            // (config.compare_a as f32) / ((config.top + 1) as f32),
            // (config.compare_b as f32) / ((config.top + 1) as f32),
            // interrupt::I2C1_IRQ.get_priority(),
            // interrupt::TIMER_IRQ_0.get_priority(),
            // duration.as_micros()
        );

        // Mono::delay(1.secs()).await;
        ticker.next().await;
    }
}

const fn top_duty_to_comp(top: u16, duty: U16F16) -> u16 {
    ((((top + 1) as u32) * duty.to_bits()) >> 16) as _
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
) -> Result<(), alloc::string::String> {
    let f_pwm_bits = f_pwm.to_bits();
    let f_sys = embassy_rp::clocks::clk_sys_freq();

    let mut div = (((f_sys as u64) << if config.phase_correct { 7 } else { 8 })
        / (65535 * f_pwm_bits as u64)) as u32;

    if div < 1 << 4 {
        div = 1 << 4;
    }

    if div > 0xFFF {
        return Err(format!("{}Hz is too low of a frequency for a sys_clk of {}Hz. Raise sys_clk, or lower the frequency!", f_pwm, f_sys));
    }

    let mut top = ((((f_sys as u64) << if config.phase_correct { 7 } else { 8 })
        / (div as u64 * f_pwm_bits as u64)) as u32)
        .saturating_sub(1);

    if top == 0 {
        return Err(format!("{}Hz is too high of a frequency for a sys_clk of {}Hz. Raise sys_clk, or lower the frequency!", f_pwm, f_sys));
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

#[embassy_executor::task]
async fn led_dimmer(
    slice: embassy_rp::peripherals::PWM_SLICE7,
    pina: embassy_rp::peripherals::PIN_14,
    pinb: embassy_rp::peripherals::PIN_15,
    // sender: Sender<'static, CriticalSectionRawMutex, Box<str>, 64>,
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

    // Mono::delay(1.secs()).await;
    Timer::after(Duration::from_secs(1)).await;
    let mut led = embassy_rp::pwm::Pwm::new_output_ab(slice, pina, pinb, config.clone());

    use core::f32::consts::TAU;

    let mut ticker = Ticker::every(Duration::from_hz(30));
    loop {
        // let x = Mono::now().ticks() as f32 / 1_000_000 as f32 * TAU;
        let x = (Instant::now().as_ticks() % TICK_HZ) as f32 / TICK_HZ as f32 * TAU;
        config.compare_a = top_duty_to_comp(config.top, ((x.sin() + 1.) / 2.).to_fixed());
        config.compare_b = top_duty_to_comp(config.top, ((x.cos() + 1.) / 2.).to_fixed());
        led.set_config(&config);
        // Mono::delay((1_000_000 / 30).micros()).await;
        ticker.next().await;
    }
}

// #[embassy_executor::task]
// async fn tx_handler(
//     mut tx: uart::UartTx<'static, UART0, uart::Async>,
//     channel: Receiver<'static, CriticalSectionRawMutex, Box<str>, 64>,
// ) {
//     loop {
//         let val = channel.receive().await;
//         tx.write(val.as_bytes()).await.unwrap();
//     }
// }

#[embassy_executor::task]
async fn on_i2c_event(mut led: Output<'static>, mut i2c: i2c_slave::I2cSlave<'static, I2C1>) {
    let mut buf = [0u8; 128];

    loop {
        match i2c.listen(&mut buf).await {
            Ok(i2c_slave::Command::GeneralCall(_)) => {}
            Ok(i2c_slave::Command::Read) => loop {
                match i2c
                    .respond_and_fill(&[led.get_output_level() as u8], 0)
                    .await
                {
                    Ok(ReadStatus::NeedMoreBytes) => {}
                    Ok(_) => break,
                    Err(_) => {
                        rprint!("error on read")
                    }
                }
            },
            Ok(i2c_slave::Command::Write(len)) => {
                if len > 0 {
                    led.set_level((buf[len - 1] != 0).into())
                }
            }
            Ok(i2c_slave::Command::WriteRead(len)) => {
                loop {
                    rprintln!("*");
                    match i2c
                        .respond_and_fill(&[led.get_output_level() as u8], 0)
                        .await
                    {
                        Ok(ReadStatus::NeedMoreBytes) => {}
                        Ok(_) => break,
                        Err(_) => {
                            rprint!("error on writeread")
                        }
                    }
                }

                if len > 0 {
                    led.set_level((buf[len - 1] != 0).into());
                }
            }
            Err(_) => {}
        }
    }
}
