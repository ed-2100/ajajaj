#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use core::mem::MaybeUninit;
use embassy_executor::{Executor, InterruptExecutor};
use embassy_rp::interrupt;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    i2c, i2c_slave,
    interrupt::{InterruptExt as _, Priority},
    peripherals::{I2C1, UART0},
    uart,
};
use embassy_time::{Duration, Instant, Ticker, Timer};
use fixed::traits::ToFixed;
use fixed::types::{U0F16, U16F16, U1F15, U28F4};
use panic_probe as _;
use rtt_target::rprint;

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

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MED: InterruptExecutor = InterruptExecutor::new();
static mut EXECUTOR_LOW: MaybeUninit<Executor> = MaybeUninit::uninit();

// static CHANNEL: Channel<CriticalSectionRawMutex, Box<str>, 64> = Channel::new();

#[cortex_m_rt::interrupt]
unsafe fn SWI_IRQ_0() {
    EXECUTOR_HIGH.on_interrupt()
}

#[cortex_m_rt::interrupt]
unsafe fn SWI_IRQ_1() {
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

    let i2c = {
        let mut config = embassy_rp::i2c_slave::Config::default();
        config.addr = 0x17;
        config.general_call = false;
        embassy_rp::i2c_slave::I2cSlave::new(p.I2C1, p.PIN_3, p.PIN_2, Irqs, config)
    };

    let uart = uart::Uart::new(
        p.UART0,
        p.PIN_0,
        p.PIN_1,
        Irqs,
        p.DMA_CH1,
        p.DMA_CH0,
        Default::default(),
    );

    rtt_target::rtt_init_print!();

    let (tx, rx) = uart.split();

    interrupt::SWI_IRQ_0.set_priority(Priority::P2);
    let high_spawner = EXECUTOR_HIGH.start(interrupt::SWI_IRQ_0);
    // spawner.spawn(run_high()).unwrap();

    interrupt::SWI_IRQ_1.set_priority(Priority::P3);
    let med_spawner = EXECUTOR_MED.start(interrupt::SWI_IRQ_1);
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
    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut led = Output::new(pin, Level::Low);

    for i in 1.. {
        ticker.next().await;

        led.toggle();
        // sender.send(format!("Your count is: {}\n", i).into()).await;
        let mut config = embassy_rp::pwm::Config::default();
        config.phase_correct = true;

        let start = Instant::now();
        config = set_freq_duty(config, 3000.to_fixed(), 0.75.to_fixed(), 0.25.to_fixed());
        let duration = Instant::now() - start;
        rprint!(
            "{}, {}, {}, {}, {}, {}, {}, {}\n",
            i,
            config.top,
            config.divider,
            config.compare_a,
            (embassy_rp::clocks::clk_sys_freq() as f32)
                / ((if config.phase_correct { 2f32 } else { 1f32 })
                    * ((config.top + 1) as f32)
                    * config.divider.to_num::<f32>()),
            (config.compare_a as f32) / ((config.top + 1) as f32),
            (config.compare_b as f32) / ((config.top + 1) as f32),
            duration.as_micros()
        );
    }
}

fn set_freq_duty(
    mut config: embassy_rp::pwm::Config,
    f_pwm: U28F4,
    duty_a: U1F15,
    duty_b: U1F15,
) -> embassy_rp::pwm::Config {
    let f_pwm = f_pwm.to_bits();
    let f_sys = embassy_rp::clocks::clk_sys_freq();

    let div = (f_sys as u64) << if config.phase_correct { 7 } else { 8 };
    let mut div = (div / (65535 * f_pwm) as u64) as u32;
    if div < 1 << 4 {
        div = 1 << 4;
    }

    assert!(
        div <= 0xFFF,
        "Too low of a frequency! Lower sys_clk, or raise the frequency! {:?}",
        div
    );

    let top = (f_sys as u64) << if config.phase_correct { 7 } else { 8 };
    let mut top = (top / (div * f_pwm) as u64) as u32 - 1;
    if top > 65534 {
        top = 65534;
    }

    config.top = top as _;
    config.divider = fixed::FixedU16::<fixed::types::extra::U4>::from_bits(div as _);
    config.compare_a = ((((config.top + 1) as u32) * (duty_a.to_bits() as u32)) >> 15) as _;
    config.compare_b = ((((config.top + 1) as u32) * (duty_b.to_bits() as u32)) >> 15) as _;
    config
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
    config = set_freq_duty(config, 3.75.to_fixed(), 0.75.to_fixed(), 0.25.to_fixed());

    Timer::after_secs(2).await;
    let _led = embassy_rp::pwm::Pwm::new_output_ab(slice, pina, pinb, config);

    loop {
        Timer::after_secs(10).await;
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
    loop {
        let mut buf = [0u8; 128];

        match i2c.listen(&mut buf).await {
            Ok(i2c_slave::Command::GeneralCall(_)) => {}
            Ok(i2c_slave::Command::Read) => loop {
                match i2c.respond_to_read(&[led.get_output_level() as u8]).await {
                    Ok(x) => match x {
                        i2c_slave::ReadStatus::Done => break,
                        i2c_slave::ReadStatus::NeedMoreBytes => {}
                        i2c_slave::ReadStatus::LeftoverBytes(_) => break,
                    },
                    Err(_) => {}
                }
            },
            Ok(i2c_slave::Command::Write(len)) => led.set_level((buf[len - 1] != 0).into()),
            Ok(i2c_slave::Command::WriteRead(len)) => loop {
                match i2c
                    .respond_and_fill(&[led.get_output_level() as u8], 0)
                    .await
                {
                    Ok(x) => {
                        led.set_level((buf[len - 1] != 0).into());

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
