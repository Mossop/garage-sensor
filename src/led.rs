use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use esp_hal::{
    gpio::GpioPin,
    ledc::{
        channel::{
            config::{Config, PinConfig},
            Number,
        },
        timer::{self, TimerIFace},
        LSGlobalClkSource, Ledc, LowSpeed,
    },
    peripherals::LEDC,
    prelude::*,
};

static LED_CHANNEL: Channel<CriticalSectionRawMutex, bool, 2> = Channel::new();

#[embassy_executor::task]
async fn led_task(ledc_peripheral: LEDC, gpio: GpioPin<15>) {
    let mut ledc = Ledc::new(ledc_peripheral);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut timer = ledc.timer::<LowSpeed>(timer::Number::Timer0);

    timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty5Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 24.kHz(),
        })
        .unwrap();

    let mut channel = ledc.channel(Number::Channel0, gpio);
    channel
        .configure(Config {
            timer: &timer,
            duty_pct: 10,
            pin_config: PinConfig::PushPull,
        })
        .unwrap();

    loop {
        let state = LED_CHANNEL.receive().await;

        let _ = channel.set_duty(if state { 0 } else { 100 });
    }
}

pub fn spawn_led(spawner: &Spawner, ledc_peripheral: LEDC, gpio: GpioPin<15>) {
    let _ = spawner.spawn(led_task(ledc_peripheral, gpio));
}

pub async fn on() {
    LED_CHANNEL.send(true).await;
}

pub async fn off() {
    LED_CHANNEL.send(false).await;
}
