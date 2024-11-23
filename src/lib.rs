#![no_std]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_hal::prelude::*;

use crate::led::spawn_led;

mod led;

pub async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);
    info!("Embassy initialized!");

    spawn_led(&spawner, peripherals.LEDC, peripherals.GPIO15);

    loop {
        Timer::after_secs(1).await;
    }
}
