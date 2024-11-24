#![no_std]
#![no_main]

#[cfg(feature = "defmt")]
use defmt_rtt as _;
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal_embassy::main;

#[main]
async fn main(spawner: Spawner) {
    garage_sensor::main(spawner).await;
}
