#![no_std]

use embassy_executor::Spawner;
use embassy_time::Timer;

#[cfg_attr(feature = "esp32c6", path = "board/esp32c6.rs")]
mod board;

use board::Board;

pub async fn main(spawner: Spawner) {
    let board = Board::init(&spawner, env!("GARAGE_SSID"), env!("GARAGE_PASSWORD")).await;
    board.wait_for_network().await;

    loop {
        board.set_led(true).await;
        Timer::after_millis(1000).await;
        board.set_led(false).await;
        Timer::after_millis(100).await;
    }
}
