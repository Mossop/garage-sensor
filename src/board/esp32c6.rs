#[cfg(feature = "defmt")]
use defmt::trace;
use defmt::{error, warn};
use embassy_executor::Spawner;
use embassy_net::{Runner, Stack, StackResources};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Timer;
use esp_alloc as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{AnyPin, GpioPin, Input, Pull},
    i2c::master::{Config, I2c},
    ledc::{channel, timer, LSGlobalClkSource, Ledc, LowSpeed},
    peripherals::LEDC,
    prelude::*,
    rng::Rng,
    Async,
};
use esp_wifi::{
    wifi::{
        self, ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent,
        WifiStaDevice, WifiState,
    },
    EspWifiController,
};
use mcutie::homeassistant::binary_sensor::BinarySensorState;
use rand::RngCore;
use static_cell::StaticCell;

const HTU31D_ADDRESS: u8 = 0x40;
const HTU32D_CMD_HEATER_OFF: u8 = 0x02;
const HTU32D_CMD_RESET: u8 = 0x1E;
const HTU32D_CMD_START_CONVERSION: u8 = 0x5e;
const HTU32D_CMD_READ: u8 = 0x00;
const HTU32D_CMD_READ_DIAGNOSTIC: u8 = 0x08;

static LED_STATE: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static MOTION_STATE: Signal<CriticalSectionRawMutex, bool> = Signal::new();

fn crc(value: u16) -> u8 {
    let mut polynom: u32 = 0x988000; // x^8 + x^5 + x^4 + 1
    let mut msb: u32 = 0x800000;
    let mut mask: u32 = 0xFF8000;
    let mut result: u32 = (value as u32) << 8; // Pad with zeros as specified in spec

    while msb != 0x80 {
        // Check if msb of current value is 1 and apply XOR mask
        if (result & msb) != 0 {
            result = ((result ^ polynom) & mask) | (result & !mask);
        }

        // Shift by one
        msb >>= 1;
        mask >>= 1;
        polynom >>= 1;
    }

    (result & 0xff) as u8
}

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

    let mut channel = ledc.channel(channel::Number::Channel0, gpio);
    channel
        .configure(channel::config::Config {
            timer: &timer,
            duty_pct: 10,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    loop {
        let state = LED_STATE.wait().await;

        let _ = channel.set_duty(if state { 0 } else { 100 });
    }
}

#[embassy_executor::task]
async fn connection(
    mut controller: WifiController<'static>,
    ssid: &'static str,
    password: &'static str,
) {
    loop {
        if wifi::wifi_state() == WifiState::StaConnected {
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            Timer::after_millis(500).await;
        }

        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: ssid.try_into().unwrap(),
                password: password.try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            controller.start_async().await.unwrap();
        }

        match controller.connect_async().await {
            Ok(_) => trace!("Wifi connected!"),
            Err(e) => {
                trace!("Failed to connect to wifi: {:?}", e);
                Timer::after_millis(500).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static, WifiStaDevice>>) {
    runner.run().await
}

#[embassy_executor::task]
async fn motion_task(mut motion: Input<'static, AnyPin>) {
    loop {
        motion.wait_for_any_edge().await;

        MOTION_STATE.signal(motion.is_high());
    }
}

pub struct TempSensor {
    i2c: I2c<'static, Async>,
}

impl TempSensor {
    async fn new(mut i2c: I2c<'static, Async>) -> Self {
        let _ = i2c.write(HTU31D_ADDRESS, &[HTU32D_CMD_RESET]).await;
        let _ = i2c.write(HTU31D_ADDRESS, &[HTU32D_CMD_HEATER_OFF]).await;

        let mut buffer = [0_u8; 2];
        if i2c
            .write_read(HTU31D_ADDRESS, &[HTU32D_CMD_READ_DIAGNOSTIC], &mut buffer)
            .await
            .is_ok()
        {
            let calculated_crc = crc(buffer[0] as u16);
            if buffer[1] != calculated_crc {
                warn!(
                    "Invalid CRC for diagnostics data: {} != {}",
                    calculated_crc, buffer[1]
                );
            } else if buffer[0] != 0 {
                warn!("Diagnostics reports a fault: {}", buffer[0]);
            }
        }

        Self { i2c }
    }

    pub async fn read(&mut self) -> Option<(f32, f32)> {
        if let Err(e) = self
            .i2c
            .write(HTU31D_ADDRESS, &[HTU32D_CMD_START_CONVERSION])
            .await
        {
            error!("Failed sending conversion start command: {}", e);
            return None;
        }

        Timer::after_millis(50).await;

        let mut buffer = [0_u8; 6];
        if let Err(e) = self
            .i2c
            .write_read(HTU31D_ADDRESS, &[HTU32D_CMD_READ], &mut buffer)
            .await
        {
            error!("Failed reading temperature and humidity: {}", e);
            return None;
        }

        let temp = u16::from_be_bytes([buffer[0], buffer[1]]);
        let temp_crc = buffer[2];
        let humid = u16::from_be_bytes([buffer[3], buffer[4]]);
        let humid_crc = buffer[5];

        let humidity = (100.0 * humid as f32) / 65535.0;
        let temperature = ((165.0 * temp as f32) / 65535.0) - 40.0;
        trace!("Read temperature={}, humidity={}", temperature, humidity);

        if crc(temp) != temp_crc || crc(humid) != humid_crc {
            warn!(
                "Invalid CRC for temperature or humidity: {} != {} || {} != {}",
                crc(temp),
                temp_crc,
                crc(humid),
                humid_crc
            );
            return None;
        }

        Some((temperature, humidity))
    }
}

pub struct Led;

impl Led {
    pub async fn set(&self, state: bool) {
        LED_STATE.signal(state);
    }
}

pub struct Motion {
    state: bool,
}

impl Motion {
    pub fn state(&self) -> BinarySensorState {
        self.state.into()
    }

    pub async fn wait(&mut self) -> BinarySensorState {
        let current_state = self.state;
        loop {
            let mut new_state = MOTION_STATE.wait().await;
            Timer::after_millis(50).await;

            if let Some(st) = MOTION_STATE.try_take() {
                new_state = st;
            }

            if new_state != current_state {
                self.state = new_state;
                return new_state.into();
            }
        }
    }
}

pub struct Board {
    pub network: Stack<'static>,
    pub led: Led,
    pub temp: TempSensor,
    pub motion: Motion,
}

impl Board {
    pub async fn init(spawner: &Spawner, ssid: &'static str, password: &'static str) -> Self {
        let peripherals = esp_hal::init({
            let mut config = esp_hal::Config::default();
            config.cpu_clock = CpuClock::max();
            config
        });

        esp_alloc::heap_allocator!(96 * 1024);

        let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);

        let mut rng = Rng::new(peripherals.RNG);

        static CONTROLLER: StaticCell<EspWifiController<'static>> = StaticCell::new();
        let esp_controller = CONTROLLER
            .uninit()
            .write(esp_wifi::init(timg0.timer0, rng, peripherals.RADIO_CLK).unwrap());

        let (wifi_interface, controller) =
            wifi::new_with_mode(esp_controller, peripherals.WIFI, WifiStaDevice).unwrap();

        let timg1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1);
        esp_hal_embassy::init(timg1.timer0);

        let _ = spawner.spawn(led_task(peripherals.LEDC, peripherals.GPIO15));

        let config = embassy_net::Config::dhcpv4(Default::default());

        static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
        let resources = RESOURCES.init(StackResources::<3>::new());

        let (network, runner) = embassy_net::new(wifi_interface, config, resources, rng.next_u64());

        spawner.spawn(connection(controller, ssid, password)).ok();
        spawner.spawn(net_task(runner)).ok();

        let i2c = I2c::new(
            peripherals.I2C0,
            Config {
                frequency: 70.kHz(),
                timeout: Some(10),
            },
        )
        .with_sda(peripherals.GPIO22)
        .with_scl(peripherals.GPIO23)
        .into_async();

        let temp = TempSensor::new(i2c).await;

        let motion_input = Input::new(peripherals.GPIO17, Pull::Down);

        let motion = Motion {
            state: motion_input.is_high(),
        };

        spawner.spawn(motion_task(motion_input)).ok();

        Board {
            led: Led,
            network,
            temp,
            motion,
        }
    }
}
