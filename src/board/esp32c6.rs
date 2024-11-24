#[cfg(feature = "defmt")]
use defmt::trace;
use embassy_executor::Spawner;
use embassy_net::{Runner, Stack, StackResources};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Timer;
use esp_alloc as _;
use esp_hal::{
    clock::CpuClock,
    gpio::GpioPin,
    ledc::{channel, timer, LSGlobalClkSource, Ledc, LowSpeed},
    peripherals::LEDC,
    prelude::*,
    rng::Rng,
};
use esp_wifi::{
    wifi::{
        self, ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent,
        WifiStaDevice, WifiState,
    },
    EspWifiController,
};
use rand::RngCore;
use static_cell::StaticCell;

static LED_STATE: Signal<CriticalSectionRawMutex, bool> = Signal::new();

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

#[derive(Clone, Copy)]
pub struct Board {
    stack: Stack<'static>,
}

impl Board {
    pub async fn init(spawner: &Spawner, ssid: &'static str, password: &'static str) -> Self {
        let peripherals = esp_hal::init({
            let mut config = esp_hal::Config::default();
            config.cpu_clock = CpuClock::max();
            config
        });

        esp_alloc::heap_allocator!(72 * 1024);

        let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);

        let mut rng = Rng::new(peripherals.RNG);

        trace!("1");
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

        let (stack, runner) = embassy_net::new(wifi_interface, config, resources, rng.next_u64());

        spawner.spawn(connection(controller, ssid, password)).ok();
        spawner.spawn(net_task(runner)).ok();

        Board { stack }
    }

    pub async fn set_led(&self, state: bool) {
        LED_STATE.signal(state);
    }

    pub async fn wait_for_network(&self) {
        self.stack.wait_config_up().await;
    }
}
