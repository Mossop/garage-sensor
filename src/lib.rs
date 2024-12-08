#![no_std]

use defmt::{debug, trace};
use embassy_executor::Spawner;

#[cfg_attr(feature = "esp32c6", path = "board/esp32c6.rs")]
mod board;

use board::Board;
use embassy_futures::select::{select, select3, Either};
use embassy_time::Timer;
use mcutie::{
    homeassistant::{
        binary_sensor::{BinarySensor, BinarySensorClass},
        sensor::{Sensor, SensorClass, SensorStateClass},
        AvailabilityState, AvailabilityTopics, Device, Entity, Origin,
    },
    McutieBuilder, McutieReceiver, McutieTask, MqttMessage, PublishBytes, Publishable, Topic,
};

use crate::board::{Led, Motion, TempSensor};

const DEVICE_AVAILABILITY_TOPIC: Topic<&'static str> = Topic::Device("status");
const MOTION_STATE_TOPIC: Topic<&'static str> = Topic::Device("motion/state");
const TEMP_STATE_TOPIC: Topic<&'static str> = Topic::Device("temp/state");
const HUMID_STATE_TOPIC: Topic<&'static str> = Topic::Device("humidity/state");

const DEVICE: Device<'static> = Device::new();
const ORIGIN: Origin<'static> = Origin::new();

const MOTION_SENSOR: Entity<'static, 1, BinarySensor> = Entity {
    device: DEVICE,
    origin: ORIGIN,
    object_id: "motion",
    unique_id: Some("motion"),
    name: "Motion",
    availability: AvailabilityTopics::All([DEVICE_AVAILABILITY_TOPIC]),
    state_topic: MOTION_STATE_TOPIC,
    component: BinarySensor {
        device_class: Some(BinarySensorClass::Motion),
    },
};

const TEMPERATURE_SENSOR: Entity<'static, 1, Sensor> = Entity {
    device: DEVICE,
    origin: ORIGIN,
    object_id: "temp",
    unique_id: Some("temp"),
    name: "Temperature",
    availability: AvailabilityTopics::All([DEVICE_AVAILABILITY_TOPIC]),
    state_topic: TEMP_STATE_TOPIC,
    component: Sensor {
        device_class: Some(SensorClass::Temperature),
        state_class: Some(SensorStateClass::Measurement),
        unit_of_measurement: Some("°C"),
    },
};

const HUMIDITY_SENSOR: Entity<'static, 1, Sensor> = Entity {
    device: DEVICE,
    origin: ORIGIN,
    object_id: "humidity",
    unique_id: Some("humidity"),
    name: "Relative Humidity",
    availability: AvailabilityTopics::All([DEVICE_AVAILABILITY_TOPIC]),
    state_topic: HUMID_STATE_TOPIC,
    component: Sensor {
        device_class: Some(SensorClass::Humidity),
        state_class: Some(SensorStateClass::Measurement),
        unit_of_measurement: Some("%"),
    },
};

#[embassy_executor::task]
async fn mqtt_task(
    runner: McutieTask<
        'static,
        &'static str,
        PublishBytes<'static, &'static str, AvailabilityState>,
        0,
    >,
) {
    runner.run().await;
}

async fn motion_sensor(mut motion: Motion) {
    let _ = MOTION_SENSOR.publish_state(motion.state()).await;

    loop {
        if let Either::First(_) = select(motion.wait(), Timer::after_secs(60)).await {
            debug!("Motion changed state");
        }

        let _ = MOTION_SENSOR.publish_state(motion.state()).await;
    }
}

async fn temp_sensor(mut sensor: TempSensor) {
    loop {
        if let Some((temperature, humidity)) = sensor.read().await {
            trace!(
                "Read data, temperature={}°C, humidity={}%",
                temperature,
                humidity
            );
            let _ = TEMPERATURE_SENSOR.publish_state(temperature).await;
            let _ = HUMIDITY_SENSOR.publish_state(humidity).await;
        }

        Timer::after_secs(30).await;
    }
}

async fn discovery(receiver: McutieReceiver, led: Led) {
    let mut next_poll = 60 * 5;

    loop {
        match select(receiver.receive(), Timer::after_secs(next_poll)).await {
            Either::First(MqttMessage::Connected)
            | Either::First(MqttMessage::HomeAssistantOnline) => {
                led.set(true).await;

                let _ = MOTION_SENSOR.publish_discovery().await;
                let _ = TEMPERATURE_SENSOR.publish_discovery().await;
                let _ = HUMIDITY_SENSOR.publish_discovery().await;

                let _ = DEVICE_AVAILABILITY_TOPIC
                    .with_bytes(AvailabilityState::Online)
                    .publish()
                    .await;

                // Update availability quickly to ensure home assistant gets it.
                next_poll = 10;
            }
            Either::First(MqttMessage::Disconnected) => {
                led.set(false).await;
            }
            Either::Second(_) => {
                let _ = DEVICE_AVAILABILITY_TOPIC
                    .with_bytes(AvailabilityState::Online)
                    .publish()
                    .await;

                next_poll = 60 * 5;
            }
            _ => {}
        }
    }
}

pub async fn main(spawner: Spawner) {
    let board = Board::init(
        &spawner,
        env!("GARAGE_SSID"),
        option_env!("GARAGE_PASSWORD"),
    )
    .await;

    board.led.set(false).await;

    let (receiver, mqtt_runner) =
        McutieBuilder::new(board.network, "garage", env!("GARAGE_BROKER"))
            .with_last_will(DEVICE_AVAILABILITY_TOPIC.with_bytes(AvailabilityState::Offline))
            .build();

    spawner.spawn(mqtt_task(mqtt_runner)).unwrap();

    select3(
        discovery(receiver, board.led),
        temp_sensor(board.temp),
        motion_sensor(board.motion),
    )
    .await;
}
