#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    async_attribute_server::AttributeServer,
    asynch::Ble,
    attribute_server::NotificationData,
    gatt, Addr,
};
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{Channel, Sender},
};
use embassy_time::{Duration, Timer};
use embedded_hal_async::digital::Wait;
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::{ble::controller::asynch::BleConnector, initialize, EspWifiInitFor};
use hal::{
    clock::ClockControl,
    embassy,
    gpio::{GpioPin, Input, PullDown},
    macros::main,
    peripherals::*,
    prelude::*,
    timer::TimerGroup,
    Rng, IO,
};
use static_cell::make_static;

const REPORT_MAP: [u8; 65] = [
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x06, // USAGE (Keyboard)
    0xa1, 0x01, // COLLECTION (Application)
    0x85, 0x01, //   REPORT_ID (1)
    0x05, 0x07, //   USAGE_PAGE (Keyboard)
    0x19, 0x01, //   USAGE_MINIMUM
    0x29, 0x7f, //   USAGE_MAXIMUM
    0x15, 0x00, //   LOGICAL_MINIMUM (0)
    0x25, 0x01, //   LOGICAL_MAXIMUM (1)
    0x75, 0x01, //   REPORT_SIZE (1)
    0x95, 0x08, //   REPORT_COUNT (8)
    0x81, 0x02, //   INPUT (Data,Var,Abs)
    0x95, 0x01, //   REPORT_COUNT (1)
    0x75, 0x08, //   REPORT_SIZE (8)
    0x81, 0x01, //   INPUT (Cnst,Ary,Abs)
    0x95, 0x05, //   REPORT_COUNT (5)
    0x75, 0x01, //   REPORT_SIZE (1)
    0x05, 0x08, //   USAGE_PAGE (LEDs)
    0x19, 0x01, //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05, //   USAGE_MAXIMUM (Kana)
    0x91, 0x02, //   OUTPUT (Data,Var,Abs)
    0x95, 0x01, //   REPORT_COUNT (1)
    0x75, 0x03, //   REPORT_SIZE (3)
    0x91, 0x01, //   OUTPUT (Cnst,Ary,Abs)
    0x95, 0x06, //   REPORT_COUNT (6)
    0x75, 0x08, //   REPORT_SIZE (8)
    0x15, 0x00, //   LOGICAL_MINIMUM (0)
    0x25, 0x65, //   LOGICAL_MAXIMUM (101)
    0x05, 0x07, //   USAGE_PAGE (Keyboard)
    0x19, 0x00, //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65, //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00, //   INPUT (Data,Ary,Abs)
    0xc0, // END_COLLECTION
];

struct KeyboardReport {
    report_id: u8,
    modifiers: u8,
    reserved: u8,
    key_codes: [u8; 6],
}

impl KeyboardReport {
    fn to_bytes(&self) -> [u8; 9] {
        [
            self.report_id,
            self.modifiers,
            self.reserved,
            self.key_codes[0],
            self.key_codes[1],
            self.key_codes[2],
            self.key_codes[3],
            self.key_codes[4],
            self.key_codes[5],
        ]
    }
}

#[main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let peripherals = Peripherals::take();

    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();

    let rng = Rng::new(peripherals.RNG);
    let timer = hal::systimer::SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let init = initialize(
        EspWifiInitFor::Ble,
        timer,
        rng.clone(),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let mut rng_wrap = RngWrapper { rng: rng };

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let button = io.pins.gpio9.into_pull_down_input();

    // Async requires the GPIO interrupt to wake futures
    hal::interrupt::enable(
        hal::peripherals::Interrupt::GPIO,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0);

    let channel: Channel<NoopRawMutex, u8, 3> = Channel::new();
    let channel = make_static!(channel);

    let receiver = channel.receiver();
    let sender = channel.sender();

    spawner.spawn(key_reader(button, sender)).ok();

    let mut bluetooth = peripherals.BT;

    let connector = BleConnector::new(&init, &mut bluetooth);
    let mut ble = Ble::new(connector, esp_wifi::current_millis);
    println!("Connector created");

    let mut ltk = None;

    loop {
        println!("{:?}", ble.init().await);
        println!("{:?}", ble.cmd_set_le_advertising_parameters().await);
        println!(
            "{:?}",
            ble.cmd_set_le_advertising_data(
                create_advertising_data(&[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::Unknown {
                        ty: 0x03,
                        data: &[0x12, 0x18]
                    }, // HID
                    AdStructure::CompleteLocalName("ESP32-C3"),
                    AdStructure::Unknown {
                        ty: 0x19,
                        data: &[0xc1, 0x03]
                    }, // Appearance (0x03C1 = Keyboard)
                ])
                .unwrap()
            )
            .await
        );
        println!("{:?}", ble.cmd_set_le_advertise_enable(true).await);
        let local_addr = Addr::from_le_bytes(false, ble.cmd_read_br_addr().await.unwrap());

        println!("started advertising");

        let mut read_hid_information = |_offset: usize, data: &mut [u8]| {
            data[..4].copy_from_slice(&[0x01, 0x11, 0x00, 0x02]);
            4
        };

        let mut read_hid_report_map = |offset: usize, data: &mut [u8]| {
            println!("read hid report map {offset} {}", data.len());

            let val = REPORT_MAP;
            let off = offset;
            if off < val.len() {
                let len = data.len().min(val.len() - off);
                data[..len].copy_from_slice(&val[off..off + len]);
                len
            } else {
                0
            }
        };

        let mut write_hid_control_point = |offset: usize, data: &[u8]| {
            println!("write hid control point: {} {:?}", offset, data);
        };

        let mut read_hid_report = |_offset: usize, data: &mut [u8]| {
            println!("read hid report");
            let resp = KeyboardReport {
                report_id: 0,
                modifiers: 0,
                reserved: 0,
                key_codes: [0u8; 6],
            };

            data[..9].copy_from_slice(&resp.to_bytes());
            9
        };

        let mut read_protocol_mode = |_offset: usize, data: &mut [u8]| {
            data[..2].copy_from_slice(&[0x00, 0x00]);
            2
        };

        let mut write_protocol_mode = |offset: usize, data: &[u8]| {
            println!("write_protocol_mode: Offset {}, data {:?}", offset, data);
        };

        let mut read_device_info = |_offset: usize, data: &mut [u8]| {
            data[..7].copy_from_slice(&[0x02, 0x8a, 0x24, 0x66, 0x82, 0x34, 0x36]);
            7
        };

        let mut read_battery_level = |_offset: usize, data: &mut [u8]| {
            data[..1].copy_from_slice(&[100]);
            1
        };

        gatt!([
            // BLE HID Service
            service {
                uuid: "00001812-0000-1000-8000-00805f9b34fb",
                characteristics: [
                    // BLE HID_information
                    characteristic {
                        uuid: "00002a4a-0000-1000-8000-00805f9b34fb",
                        read: read_hid_information,
                    },
                    // BLE HID Report Map characteristic
                    characteristic {
                        uuid: "00002a4b-0000-1000-8000-00805f9b34fb",
                        read: read_hid_report_map,
                    },
                    // BLE HID control point characteristic
                    characteristic {
                        uuid: "00002a4c-0000-1000-8000-00805f9b34fb",
                        write: write_hid_control_point,
                    },
                    // BLE HID Report characteristic
                    characteristic {
                        uuid: "00002a4d-0000-1000-8000-00805f9b34fb",
                        name: "hid_report",
                        notify: true,
                        read: read_hid_report,
                    },
                    // BLE HID protocol mode characteristic
                    characteristic {
                        uuid: "00002a4e-0000-1000-8000-00805f9b34fb",
                        write: write_protocol_mode,
                        read: read_protocol_mode,
                    },
                ],
            },
            // BLE device information
            service {
                uuid: "0000180a-0000-1000-8000-00805f9b34fb",
                characteristics: [
                    // BLE Device Information characteristic
                    characteristic {
                        uuid: "00002a50-0000-1000-8000-00805f9b34fb",
                        read: read_device_info,
                    },
                ],
            },
            // BLE HID Battery Service
            service {
                uuid: "0000180f-0000-1000-8000-00805f9b34fb",
                // BLE HID battery level characteristic
                characteristics: [characteristic {
                    uuid: "00002a19-0000-1000-8000-00805f9b34fb",
                    read: read_battery_level,
                },],
            },
        ]);

        let mut srv = AttributeServer::new_with_ltk(
            &mut ble,
            &mut gatt_attributes,
            local_addr,
            ltk,
            &mut rng_wrap,
        );

        let mut notifier = || async {
            let received = receiver.receive().await;

            println!("notify hid report");
            let resp = KeyboardReport {
                report_id: 1,
                modifiers: 0,
                reserved: 0,
                key_codes: [received, 0, 0, 0, 0, 0],
            };

            NotificationData::new(hid_report_handle, &resp.to_bytes())
        };

        srv.run(&mut notifier).await.unwrap();

        // TODO persist the LTK
        ltk = srv.get_ltk();
    }
}

#[embassy_executor::task]
async fn key_reader(
    mut button: GpioPin<Input<PullDown>, 9>,
    sender: Sender<'static, NoopRawMutex, u8, 3>,
) {
    loop {
        button.wait_for_rising_edge().await.ok();

        sender.send(0x08).await; // 'e'
        Timer::after(Duration::from_millis(200)).await;
        sender.send(0x0).await;

        sender.send(0x16).await; // 's'
        Timer::after(Duration::from_millis(200)).await;
        sender.send(0x0).await;

        sender.send(0x13).await; // 'p'
        Timer::after(Duration::from_millis(200)).await;
        sender.send(0x0).await;

        sender.send(0x20).await; // '3'
        Timer::after(Duration::from_millis(200)).await;
        sender.send(0x0).await;

        sender.send(0x1f).await; // '2'
        Timer::after(Duration::from_millis(200)).await;
        sender.send(0x0).await;
    }
}

struct RngWrapper {
    rng: Rng,
}

impl rand_core::RngCore for RngWrapper {
    fn next_u32(&mut self) -> u32 {
        self.rng.random()
    }

    fn next_u64(&mut self) -> u64 {
        (self.rng.random() as u64) << 32 | self.rng.random() as u64
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        for b in dest {
            *b = (self.rng.random() & 0xff) as u8;
        }
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        self.fill_bytes(dest);
        Ok(())
    }
}

impl rand_core::CryptoRng for RngWrapper {}
