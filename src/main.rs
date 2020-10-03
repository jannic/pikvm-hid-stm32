#![no_std]
#![no_main]

// Mostly taken from
// https://github.com/stm32-rs/stm32-usbd-examples/blob/master/example-stm32f103c8/examples/usb_mouse_rtfm.rs

extern crate panic_semihosting;

#[cfg(feature = "wfi")]
use cortex_m::wfi;

use cortex_m::{asm::delay, peripheral::DWT};
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::stm32::USART1;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use stm32f1xx_hal::{gpio, prelude::*, serial, serial::Event as SerialEvent, serial::Serial};
use usb_device::bus;
use usb_device::prelude::*;
use core::convert::TryInto;

use usbd_hid_device::Hid;

use keycode::{KeyboardState, KeyMap, KeyMapping, KeyMappingId, KeyState};

mod buf;
mod report;

use buf::Buffer;
use report::KeyboardReport;

type LED = gpio::gpioc::PC13<gpio::Output<gpio::PushPull>>;

struct SerialKeyCode(u8);

impl Into<KeyMapping> for SerialKeyCode {
    fn into(self) -> KeyMapping {
        match self.0 {
            1..=46 => KeyMapping::Usb(self.0 as u16 + 3),
            47..=66 => KeyMapping::Usb(self.0 as u16 + 4),
            67..=76 => KeyMapping::Usb(self.0 as u16 + 6),
            77..=84 => KeyMapping::Usb(self.0 as u16 + 147),
            85 => KeyMapping::Id(KeyMappingId::Pause),
            86 => KeyMapping::Id(KeyMappingId::ScrollLock),
            87 => KeyMapping::Id(KeyMappingId::NumLock),
            88 => KeyMapping::Id(KeyMappingId::ContextMenu),
            89..=104 => KeyMapping::Usb(self.0 as u16 - 5),
            105 => KeyMapping::Id(KeyMappingId::Power),
            _ => KeyMapping::Code(None), // TODO: Better handling of unknown codes
        }
    }
}

#[rtic::app(device = stm32f1xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        prev_response: ProtoResponse,
        led: LED,
        serial_buffer: Buffer,
        keyboard_state: KeyboardState,

        usb_dev: UsbDevice<'static, UsbBusType>,
        hid: Hid<'static, KeyboardReport, UsbBusType>,
        tx: serial::Tx<USART1>,
        rx: serial::Rx<USART1>,
    }

    #[init]
    fn init(mut cx: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;

        cx.core.DCB.enable_trace();
        DWT::unlock();
        cx.core.DWT.enable_cycle_counter();

        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();

        let mut gpioc = cx.device.GPIOC.split(&mut rcc.apb2);
        let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(48.mhz())
            .pclk1(24.mhz())
            .freeze(&mut flash.acr);

        assert!(clocks.usbclk_valid());

        let mut gpioa = cx.device.GPIOA.split(&mut rcc.apb2);

        // BluePill board has a pull-up resistor on the D+ line.
        // Pull the D+ pin down to send a RESET condition to the USB bus.
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low().ok();
        delay(clocks.sysclk().0 / 100);

        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        let usb = Peripheral {
            usb: cx.device.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        *USB_BUS = Some(UsbBus::new(usb));

        let hid = Hid::new(USB_BUS.as_ref().unwrap(), 10);

        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0xc410, 0x0000))
            .manufacturer("Jan")
            .product("PIKVM-STM32")
            .serial_number("TEST")
            .device_class(0)
            .build();

        // Set up UART
        let mut afio = cx.device.AFIO.constrain(&mut rcc.apb2);
        let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let rx = gpioa.pa10;
        let mut serial = Serial::usart1(
            cx.device.USART1,
            (tx, rx),
            &mut afio.mapr,
            serial::Config::default().baudrate(115_200.bps()),
            clocks,
            &mut rcc.apb2,
        );
        serial.listen(SerialEvent::Rxne);
        // Split TX and RX
        let (tx, rx) = serial.split();

        let serial_buffer = Buffer::new();

        // Set key rollover to 6 keys. This fixes the hid
        // report length to 8 bytes, as currently expected by
        // report::KeyboardReport.
        let keyboard_state = KeyboardState::new(Some(6));

        init::LateResources {
            prev_response: ProtoResponse::None,
            led,
            serial_buffer,
            keyboard_state,

            usb_dev,
            hid,

            tx,
            rx,
        }
    }

    #[task(spawn = [toggle], resources = [tx, prev_response, hid, keyboard_state])]
    fn handle_cmd(cx: handle_cmd::Context, cmd: [u8; 8]) {
        let tx = cx.resources.tx;
        let prev = cx.resources.prev_response;
        let mut hid = cx.resources.hid;
        let keyboard_state = cx.resources.keyboard_state;
        let crc: u16 = ((cmd[6] as u16) << 8) | (cmd[7] as u16);
        if calculate_crc(&cmd[0..6]) == crc {
            const PROTO_CMD_PING: u8 = 0x01;
            const PROTO_CMD_REPEAT: u8 = 0x02;
            const PROTO_CMD_KEY_EVENT: u8 = 0x11;
            const PROTO_CMD_MOUSE_MOVE_EVENT: u8 = 0x12;
            const PROTO_CMD_MOUSE_BUTTON_EVENT: u8 = 0x13;
            match cmd[1] {
                PROTO_CMD_PING => {
                    *prev = ProtoResponse::Pong;
                    send_cmd_response(tx, ProtoResponse::Pong);
                    cx.spawn.toggle().ok();
                }
                PROTO_CMD_REPEAT => {
                    send_cmd_response(tx, prev.clone());
                }
                PROTO_CMD_KEY_EVENT => {
                    *prev = ProtoResponse::Ok;
                    let key_mapping: KeyMapping = SerialKeyCode(cmd[2]).into();
                    let key: Result<KeyMap, ()> = KeyMap::from_key_mapping(key_mapping);
                    let resp = key.map(|key| {
                        if cmd[3] != 0 {
                            keyboard_state.update_key(key, KeyState::Pressed);
                        } else  {
                            keyboard_state.update_key(key, KeyState::Released);
                        }
                        let report = KeyboardReport::new(keyboard_state.usb_input_report().try_into().expect("report should be 8 bytes long"));
                        hid.lock(|hid| hid.send_report(&report))
                    });

                    if resp.is_ok() {
                        send_cmd_response(tx, ProtoResponse::Ok);
                    } else {
                        send_cmd_response(tx, ProtoResponse::InvalidError);
                    }
                    cx.spawn.toggle().ok();
                }
                PROTO_CMD_MOUSE_MOVE_EVENT | PROTO_CMD_MOUSE_BUTTON_EVENT => {
                    *prev = ProtoResponse::Ok;
                    send_cmd_response(tx, ProtoResponse::InvalidError);
                    cx.spawn.toggle().ok();
                }
                _ => send_cmd_response(tx, ProtoResponse::InvalidError),
            }
        } else {
            send_cmd_response(tx, ProtoResponse::CrcError);
        }
    }

    /* Toggle LED */
    #[task(resources = [led])]
    fn toggle(cx: toggle::Context) {
        let led = cx.resources.led;
        led.toggle().ok(); // ignoring errors
    }

    #[task(binds=USB_HP_CAN_TX, resources = [usb_dev, hid], priority=3)]
    fn usb_tx(cx: usb_tx::Context) {
        usb_poll(cx.resources.usb_dev, cx.resources.hid);
    }

    #[task(binds=USB_LP_CAN_RX0, resources = [usb_dev, hid], priority=3)]
    fn usb_rx(cx: usb_rx::Context) {
        usb_poll(cx.resources.usb_dev, cx.resources.hid);
    }

    // UART interrupt, read from the RX buffer and write to hid
    #[task(binds = USART1, resources = [rx, serial_buffer], priority = 3, spawn = [handle_cmd, toggle])]
    fn usart1(cx: usart1::Context) {
        let buf = cx.resources.serial_buffer;
        if !buf.has_capacity() {
            buf.clear();
            // TODO better overflow handling? Is this needed at all?
        }
        match cx.resources.rx.read() {
            Ok(byte) if buf.len() == 0 && byte != 0x33 => {
                // ignore
            }
            Ok(byte) => {
                //set_led(led, buf.len % 2 == 0);
                buf.put(byte).unwrap(); // checked capacity above
                if buf.len() >= 8 {
                    let mut cmd = [0u8; 8];
                    for b in cmd.iter_mut() {
                        *b = buf.get().unwrap();
                    }
                    cx.spawn.handle_cmd(cmd).ok();
                }
            }
            Err(_error) => {
                // TODO error handling
            }
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            //wfi would safe power, but interferes with
            //debugging / flashing via swd
            //see https://github.com/probe-rs/probe-rs/issues/300
            #[cfg(feature = "wfi")]
            wfi();
            #[cfg(not(feature = "wfi"))]
            core::sync::atomic::spin_loop_hint();
        }
    }

    extern "C" {
        fn EXTI0();
        fn EXTI1();
    }
};

fn calculate_crc(bytes: &[u8]) -> u16 {
    /* This algorithm seems to be incompatible to PiKVM
    let mut digest = crc::crc16::Digest::new_with_initial(0xa001, 0xffff);
    digest.write(bytes);
    digest.sum16()
    */

    pi_kvm_crc(bytes)
}

fn usb_poll<B: bus::UsbBus>(
    usb_dev: &mut UsbDevice<'static, B>,
    hid: &mut Hid<'static, KeyboardReport, B>,
) {
    usb_dev.poll(&mut [hid]);
}

#[derive(Clone)]
pub enum ProtoResponse {
    None,
    Ok,
    CrcError,
    InvalidError,
    Pong,
}

// TODO: make generic
fn send_cmd_response(tx: &mut serial::Tx<USART1>, resp: ProtoResponse) {
    match resp {
        ProtoResponse::Pong => {
            let mut buf = [0x33u8, 0x80, 0x00, 0x00];
            let crc = calculate_crc(&buf[0..2]);
            buf[2] = (crc >> 8) as u8;
            buf[3] = (crc & 0xff) as u8;
            for &b in buf.iter() {
                tx.write(b).ok();
                // TODO hack
                delay(48_000_000 / 10000);
            }
        }
        ProtoResponse::Ok => {
            let mut buf = [0x33u8, 0x20, 0x00, 0x00];
            let crc = calculate_crc(&buf[0..2]);
            buf[2] = (crc >> 8) as u8;
            buf[3] = (crc & 0xff) as u8;
            for &b in buf.iter() {
                tx.write(b).ok();
                // TODO hack
                delay(48_000_000 / 10000);
            }
        }
        ProtoResponse::InvalidError => {
            let mut buf = [0x33u8, 0x45, 0x00, 0x00];
            let crc = calculate_crc(&buf[0..2]);
            buf[2] = (crc >> 8) as u8;
            buf[3] = (crc & 0xff) as u8;
            for &b in buf.iter() {
                tx.write(b).ok();
                // TODO hack
                delay(48_000_000 / 10000);
            }
        }
        ProtoResponse::CrcError => (),
        _ => (),
    }
}

fn pi_kvm_crc(bytes: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    for &byte in bytes {
        crc ^= byte as u16;
        for _ in 0..8 {
            if crc & 0x0001 == 0 {
                crc >>= 1;
            } else {
                crc >>= 1;
                crc ^= 0xa001;
            }
        }
    }
    crc
}
