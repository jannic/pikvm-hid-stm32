#![no_std]
#![no_main]

// Mostly taken from
// https://github.com/stm32-rs/stm32-usbd-examples/blob/master/example-stm32f103c8/examples/usb_mouse_rtfm.rs

extern crate panic_semihosting;

use core::convert::TryInto;

#[cfg(feature = "wfi")]
use cortex_m::wfi;

use cortex_m::{asm::delay, peripheral::DWT};
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::stm32::USART1;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use stm32f1xx_hal::{gpio, prelude::*, serial, serial::Event as SerialEvent, serial::Serial};

use usb_device::bus;
use usb_device::prelude::*;

use usbd_hid_device::Hid;

use keycode::KeyboardState;

mod buf;
use buf::Buffer;

mod report;
use report::KeyboardReport;
use report::MouseReport;

mod proto;

type LED = gpio::gpioc::PC13<gpio::Output<gpio::PushPull>>;

pub struct MouseState {
    pub x: u16,
    pub y: u16,
    pub buttons: u16,
}

impl MouseState {
    fn new() -> Self {
        Self {
            x: 0,
            y: 0,
            buttons: 0,
        }
    }
    fn report(&self) -> MouseReport {
        MouseReport::new((self.buttons & 0xff) as u8, self.x, self.y)
    }
}

#[rtic::app(device = stm32f1xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        proto_state: proto::ProtoState,
        led: LED,
        serial_buffer: Buffer,
        keyboard_state: KeyboardState,
        mouse_state: MouseState,

        usb_dev: UsbDevice<'static, UsbBusType>,
        keyboard_hid: Hid<'static, KeyboardReport, UsbBusType>,
        mouse_hid: Hid<'static, MouseReport, UsbBusType>,
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

        let keyboard_hid = Hid::new(USB_BUS.as_ref().unwrap(), 10);
        let mouse_hid = Hid::new(USB_BUS.as_ref().unwrap(), 10);

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

        let mouse_state = MouseState::new();

        init::LateResources {
            proto_state: proto::ProtoState::default(),
            led,
            serial_buffer,
            keyboard_state,
            mouse_state,

            usb_dev,
            keyboard_hid,
            mouse_hid,

            tx,
            rx,
        }
    }

    #[task(resources = [tx, proto_state, keyboard_hid, mouse_hid, keyboard_state, mouse_state])]
    fn handle_cmd(cx: handle_cmd::Context, cmd: [u8; 8]) {
        let tx = cx.resources.tx;
        let proto_state: &mut proto::ProtoState = cx.resources.proto_state;
        let mut keyboard_hid = cx.resources.keyboard_hid;
        let mut mouse_hid = cx.resources.mouse_hid;
        let keyboard_state = cx.resources.keyboard_state;
        let mouse_state = cx.resources.mouse_state;

        let send_keyboard_report = |state: &mut KeyboardState| {
            let report = KeyboardReport::new(
                state
                    .usb_input_report()
                    .try_into()
                    .expect("report should be 8 bytes long"),
            );

            keyboard_hid.lock(|hid| hid.send_report(&report))
        };
        let send_mouse_report =
            |state: &mut MouseState| mouse_hid.lock(|hid| hid.send_report(&state.report()));
        proto::handle_cmd(
            cmd,
            tx,
            proto_state,
            keyboard_state,
            send_keyboard_report,
            mouse_state,
            send_mouse_report,
        )
    }

    /* Toggle LED */
    #[task(resources = [led])]
    fn toggle(cx: toggle::Context) {
        let led = cx.resources.led;
        led.toggle().ok(); // ignoring errors
    }

    #[task(binds=USB_HP_CAN_TX, resources = [usb_dev, keyboard_hid, mouse_hid], priority=3)]
    fn usb_tx(cx: usb_tx::Context) {
        usb_poll(
            cx.resources.usb_dev,
            cx.resources.keyboard_hid,
            cx.resources.mouse_hid,
        );
    }

    #[task(binds=USB_LP_CAN_RX0, resources = [usb_dev, keyboard_hid, mouse_hid], priority=3)]
    fn usb_rx(cx: usb_rx::Context) {
        usb_poll(
            cx.resources.usb_dev,
            cx.resources.keyboard_hid,
            cx.resources.mouse_hid,
        );
    }

    // UART interrupt, read from the RX buffer and write to hid
    #[task(binds = USART1, resources = [rx, serial_buffer], priority = 3, spawn = [handle_cmd, toggle])]
    fn usart1(cx: usart1::Context) {
        let buf = cx.resources.serial_buffer;
        if !buf.has_capacity() {
            buf.clear(); // should not happen, as buffer is cleared below, as soon as it contains 8 bytes
        }
        match cx.resources.rx.read() {
            Ok(byte) if buf.len() == 0 && byte != 0x33 => { /* ignore */ }
            Ok(byte) => {
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

fn usb_poll<B: bus::UsbBus>(
    usb_dev: &mut UsbDevice<'static, B>,
    keyboard_hid: &mut Hid<'static, KeyboardReport, B>,
    mouse_hid: &mut Hid<'static, MouseReport, B>,
) {
    usb_dev.poll(&mut [keyboard_hid, mouse_hid]);
}
