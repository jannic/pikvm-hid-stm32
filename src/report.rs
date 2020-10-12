//! Taken from https://github.com/agalakhov/usbd-hid-device-example
//! Copyright (c) 2020 Alexey Galakhov, license MIT OR Apache-2.0
//!
//! Modified by Jan Niehusmann, same license.
//!
//! HID report for Keyboard

use usbd_hid_device::HidReport;

/// Hid report for a keyboard
pub struct KeyboardReport {
    // byte 0: modifier keys
    // byte 1: reserved
    // byte 2-7: keys
    bytes: [u8; 8],
}

impl KeyboardReport {
    pub fn new(keys: [u8;8 ]) -> Self {
        KeyboardReport {
            bytes: keys
        }
    }
}

impl AsRef<[u8]> for KeyboardReport {
    fn as_ref(&self) -> &[u8] {
        &self.bytes
    }
}

impl HidReport for KeyboardReport {
    // Descriptor taken from
    // https://github.com/TeXitoi/keyberon/blob/master/src/keyboard.rs
    // License: MIT
    const DESCRIPTOR: &'static [u8] = &[
        0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25,
        0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01, 0x75, 0x08, 0x81, 0x03, 0x95, 0x05,
        0x75, 0x01, 0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02, 0x95, 0x01, 0x75, 0x03, 0x91,
        0x03, 0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0xFB, 0x05, 0x07, 0x19, 0x00, 0x29, 0xFB,
        0x81, 0x00, 0x09, 0x03, 0x75, 0x08, 0x95, 0x40, 0xB1, 0x02, 0xC0,
    ];
}


/// Hid report for a 3-button mouse.
pub struct MouseReport {
    // byte 0: buttons
    // byte 1,2: x-axis
    // byte 3,4: y-axis
    bytes: [u8; 7],
}

impl MouseReport {
    pub fn new(buttons: u8, x: u16, y: u16) -> Self {
        MouseReport {
            bytes: [buttons, x as u8, (x >> 8) as u8, y as u8, (y >> 8) as u8,  0, 0]
        }
    }
}

impl AsRef<[u8]> for MouseReport {
    fn as_ref(&self) -> &[u8] {
        &self.bytes
    }
}

impl HidReport for MouseReport {
    const DESCRIPTOR: &'static [u8] = &[
        0x05, 0x01,     // USAGE_PAGE Generic Desktop
        0x09, 0x02,     // USAGE Mouse
        0xa1, 0x01,     // COLLECTION Application
            0x09, 0x01,     // USAGE Pointer
            0xa1, 0x00,     // COLLECTION Physical

                0x05, 0x09,     // USAGE_PAGE Button
                0x19, 0x01,     // USAGE_MINIMUM Button 1
                0x29, 0x03,     // USAGE_MAXIMUM Button 3
                0x15, 0x00,     // LOGICAL_MINIMUM 0
                0x25, 0x01,     // LOGICAL_MAXIMUM 1
                0x95, 0x03,     // REPORT_COUNT 3
                0x75, 0x01,     // REPORT_SIZE 1
                0x81, 0x02,     // INPUT Data,Var,Abs
                0x95, 0x01,     // REPORT_COUNT 1
                0x75, 0x05,     // REPORT_SIZE 5
                0x81, 0x01,     // INPUT Cnst,Ary,Abs

                0x05, 0x01,     // USAGE_PAGE Generic Desktop
                0x09, 0x30,     // USAGE X
                0x09, 0x31,     // USAGE Y
                0x09, 0x38,     // USAGE Wheel
                0x16, 0x00, 0x00,   // LOGICAL_MINIMUM 0
                0x26, 0xff, 0x7f,   // LOGICAL_MAXIMUM 32767
                0x75, 0x10,     // REPORT_SIZE 16
                0x95, 0x03,     // REPORT_COUNT 3
                0x81, 0x02,     // INPUT Data,Var,Abs

            0xc0,           // END COLLECTION
        0xc0,           // END COLLECTION
    ];
}

