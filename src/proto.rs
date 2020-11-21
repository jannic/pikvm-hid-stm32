use super::MouseState;
use core::convert::TryInto;

use cortex_m::asm::delay;
use embedded_hal::serial::Write;
use keycode::{KeyMap, KeyMapping, KeyMappingId, KeyState, KeyboardState};
use usb_device::UsbError;

pub const CMD_PING: u8 = 0x01;
pub const CMD_REPEAT: u8 = 0x02;

pub const CMD_KEY_EVENT: u8 = 0x11;
pub const CMD_MOUSE_MOVE_EVENT: u8 = 0x12;
pub const CMD_MOUSE_BUTTON_EVENT: u8 = 0x13;

pub const CMD_MOUSE_BUTTON_LEFT_SELECT: u8 = 0b10000000;
pub const CMD_MOUSE_BUTTON_LEFT_STATE: u8 = 0b00001000;
pub const CMD_MOUSE_BUTTON_MIDDLE_SELECT: u8 = 0b00100000;
pub const CMD_MOUSE_BUTTON_MIDDLE_STATE: u8 = 0b00000010;
pub const CMD_MOUSE_BUTTON_RIGHT_SELECT: u8 = 0b01000000;
pub const CMD_MOUSE_BUTTON_RIGHT_STATE: u8 = 0b00000100;

pub struct ProtoState {
    prev: ProtoResponse,
}

impl Default for ProtoState {
    fn default() -> Self {
        Self {
            prev: ProtoResponse::None,
        }
    }
}

#[derive(Clone)]
pub enum ProtoResponse {
    None,
    Ok,
    CrcError,
    InvalidError,
    Pong,
}

pub struct SerialKeyCode(pub u8);

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
            106 => KeyMapping::Id(KeyMappingId::IntlBackslash),
            _ => KeyMapping::Code(None), // TODO: Better handling of unknown codes
        }
    }
}

pub fn handle_cmd<
    TX: Write<u8>,
    KeyFn: FnMut(&mut KeyboardState) -> Result<usize, UsbError>,
    MouseFn: FnMut(&mut MouseState) -> Result<usize, UsbError>,
>(
    cmd: [u8; 8],
    tx: &mut TX,
    proto_state: &mut ProtoState,
    keyboard_state: &mut KeyboardState,
    mut send_keyboard_report: KeyFn,
    mouse_state: &mut MouseState,
    mut send_mouse_report: MouseFn,
) {
    let crc: u16 = ((cmd[6] as u16) << 8) | (cmd[7] as u16);
    if calculate_crc(&cmd[0..6]) == crc {
        match cmd[1] {
            CMD_PING => {
                proto_state.prev = ProtoResponse::Pong;
                send_cmd_response(tx, ProtoResponse::Pong);
            }
            CMD_REPEAT => {
                send_cmd_response(tx, proto_state.prev.clone());
            }
            CMD_KEY_EVENT => {
                proto_state.prev = ProtoResponse::Ok;
                let key_mapping: KeyMapping = SerialKeyCode(cmd[2]).into();
                let key: Result<KeyMap, ()> = KeyMap::from_key_mapping(key_mapping);
                let resp = key.map(|key| {
                    if cmd[3] != 0 {
                        keyboard_state.update_key(key, KeyState::Pressed);
                    } else {
                        keyboard_state.update_key(key, KeyState::Released);
                    }
                    send_keyboard_report(keyboard_state)
                });

                if resp.is_ok() {
                    send_cmd_response(tx, ProtoResponse::Ok);
                } else {
                    send_cmd_response(tx, ProtoResponse::InvalidError);
                }
            }
            CMD_MOUSE_MOVE_EVENT => {
                proto_state.prev = ProtoResponse::Ok;
                let x = ((i16::from_be_bytes(cmd[2..=3].try_into().unwrap()) / 2) as u16)
                    .wrapping_add(0x4000);
                let y = ((i16::from_be_bytes(cmd[4..=5].try_into().unwrap()) / 2) as u16)
                    .wrapping_add(0x4000);
                mouse_state.x = x;
                mouse_state.y = y;

                let resp = send_mouse_report(mouse_state);
                if resp.is_ok() {
                    send_cmd_response(tx, ProtoResponse::Ok);
                } else {
                    send_cmd_response(tx, ProtoResponse::InvalidError);
                }
            }
            CMD_MOUSE_BUTTON_EVENT => {
                proto_state.prev = ProtoResponse::Ok;
                let state = cmd[2];
                if state & CMD_MOUSE_BUTTON_LEFT_SELECT != 0 {
                    if state & CMD_MOUSE_BUTTON_LEFT_STATE != 0 {
                        mouse_state.buttons |= 1 << 0;
                    } else {
                        mouse_state.buttons &= !(1 << 0);
                    }
                }
                if state & CMD_MOUSE_BUTTON_RIGHT_SELECT != 0 {
                    if state & CMD_MOUSE_BUTTON_RIGHT_STATE != 0 {
                        mouse_state.buttons |= 1 << 1;
                    } else {
                        mouse_state.buttons &= !(1 << 1);
                    }
                }
                if state & CMD_MOUSE_BUTTON_MIDDLE_SELECT != 0 {
                    if state & CMD_MOUSE_BUTTON_MIDDLE_STATE != 0 {
                        mouse_state.buttons |= 1 << 2;
                    } else {
                        mouse_state.buttons &= !(1 << 2);
                    }
                }
                let resp = send_mouse_report(mouse_state);
                if resp.is_ok() {
                    send_cmd_response(tx, ProtoResponse::Ok);
                } else {
                    send_cmd_response(tx, ProtoResponse::InvalidError);
                }
            }
            _ => send_cmd_response(tx, ProtoResponse::InvalidError),
        }
    } else {
        send_cmd_response(tx, ProtoResponse::CrcError);
    }
}

fn send_cmd_response<TX: Write<u8>>(tx: &mut TX, resp: ProtoResponse) {
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

fn calculate_crc(bytes: &[u8]) -> u16 {
    /* This algorithm seems to be incompatible to PiKVM
    let mut digest = crc::crc16::Digest::new_with_initial(0xa001, 0xffff);
    digest.write(bytes);
    digest.sum16()
    */

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
