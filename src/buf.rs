/// Simple ring buffer
pub struct Buffer {
    buffer: [u8; 16],
    pos: u8,
    len: u8,
}

impl Buffer {
    pub fn new() -> Buffer {
        Buffer {
            buffer: [0u8; 16],
            pos: 0,
            len: 0,
        }
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }

    pub fn get(&mut self) -> Option<u8> {
        if self.len > 0 {
            self.len -= 1;
            let val = Some(self.buffer[self.pos as usize]);
            self.pos = (self.pos + 1) % 16;
            val
        } else {
            None
        }
    }

    pub fn put(&mut self, val: u8) -> Result<(), ()> {
        if self.len < 16 {
            self.buffer[((self.pos + self.len) % 16) as usize] = val;
            self.len += 1;
            Ok(())
        } else {
            Err(())
        }
    }

    pub fn has_capacity(&self) -> bool {
        self.len < 16
    }

    pub fn len(&self) -> u8 {
        self.len
    }
}

impl Default for Buffer {
    fn default() -> Self {
        Self::new()
    }
}
