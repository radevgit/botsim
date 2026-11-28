//! The primitives of the file format, to write into and read from the bytes of
//! one record.
//!
//! Whole numbers are written in 1 to 10 bytes (LEB128), signed ones folded so that
//! small ones of either sign are short; numbers with a fraction as their 8 bytes,
//! so that they come back bit for bit. A list is its length and then its entries.
//!
//! Nothing read is believed: every length is checked against the bytes that are
//! there, and no memory is set aside for entries before they have been read.

/// What is wrong with the bytes of a record.
pub type Bad = &'static str;

const SHORT: Bad = "a record ends in the middle of its content";

/// Writes into a buffer.
#[derive(Debug, Default)]
pub struct Encoder {
    pub bytes: Vec<u8>,
}

impl Encoder {
    pub fn byte(&mut self, value: u8) {
        self.bytes.push(value);
    }

    pub fn bool(&mut self, value: bool) {
        self.byte(u8::from(value));
    }

    pub fn unsigned(&mut self, mut value: u64) {
        while value >= 0x80 {
            // (the low 7 bits, and the mark that more follow)
            self.byte((value & 0x7F) as u8 | 0x80);
            value >>= 7;
        }
        self.byte(value as u8);
    }

    pub fn signed(&mut self, value: i64) {
        self.unsigned(((value << 1) ^ (value >> 63)) as u64);
    }

    pub fn u32(&mut self, value: u32) {
        self.unsigned(u64::from(value));
    }

    pub fn index(&mut self, value: usize) {
        // (a `usize` has at most 64 bits on the machines we build for)
        self.unsigned(value as u64);
    }

    /// All 8 bytes of a number that is no count, such as a checksum.
    pub fn fixed(&mut self, value: u64) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    pub fn float(&mut self, value: f64) {
        self.fixed(value.to_bits());
    }

    pub fn text(&mut self, value: &str) {
        self.index(value.len());
        self.bytes.extend_from_slice(value.as_bytes());
    }

    pub fn option<T>(&mut self, value: Option<T>, write: impl FnOnce(&mut Encoder, T)) {
        match value {
            Some(value) => {
                self.bool(true);
                write(self, value);
            }
            None => self.bool(false),
        }
    }

    pub fn list<T>(&mut self, entries: &[T], mut write: impl FnMut(&mut Encoder, &T)) {
        self.index(entries.len());
        for entry in entries {
            write(self, entry);
        }
    }
}

/// Reads from the bytes of a record.
#[derive(Debug)]
pub struct Decoder<'a> {
    rest: &'a [u8],
}

impl<'a> Decoder<'a> {
    pub fn new(bytes: &'a [u8]) -> Decoder<'a> {
        Decoder { rest: bytes }
    }

    /// Nothing must be left over.
    pub fn end(self) -> Result<(), Bad> {
        if self.rest.is_empty() {
            Ok(())
        } else {
            Err("a record has bytes after its content")
        }
    }

    fn take(&mut self, count: usize) -> Result<&'a [u8], Bad> {
        let (taken, rest) = self.rest.split_at_checked(count).ok_or(SHORT)?;
        self.rest = rest;
        Ok(taken)
    }

    pub fn byte(&mut self) -> Result<u8, Bad> {
        self.take(1)?.first().copied().ok_or(SHORT)
    }

    pub fn bool(&mut self) -> Result<bool, Bad> {
        match self.byte()? {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err("a yes or no is neither"),
        }
    }

    pub fn unsigned(&mut self) -> Result<u64, Bad> {
        let mut value = 0;
        for shift in (0..64).step_by(7) {
            let byte = self.byte()?;
            let bits = u64::from(byte & 0x7F);
            // (the last byte has room for one bit only)
            if shift == 63 && bits > 1 {
                break;
            }
            value |= bits << shift;
            if byte & 0x80 == 0 {
                return Ok(value);
            }
        }
        Err("a number is too large")
    }

    pub fn signed(&mut self) -> Result<i64, Bad> {
        let value = self.unsigned()?;
        Ok((value >> 1) as i64 ^ -((value & 1) as i64))
    }

    pub fn u32(&mut self) -> Result<u32, Bad> {
        u32::try_from(self.unsigned()?).map_err(|_| "a number is too large")
    }

    pub fn index(&mut self) -> Result<usize, Bad> {
        usize::try_from(self.unsigned()?).map_err(|_| "a number is too large")
    }

    pub fn fixed(&mut self) -> Result<u64, Bad> {
        let mut bytes = [0; 8];
        bytes.copy_from_slice(self.take(8)?);
        Ok(u64::from_le_bytes(bytes))
    }

    pub fn float(&mut self) -> Result<f64, Bad> {
        Ok(f64::from_bits(self.fixed()?))
    }

    pub fn text(&mut self) -> Result<&'a str, Bad> {
        let length = self.index()?;
        std::str::from_utf8(self.take(length)?).map_err(|_| "a text is not UTF-8")
    }

    pub fn string(&mut self) -> Result<String, Bad> {
        self.text().map(str::to_string)
    }

    pub fn option<T>(
        &mut self,
        read: impl FnOnce(&mut Decoder<'a>) -> Result<T, Bad>,
    ) -> Result<Option<T>, Bad> {
        if self.bool()? {
            read(self).map(Some)
        } else {
            Ok(None)
        }
    }

    pub fn list<T>(
        &mut self,
        mut read: impl FnMut(&mut Decoder<'a>) -> Result<T, Bad>,
    ) -> Result<Vec<T>, Bad> {
        let count = self.index()?;
        // (every entry takes at least one byte)
        if count > self.rest.len() {
            return Err(SHORT);
        }
        // grows as entries are read: nothing is set aside for what only the count
        // promises
        let mut entries = Vec::new();
        for _ in 0..count {
            entries.push(read(self)?);
        }
        Ok(entries)
    }
}

#[cfg(test)]
mod tests {
    use super::{Decoder, Encoder};

    #[test]
    fn numbers_come_back() {
        let unsigned = [
            0,
            1,
            127,
            128,
            16_383,
            16_384,
            u64::from(u32::MAX),
            u64::MAX,
        ];
        let signed = [0, 1, -1, 63, -64, 64, i64::MAX, i64::MIN];
        let floats = [0.0, -0.0, 1.5, f64::NAN, f64::INFINITY, f64::MIN_POSITIVE];
        let mut e = Encoder::default();
        unsigned.iter().for_each(|&v| e.unsigned(v));
        signed.iter().for_each(|&v| e.signed(v));
        floats.iter().for_each(|&v| e.float(v));
        e.text("Röbot");
        let mut d = Decoder::new(&e.bytes);
        for v in unsigned {
            assert_eq!(d.unsigned(), Ok(v));
        }
        for v in signed {
            assert_eq!(d.signed(), Ok(v));
        }
        for v in floats {
            assert_eq!(d.float().map(f64::to_bits), Ok(v.to_bits()));
        }
        assert_eq!(d.text(), Ok("Röbot"));
        assert_eq!(d.end(), Ok(()));
    }

    #[test]
    fn what_cannot_be_is_refused() {
        // too long, too large, and cut off
        for bytes in [&[0x80u8; 11][..], &[0xFF; 10], &[0x80]] {
            assert!(Decoder::new(bytes).unsigned().is_err(), "{bytes:?}");
        }
        assert!(Decoder::new(&[2]).bool().is_err());
        // a length far beyond the bytes there are
        let mut e = Encoder::default();
        e.unsigned(u64::MAX);
        assert!(Decoder::new(&e.bytes).text().is_err());
        assert!(Decoder::new(&e.bytes).list(|d| d.byte()).is_err());
        assert!(Decoder::new(&[1, 0xFF]).text().is_err());
        assert!(Decoder::new(&[0]).end().is_err());
    }
}
