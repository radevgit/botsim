//! A small hash to tell whether two runs of the same build are in the same state,
//! or two pieces of data are the same.
//!
//! [`Checksum`] is a [`Hasher`], so that anything that derives or implements
//! [`Hash`](std::hash::Hash) can be fed to it. It is no cryptographic hash. Its
//! values are **for one build on one kind of machine**: what is written for a value
//! (and so the result) can change with the compiler and with the types hashed.
//!
//! **What is certain:** two sequences of writes that differ in one write of the
//! same length give different results (every step is one to one in the state and in
//! what is added). So one changed number, or one changed byte, is always seen.
//! Sequences that differ in more places give different results in all but chance
//! cases.

use std::hash::Hasher;

/// An odd number with well spread bits (2⁶⁴ divided by the golden ratio).
const FACTOR: u64 = 0x9E37_79B9_7F4A_7C15;

/// The hash; see the module text.
#[derive(Debug, Clone, Default)]
pub struct Checksum(u64);

impl Checksum {
    pub fn new() -> Checksum {
        Checksum::default()
    }

    /// The checksum of `bytes` alone.
    pub fn of_bytes(bytes: &[u8]) -> u64 {
        let mut sum = Checksum::new();
        sum.write(bytes);
        sum.finish()
    }

    #[inline]
    fn add(&mut self, word: u64) {
        // (one to one in the state for a given word, and in the word for a given
        // state)
        self.0 = (self.0.rotate_left(23) ^ word)
            .wrapping_mul(FACTOR)
            .wrapping_add(1);
    }
}

impl Hasher for Checksum {
    fn write(&mut self, bytes: &[u8]) {
        let (words, rest) = bytes.as_chunks::<8>();
        for word in words {
            self.add(u64::from_le_bytes(*word));
        }
        if !rest.is_empty() {
            // (at most 7 bytes: the top one says how many, so that trailing zeros
            // count)
            let mut w = [0; 8];
            w[..rest.len()].copy_from_slice(rest);
            w[7] = rest.len() as u8;
            self.add(u64::from_le_bytes(w));
        }
    }

    fn write_u8(&mut self, i: u8) {
        self.add(u64::from(i));
    }

    fn write_u16(&mut self, i: u16) {
        self.add(u64::from(i));
    }

    fn write_u32(&mut self, i: u32) {
        self.add(u64::from(i));
    }

    fn write_u64(&mut self, i: u64) {
        self.add(i);
    }

    fn write_usize(&mut self, i: usize) {
        self.add(i as u64);
    }

    fn write_i8(&mut self, i: i8) {
        self.add(i as u64);
    }

    fn write_i16(&mut self, i: i16) {
        self.add(i as u64);
    }

    fn write_i32(&mut self, i: i32) {
        self.add(i as u64);
    }

    fn write_i64(&mut self, i: i64) {
        self.add(i as u64);
    }

    fn write_isize(&mut self, i: isize) {
        self.add(i as u64);
    }

    fn finish(&self) -> u64 {
        // (spreads every bit of the state over the result; one to one)
        let mut x = self.0;
        x ^= x >> 30;
        x = x.wrapping_mul(0xBF58_476D_1CE4_E5B9);
        x ^= x >> 27;
        x = x.wrapping_mul(0x94D0_49BB_1331_11EB);
        x ^ (x >> 31)
    }
}

#[cfg(test)]
mod tests {
    use std::hash::{Hash, Hasher};

    use super::Checksum;

    fn of(value: impl Hash) -> u64 {
        let mut sum = Checksum::new();
        value.hash(&mut sum);
        sum.finish()
    }

    #[test]
    fn every_changed_byte_is_seen() {
        let bytes: Vec<u8> = (0..41).collect();
        let sum = Checksum::of_bytes(&bytes);
        for i in 0..bytes.len() {
            for bit in 0..8 {
                let mut other = bytes.clone();
                other[i] ^= 1 << bit;
                assert_ne!(Checksum::of_bytes(&other), sum, "byte {i}, bit {bit}");
            }
        }
    }

    #[test]
    fn trailing_zeros_lengths_and_order_count() {
        assert_ne!(Checksum::of_bytes(&[]), Checksum::of_bytes(&[0]));
        assert_ne!(Checksum::of_bytes(&[1]), Checksum::of_bytes(&[1, 0]));
        assert_ne!(of([1u64, 2]), of([2u64, 1]));
        assert_ne!(
            of((vec![1u64], vec![2u64])),
            of((vec![1u64, 2], vec![0u64; 0]))
        );
        assert_ne!(of(("a", "bc")), of(("ab", "c")));
        assert_eq!(of(("a", 7u32)), of(("a", 7u32)));
    }
}
