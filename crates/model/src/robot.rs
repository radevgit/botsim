//! Who a robot is: its VDA 5050 identity, and the small number that stands for it
//! inside the program.

/// Identity of a robot, as in the VDA 5050 topic.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct RobotId {
    pub manufacturer: String,
    pub serial_number: String,
}

impl RobotId {
    pub fn new(manufacturer: &str, serial_number: &str) -> RobotId {
        RobotId {
            manufacturer: manufacturer.to_string(),
            serial_number: serial_number.to_string(),
        }
    }
}

/// The small number of a robot: a slot, usable as an array index, and the
/// generation of that slot.
///
/// A slot that a robot has left can be used again, with the next generation. So the
/// number of a robot that is gone is not equal to the number of its successor.
/// Which [`RobotId`] a number stands for is kept by whoever hands the numbers out.
///
/// The order is by slot, then by generation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct RobotNo {
    slot: u32,
    generation: u32,
}

impl RobotNo {
    pub const fn new(slot: u32, generation: u32) -> RobotNo {
        RobotNo { slot, generation }
    }

    pub const fn slot(self) -> u32 {
        self.slot
    }

    /// The slot as an array index.
    pub const fn index(self) -> usize {
        // (a `usize` holds every `u32` on the 32- and 64-bit machines we build for)
        self.slot as usize
    }

    pub const fn generation(self) -> u32 {
        self.generation
    }

    /// The number for the next robot in the same slot; `None` if the slot has no
    /// generation left. It must not be used again then.
    pub const fn next_generation(self) -> Option<RobotNo> {
        match self.generation.checked_add(1) {
            Some(generation) => Some(RobotNo::new(self.slot, generation)),
            None => None,
        }
    }
}
