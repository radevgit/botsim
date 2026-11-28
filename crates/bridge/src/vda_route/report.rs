//! What the route through `vda` could not translate between the world and the
//! controller. Nothing of it is dropped silently: each case is counted here.

use std::fmt;

use botsim_model::robot::RobotNo;
use botsim_vda::report::Issue;
use botsim_vda::translator::RobotTableError;

/// The most different cases that are kept; further ones are only counted.
pub const MOST_ENTRIES: usize = 100;

/// What was being translated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum What {
    /// Giving a robot its number.
    Robot,
    /// A robot's state message.
    State,
    Order,
    InstantActions,
    Responses,
    /// A zone set message to a robot.
    ZoneSet,
    /// The site's zone set, when the controller is set up.
    SiteZoneSet,
}

/// Whom it was of or for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Subject {
    Site,
    Robot(RobotNo),
}

/// Why it could not be translated.
#[derive(Debug, Clone, PartialEq)]
pub enum Reason {
    /// What the translator says (`botsim-vda`).
    Translator(Issue),
    /// The translator's table of robots did not take the robot.
    Table(RobotTableError),
}

/// One case, with how often it came up.
#[derive(Debug, Clone, PartialEq)]
pub struct Entry {
    pub what: What,
    pub subject: Subject,
    /// The subject in words: `manufacturer/serialNumber` of a robot (the number
    /// if there is no such one), or the id of the site's zone set.
    pub name: String,
    /// Nothing of it went through; `false`: the rest did, without the part.
    pub refused: bool,
    pub reason: Reason,
    /// The tick of the world when it came up first.
    pub first_tick: u64,
    pub count: u64,
}

/// All that could not be translated so far.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Untranslated {
    entries: Vec<Entry>,
    total: u64,
}

impl Untranslated {
    pub fn is_empty(&self) -> bool {
        self.total == 0
    }

    /// How many cases came up, counting each time.
    pub fn total(&self) -> u64 {
        self.total
    }

    /// The different cases, in the order they came up first; at most
    /// [`MOST_ENTRIES`].
    pub fn entries(&self) -> &[Entry] {
        &self.entries
    }

    /// Count a case. `name` is asked only for a new one.
    pub(crate) fn note(
        &mut self,
        (what, subject): (What, Subject),
        refused: bool,
        reason: Reason,
        tick: u64,
        name: impl FnOnce() -> String,
    ) {
        self.total = self.total.saturating_add(1);
        let same = |e: &&mut Entry| {
            (e.what, e.subject, e.refused) == (what, subject, refused) && e.reason == reason
        };
        if let Some(entry) = self.entries.iter_mut().find(same) {
            entry.count = entry.count.saturating_add(1);
        } else if self.entries.len() < MOST_ENTRIES {
            self.entries.push(Entry {
                what,
                subject,
                name: name(),
                refused,
                reason,
                first_tick: tick,
                count: 1,
            });
        }
    }
}

impl fmt::Display for What {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            What::Robot => "robot",
            What::State => "state",
            What::Order => "order",
            What::InstantActions => "instantActions",
            What::Responses => "responses",
            What::ZoneSet => "zoneSet",
            What::SiteZoneSet => "site zone set",
        })
    }
}

impl fmt::Display for Reason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Reason::Translator(issue) => issue.fmt(f),
            Reason::Table(error) => error.fmt(f),
        }
    }
}

#[cfg(test)]
mod tests {
    use botsim_vda::report::{Item, Problem};

    use super::*;

    #[test]
    fn equal_cases_are_counted_together_and_the_list_is_bounded() {
        let mut report = Untranslated::default();
        assert!(report.is_empty());
        let robot = |n| (What::Order, Subject::Robot(RobotNo::new(n, 0)));
        let reason = || {
            Reason::Translator(Issue {
                problem: Problem::OutsideLayout {
                    item: Item::Node,
                    number: 9,
                },
                error: None,
                references: Vec::new(),
                entry: None,
            })
        };
        for tick in [3, 4, 9] {
            report.note(robot(0), true, reason(), tick, || "r".to_string());
        }
        assert_eq!(report.total(), 3);
        let [entry] = report.entries() else {
            panic!("one case: {:?}", report.entries());
        };
        assert_eq!((entry.count, entry.first_tick), (3, 3));
        for n in 1..=2 * MOST_ENTRIES as u32 {
            report.note(robot(n), true, reason(), 10, || n.to_string());
        }
        assert_eq!(report.entries().len(), MOST_ENTRIES);
        assert_eq!(report.total(), 3 + 2 * MOST_ENTRIES as u64);
        // a known case is still counted
        report.note(robot(0), true, reason(), 11, String::new);
        assert_eq!(report.entries()[0].count, 4);
    }
}
