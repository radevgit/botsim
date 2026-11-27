//! What the route of a state report promises.

use botsim_link::shape::ShapeError;
use botsim_link::state::{Ahead, LastNode, Route};

/// The highest `step` whose `sequenceId` (twice as much) still fits in 32 bits.
const LAST_STEP: u32 = u32::MAX / 2;

fn ahead(n: usize) -> Vec<Ahead> {
    (0..n)
        .map(|i| Ahead {
            edge: 100 + i,
            node: 11 + i,
        })
        .collect()
}

fn at(step: u32) -> Option<LastNode> {
    Some(LastNode { node: 10, step })
}

#[test]
fn it_gives_back_what_it_was_made_of() {
    let r = Route::new(at(4), ahead(2), 1).expect("a route");
    assert_eq!(r.last_node(), at(4));
    assert_eq!(r.ahead(), ahead(2));
    assert_eq!(r.released(), 1);
}

#[test]
fn sequence_ids_go_on_from_the_last_node() {
    let r = Route::new(at(4), ahead(2), 1).expect("a route");
    assert_eq!(
        [
            r.leg_sequence_id(0),
            r.node_sequence_id(0),
            r.leg_sequence_id(1),
            r.node_sequence_id(1),
        ],
        [Some(9), Some(10), Some(11), Some(12)]
    );
    assert_eq!(r.leg_sequence_id(2), None);
    assert_eq!(r.node_sequence_id(2), None);
    assert_eq!(r.node_sequence_id(u32::MAX), None);
}

#[test]
fn nothing_is_ahead_without_a_last_node() {
    let none = Route::new(None, Vec::new(), 0).expect("a robot without an order");
    assert_eq!(none.leg_sequence_id(0), None);
    assert_eq!(
        Route::new(None, ahead(1), 0),
        Err(ShapeError::AheadWithoutLastNode)
    );
}

#[test]
fn no_more_entries_are_released_than_there_are() {
    assert!(Route::new(at(0), ahead(2), 2).is_ok());
    assert_eq!(
        Route::new(at(0), ahead(2), 3),
        Err(ShapeError::TooManyReleased {
            released: 3,
            there: 2
        })
    );
    assert_eq!(
        Route::new(None, Vec::new(), 1),
        Err(ShapeError::TooManyReleased {
            released: 1,
            there: 0
        })
    );
}

#[test]
fn every_sequence_id_fits_in_32_bits() {
    let r = Route::new(at(LAST_STEP - 2), ahead(2), 2).expect("the last numbers");
    assert_eq!(r.node_sequence_id(1), Some(u32::MAX - 1));
    // the last node's own number counts too, with nothing ahead
    assert!(Route::new(at(LAST_STEP), Vec::new(), 0).is_ok());
    for (step, entries) in [(LAST_STEP - 1, 2), (LAST_STEP + 1, 0), (u32::MAX, 1)] {
        assert_eq!(
            Route::new(at(step), ahead(entries), 0),
            Err(ShapeError::SequenceTooLarge)
        );
    }
}
