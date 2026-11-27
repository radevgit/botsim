//! What an order promises.

use botsim_link::ids::{ActionId, ActionType, OrderId};
use botsim_link::order::{Action, Blocking, Leg, NodeAction, Order};
use botsim_link::shape::ShapeError;

/// The highest `step` whose `sequenceId` (twice as much) still fits in 32 bits.
const LAST_STEP: u32 = u32::MAX / 2;

/// `n` legs: leg `i` is edge `100 + i` and ends at node `11 + i`.
fn legs(n: usize) -> Vec<Leg> {
    (0..n)
        .map(|i| Leg {
            edge: 100 + i,
            end: 11 + i,
            orientation: None,
            orientation_type: None,
        })
        .collect()
}

fn action(node: u32, id: &str) -> NodeAction {
    NodeAction {
        node,
        action: Action {
            id: ActionId::new(id),
            kind: ActionType::new("pick"),
            blocking: Blocking::Hard,
        },
    }
}

fn order(
    first_step: u32,
    n: usize,
    released: u32,
    actions: Vec<NodeAction>,
) -> Result<Order, ShapeError> {
    Order::new(
        OrderId::new("job-5"),
        0,
        first_step,
        10,
        legs(n),
        released,
        actions,
    )
}

#[test]
fn it_gives_back_what_it_was_made_of() {
    let o = Order::new(
        OrderId::new("job-5"),
        3,
        2,
        10,
        legs(2),
        1,
        vec![action(1, "a")],
    )
    .expect("an order");
    assert_eq!(o.order_id().as_str(), "job-5");
    assert_eq!((o.update_id(), o.first_step()), (3, 2));
    assert_eq!((o.start(), o.released()), (10, 1));
    assert_eq!(o.legs(), legs(2));
    assert_eq!(o.actions(), [action(1, "a")]);
}

#[test]
fn nodes_are_the_start_and_the_ends_of_the_legs() {
    let o = order(0, 2, 0, Vec::new()).expect("an order");
    assert_eq!(
        [o.node(0), o.node(1), o.node(2), o.node(3)],
        [Some(10), Some(11), Some(12), None]
    );
}

#[test]
fn sequence_ids_follow_from_the_position() {
    // a new order starts at 0, an update at the node it starts at
    for (first_step, first) in [(0, 0), (7, 14)] {
        let o = order(first_step, 2, 1, Vec::new()).expect("an order");
        assert_eq!(
            [
                o.node_sequence_id(0),
                o.leg_sequence_id(0),
                o.node_sequence_id(1),
                o.leg_sequence_id(1),
                o.node_sequence_id(2),
            ],
            [0, 1, 2, 3, 4].map(|i| Some(first + i))
        );
        assert_eq!(o.node_sequence_id(3), None);
        assert_eq!(o.leg_sequence_id(2), None);
        assert_eq!(o.node_sequence_id(u32::MAX), None);
        assert_eq!(o.leg_sequence_id(u32::MAX), None);
    }
}

#[test]
fn an_order_can_be_a_node_alone() {
    let o = order(0, 0, 0, vec![action(0, "a")]).expect("an order");
    assert_eq!((o.node(0), o.node(1)), (Some(10), None));
    assert_eq!(o.leg_sequence_id(0), None);
}

#[test]
fn no_more_legs_are_released_than_there_are() {
    assert!(order(0, 2, 0, Vec::new()).is_ok());
    assert!(order(0, 2, 2, Vec::new()).is_ok());
    assert_eq!(
        order(0, 2, 3, Vec::new()),
        Err(ShapeError::TooManyReleased {
            released: 3,
            there: 2
        })
    );
}

#[test]
fn every_action_is_at_a_node_of_the_order() {
    // also in the horizon
    assert!(order(0, 2, 0, vec![action(2, "a")]).is_ok());
    assert_eq!(
        order(0, 2, 2, vec![action(0, "a"), action(3, "b")]),
        Err(ShapeError::ActionBeyondRoute { node: 3, last: 2 })
    );
}

#[test]
fn actions_are_sorted_by_node_and_keep_their_order_at_a_node() {
    let given = vec![
        action(2, "d"),
        action(0, "b"),
        action(2, "c"),
        action(0, "a"),
    ];
    let o = order(0, 2, 2, given).expect("an order");
    let ids = |actions: &[NodeAction]| -> Vec<String> {
        actions
            .iter()
            .map(|a| a.action.id.as_str().to_string())
            .collect()
    };
    assert_eq!(ids(o.actions()), ["b", "a", "d", "c"]);
    assert_eq!(ids(o.actions_at(0)), ["b", "a"]);
    assert!(o.actions_at(1).is_empty());
    assert_eq!(ids(o.actions_at(2)), ["d", "c"]);
    assert!(o.actions_at(3).is_empty());
}

#[test]
fn every_sequence_id_fits_in_32_bits() {
    let o = order(LAST_STEP - 2, 2, 2, Vec::new()).expect("the last numbers");
    assert_eq!(o.node_sequence_id(2), Some(u32::MAX - 1));
    assert_eq!(o.leg_sequence_id(1), Some(u32::MAX - 2));
    for first_step in [LAST_STEP - 1, u32::MAX] {
        assert_eq!(
            order(first_step, 2, 2, Vec::new()),
            Err(ShapeError::SequenceTooLarge)
        );
    }
}
