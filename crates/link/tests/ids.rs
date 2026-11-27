//! What the text ids promise.

use botsim_link::ids::{ActionId, ActionType, OrderId, RequestId, ZoneId, ZoneSetId};

#[test]
fn an_id_is_its_text() {
    let id = OrderId::new("job-5");
    assert_eq!(id.as_str(), "job-5");
    assert_eq!(id.to_string(), "job-5");
    assert_eq!(format!("[{id:>7}]"), "[  job-5]");
    assert_eq!(id, OrderId::new(String::from("job-5")));
    assert_ne!(id, OrderId::new("job-6"));
}

#[test]
fn any_text_is_accepted() {
    for text in ["", " ", "pick-2", "Förderband/7"] {
        assert_eq!(ActionId::new(text).as_str(), text);
        assert_eq!(ActionType::new(text).as_str(), text);
        assert_eq!(RequestId::new(text).as_str(), text);
        assert_eq!(ZoneId::new(text).as_str(), text);
        assert_eq!(ZoneSetId::new(text).as_str(), text);
    }
}

#[test]
fn a_copy_shares_the_text() {
    let id = OrderId::new("job-5");
    assert!(std::ptr::eq(id.as_str(), id.clone().as_str()));
}
