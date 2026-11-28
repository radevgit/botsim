//! The messages of a recording: what arrives at the server (reports of robots,
//! states of lifts, new jobs) and the zone set of the setup.
//!
//! **A report is written as what changed** since the report of that robot (by its
//! number) written before: a mask says which groups of fields follow, the others
//! are as before. A position or velocity that changed also says which of its
//! numbers did. Times are written as the distance from the time of the cycle.
//!
//! Every writer takes its value apart in full (no `..`), and every `match` over an
//! enum names each variant: a field or variant added to a message stops the build
//! until it is dealt with here.

use std::collections::BTreeMap;
use std::sync::Arc;

use botsim_link::device::{DeviceHeader, DoorPhase, FromLift, LiftNo};
use botsim_link::ids::{ActionId, ActionType, OrderId, RequestId, ZoneId, ZoneSetId};
use botsim_link::message::{FromRobot, Header, Millis};
use botsim_link::order::Blocking;
use botsim_link::state::{
    ActionState, ActionStatus, Ahead, EmergencyStop, Error, ErrorKind, ErrorLevel, LastNode,
    OperatingMode, Position, RequestStatus, Route, State, Velocity, ZoneRequest, ZoneRequestType,
    ZoneSetInfo,
};
use botsim_link::zone_set::{
    Bidirected, Directed, LossBehavior, Zone, ZoneAction, ZoneKind, ZoneSet,
};
use botsim_model::errors::ErrorType;
use botsim_model::job::Job;
use botsim_model::robot::RobotNo;

use crate::codec::{Bad, Decoder, Encoder};

tags!(
    write_emergency,
    read_emergency,
    EmergencyStop,
    Manual = 0,
    Remote = 1,
    None = 2
);
tags!(
    write_mode,
    read_mode,
    OperatingMode,
    Startup = 0,
    Automatic = 1,
    Semiautomatic = 2,
    Intervened = 3,
    Manual = 4,
    Service = 5,
    TeachIn = 6
);
tags!(
    write_status,
    read_status,
    ActionStatus,
    Waiting = 0,
    Initializing = 1,
    Running = 2,
    Paused = 3,
    Retriable = 4,
    Finished = 5,
    Failed = 6
);
tags!(
    write_request_type,
    read_request_type,
    ZoneRequestType,
    Access = 0,
    Replanning = 1
);
tags!(
    write_request_status,
    read_request_status,
    RequestStatus,
    Requested = 0,
    Queued = 1,
    Granted = 2,
    Revoked = 3,
    Expired = 4
);
tags!(
    write_level,
    read_level,
    ErrorLevel,
    Warning = 0,
    Urgent = 1,
    Critical = 2,
    Fatal = 3
);
tags!(
    write_door,
    read_door,
    DoorPhase,
    Closed = 0,
    Opening = 1,
    Open = 2,
    Closing = 3
);
tags!(
    write_blocking,
    read_blocking,
    Blocking,
    None = 0,
    Soft = 1,
    Single = 2,
    Hard = 3
);
tags!(
    write_loss,
    read_loss,
    LossBehavior,
    Stop = 0,
    Continue = 1,
    Evacuate = 2
);
tags!(
    write_directed,
    read_directed,
    Directed,
    Soft = 0,
    Restricted = 1,
    Strict = 2
);
tags!(
    write_bidirected,
    read_bidirected,
    Bidirected,
    Soft = 0,
    Restricted = 1
);

// The groups of fields of a report, one bit of the mask each.
const ORDER_ID: u32 = 1 << 0;
const UPDATE_ID: u32 = 1 << 1;
const ROUTE: u32 = 1 << 2;
const POSITION: u32 = 1 << 3;
const VELOCITY: u32 = 1 << 4;
/// Driving, field violation, emergency stop, operating mode and charging.
const FLAGS: u32 = 1 << 5;
const CHARGE: u32 = 1 << 6;
const ACTIONS: u32 = 1 << 7;
const INSTANT_ACTIONS: u32 = 1 << 8;
const ZONE_REQUESTS: u32 = 1 << 9;
const ZONE_SETS: u32 = 1 << 10;
const ERRORS: u32 = 1 << 11;
const ALL_GROUPS: u32 = (1 << 12) - 1;

const NO_BEFORE: Bad = "a first report of a robot leaves out a field";

/// The report of each robot written or read last, which the next one is told
/// against.
#[derive(Debug, Default)]
pub struct Previous(BTreeMap<RobotNo, (u32, State)>);

fn write_robot_no(e: &mut Encoder, robot: RobotNo) {
    let (slot, generation) = robot.parts();
    e.u32(slot);
    e.u32(generation);
}

fn read_robot_no(d: &mut Decoder) -> Result<RobotNo, Bad> {
    Ok(RobotNo::new(d.u32()?, d.u32()?))
}

/// A time, as its distance from the time `now`.
fn write_time(e: &mut Encoder, time: Millis, now: Millis) {
    let Millis(time) = time;
    e.signed(time.wrapping_sub(now.0));
}

fn read_time(d: &mut Decoder, now: Millis) -> Result<Millis, Bad> {
    Ok(Millis(now.0.wrapping_add(d.signed()?)))
}

fn same_floats(a: &[f64], b: &[f64]) -> bool {
    a.iter()
        .map(|v| v.to_bits())
        .eq(b.iter().map(|v| v.to_bits()))
}

fn same_position(a: Option<Position>, b: Option<Position>) -> bool {
    match (a, b) {
        (Some(a), Some(b)) => {
            let Position {
                map,
                x,
                y,
                theta,
                localized,
            } = a;
            map == b.map
                && localized == b.localized
                && same_floats(&[x, y, theta], &[b.x, b.y, b.theta])
        }
        (a, b) => a.is_none() && b.is_none(),
    }
}

fn same_velocity(a: Option<Velocity>, b: Option<Velocity>) -> bool {
    match (a, b) {
        (Some(a), Some(b)) => {
            let Velocity { vx, vy, omega } = a;
            same_floats(&[vx, vy, omega], &[b.vx, b.vy, b.omega])
        }
        (a, b) => a.is_none() && b.is_none(),
    }
}

/// Numbers of which only those that differ from `before` are written, after a byte
/// that says which they are.
fn write_changed(e: &mut Encoder, values: [f64; 3], before: Option<[f64; 3]>) {
    let changed: [bool; 3] =
        std::array::from_fn(|i| before.is_none_or(|b| b[i].to_bits() != values[i].to_bits()));
    e.byte((0..3).map(|i| u8::from(changed[i]) << i).sum());
    for (value, changed) in values.into_iter().zip(changed) {
        if changed {
            e.float(value);
        }
    }
}

fn read_changed(d: &mut Decoder, before: Option<[f64; 3]>) -> Result<[f64; 3], Bad> {
    let mask = d.byte()?;
    if mask >= 8 {
        return Err("an unknown number is marked as changed");
    }
    let mut values = [0.0; 3];
    for (i, value) in values.iter_mut().enumerate() {
        *value = if mask & (1 << i) != 0 {
            d.float()?
        } else {
            before.ok_or(NO_BEFORE)?[i]
        };
    }
    Ok(values)
}

fn write_position(e: &mut Encoder, position: Position, before: Option<Position>) {
    let Position {
        map,
        x,
        y,
        theta,
        localized,
    } = position;
    e.index(map);
    e.bool(localized);
    write_changed(e, [x, y, theta], before.map(|b| [b.x, b.y, b.theta]));
}

fn read_position(d: &mut Decoder, before: Option<Position>) -> Result<Position, Bad> {
    let map = d.index()?;
    let localized = d.bool()?;
    let [x, y, theta] = read_changed(d, before.map(|b| [b.x, b.y, b.theta]))?;
    Ok(Position {
        map,
        x,
        y,
        theta,
        localized,
    })
}

fn write_velocity(e: &mut Encoder, velocity: Velocity, before: Option<Velocity>) {
    let Velocity { vx, vy, omega } = velocity;
    write_changed(e, [vx, vy, omega], before.map(|b| [b.vx, b.vy, b.omega]));
}

fn read_velocity(d: &mut Decoder, before: Option<Velocity>) -> Result<Velocity, Bad> {
    let [vx, vy, omega] = read_changed(d, before.map(|b| [b.vx, b.vy, b.omega]))?;
    Ok(Velocity { vx, vy, omega })
}

fn write_route(e: &mut Encoder, route: &Route) {
    let (last_node, ahead, released) = route.parts();
    e.option(last_node, |e, last| {
        let LastNode { node, step } = last;
        e.index(node);
        e.u32(step);
    });
    e.list(ahead, |e, entry| {
        let Ahead { edge, node } = *entry;
        e.index(edge);
        e.index(node);
    });
    e.u32(released);
}

fn read_route(d: &mut Decoder) -> Result<Route, Bad> {
    let last_node = d.option(|d| {
        Ok(LastNode {
            node: d.index()?,
            step: d.u32()?,
        })
    })?;
    let ahead = d.list(|d| {
        Ok(Ahead {
            edge: d.index()?,
            node: d.index()?,
        })
    })?;
    Route::new(last_node, ahead, d.u32()?).map_err(|_| "a route whose parts do not fit")
}

fn write_action(e: &mut Encoder, action: &ActionState) {
    let ActionState { id, kind, status } = action;
    e.text(id.as_str());
    e.option(kind.as_ref(), |e, kind| e.text(kind.as_str()));
    write_status(e, *status);
}

fn read_action(d: &mut Decoder) -> Result<ActionState, Bad> {
    Ok(ActionState {
        id: ActionId::new(d.text()?),
        kind: d.option(|d| d.text().map(ActionType::new))?,
        status: read_status(d)?,
    })
}

fn write_zone_request(e: &mut Encoder, request: &ZoneRequest) {
    let ZoneRequest {
        id,
        kind,
        zone,
        zone_set,
        status,
    } = request;
    e.text(id.as_str());
    write_request_type(e, *kind);
    e.text(zone.as_str());
    e.text(zone_set.as_str());
    write_request_status(e, *status);
}

fn read_zone_request(d: &mut Decoder) -> Result<ZoneRequest, Bad> {
    Ok(ZoneRequest {
        id: RequestId::new(d.text()?),
        kind: read_request_type(d)?,
        zone: ZoneId::new(d.text()?),
        zone_set: ZoneSetId::new(d.text()?),
        status: read_request_status(d)?,
    })
}

fn write_zone_set_info(e: &mut Encoder, info: &ZoneSetInfo) {
    let ZoneSetInfo { id, map, enabled } = info;
    e.text(id.as_str());
    e.index(*map);
    e.bool(*enabled);
}

fn read_zone_set_info(d: &mut Decoder) -> Result<ZoneSetInfo, Bad> {
    Ok(ZoneSetInfo {
        id: ZoneSetId::new(d.text()?),
        map: d.index()?,
        enabled: d.bool()?,
    })
}

fn read_arc(d: &mut Decoder) -> Result<Arc<str>, Bad> {
    d.text().map(Arc::from)
}

fn write_error(e: &mut Encoder, error: &Error) {
    let Error {
        kind,
        level,
        references,
        description,
        hint,
    } = error;
    match kind {
        ErrorKind::Predefined(kind) => {
            e.byte(0);
            e.text(kind.as_str());
        }
        ErrorKind::Other(kind) => {
            e.byte(1);
            e.text(kind);
        }
    }
    write_level(e, *level);
    e.list(references, |e, (key, value)| {
        e.text(key);
        e.text(value);
    });
    e.option(description.as_deref(), Encoder::text);
    e.option(hint.as_deref(), Encoder::text);
}

fn read_error(d: &mut Decoder) -> Result<Error, Bad> {
    let kind = match d.byte()? {
        0 => ErrorKind::Predefined(
            ErrorType::from_text(d.text()?).ok_or("an unknown predefined error type")?,
        ),
        1 => ErrorKind::Other(read_arc(d)?),
        _ => return Err("an unknown kind of error type"),
    };
    Ok(Error {
        kind,
        level: read_level(d)?,
        references: d.list(|d| Ok((read_arc(d)?, read_arc(d)?)))?,
        description: d.option(read_arc)?,
        hint: d.option(read_arc)?,
    })
}

/// A report of a robot, in the cycle at the time `now`.
pub fn write_report(e: &mut Encoder, report: &FromRobot, now: Millis, previous: &mut Previous) {
    let FromRobot { header, state } = report;
    let Header { robot, seq, time } = *header;
    write_robot_no(e, robot);
    let before = previous.0.get(&robot);
    let seq_before = before.map_or(0, |(seq, _)| *seq);
    e.signed(i64::from(seq.wrapping_sub(seq_before) as i32));
    write_time(e, time, now);

    let State {
        order_id,
        update_id,
        route,
        position,
        velocity,
        driving,
        field_violation,
        emergency_stop,
        operating_mode,
        charge,
        charging,
        actions,
        instant_actions,
        zone_requests,
        zone_sets,
        errors,
    } = state;
    let before = before.map(|(_, state)| state);
    let mask = match before {
        None => ALL_GROUPS,
        Some(b) => [
            (ORDER_ID, b.order_id != *order_id),
            (UPDATE_ID, b.update_id != *update_id),
            (ROUTE, b.route != *route),
            (POSITION, !same_position(b.position, *position)),
            (VELOCITY, !same_velocity(b.velocity, *velocity)),
            (
                FLAGS,
                b.driving != *driving
                    || b.field_violation != *field_violation
                    || b.emergency_stop != *emergency_stop
                    || b.operating_mode != *operating_mode
                    || b.charging != *charging,
            ),
            (CHARGE, b.charge.to_bits() != charge.to_bits()),
            (ACTIONS, b.actions != *actions),
            (INSTANT_ACTIONS, b.instant_actions != *instant_actions),
            (ZONE_REQUESTS, b.zone_requests != *zone_requests),
            (ZONE_SETS, b.zone_sets != *zone_sets),
            (ERRORS, b.errors != *errors),
        ]
        .into_iter()
        .filter(|&(_, changed)| changed)
        .map(|(bit, _)| bit)
        .sum(),
    };
    e.u32(mask);
    if mask & ORDER_ID != 0 {
        e.option(order_id.as_ref(), |e, id| e.text(id.as_str()));
    }
    if mask & UPDATE_ID != 0 {
        e.u32(*update_id);
    }
    if mask & ROUTE != 0 {
        write_route(e, route);
    }
    if mask & POSITION != 0 {
        let before = before.and_then(|b| b.position);
        e.option(*position, |e, p| write_position(e, p, before));
    }
    if mask & VELOCITY != 0 {
        let before = before.and_then(|b| b.velocity);
        e.option(*velocity, |e, v| write_velocity(e, v, before));
    }
    if mask & FLAGS != 0 {
        e.bool(*driving);
        e.bool(*field_violation);
        write_emergency(e, *emergency_stop);
        write_mode(e, *operating_mode);
        e.bool(*charging);
    }
    if mask & CHARGE != 0 {
        e.float(*charge);
    }
    if mask & ACTIONS != 0 {
        e.list(actions, write_action);
    }
    if mask & INSTANT_ACTIONS != 0 {
        e.list(instant_actions, write_action);
    }
    if mask & ZONE_REQUESTS != 0 {
        e.option(zone_requests.as_deref(), |e, requests| {
            e.list(requests, write_zone_request);
        });
    }
    if mask & ZONE_SETS != 0 {
        e.list(zone_sets, write_zone_set_info);
    }
    if mask & ERRORS != 0 {
        e.list(errors, write_error);
    }
    if mask != 0 || before.is_none() {
        previous.0.insert(robot, (seq, state.clone()));
    } else if let Some((kept, _)) = previous.0.get_mut(&robot) {
        *kept = seq;
    }
}

pub fn read_report(
    d: &mut Decoder,
    now: Millis,
    previous: &mut Previous,
) -> Result<FromRobot, Bad> {
    let robot = read_robot_no(d)?;
    let before = previous.0.get(&robot);
    let seq_before = before.map_or(0, |(seq, _)| *seq);
    let step = i32::try_from(d.signed()?).map_err(|_| "a sequence number out of range")?;
    let seq = seq_before.wrapping_add(step as u32);
    let time = read_time(d, now)?;
    let before = before.map(|(_, state)| state);
    let mask = d.u32()?;
    if mask > ALL_GROUPS {
        return Err("an unknown field is marked as changed");
    }
    /// The value read, if the mask says it changed, else the one before.
    macro_rules! field {
        ($bit:ident, $name:ident, $read:expr) => {
            if mask & $bit != 0 {
                $read
            } else {
                before.ok_or(NO_BEFORE)?.$name.clone()
            }
        };
    }
    let order_id = field!(
        ORDER_ID,
        order_id,
        d.option(|d| d.text().map(OrderId::new))?
    );
    let update_id = field!(UPDATE_ID, update_id, d.u32()?);
    let route = field!(ROUTE, route, read_route(d)?);
    let position = field!(POSITION, position, {
        let before = before.and_then(|b| b.position);
        d.option(|d| read_position(d, before))?
    });
    let velocity = field!(VELOCITY, velocity, {
        let before = before.and_then(|b| b.velocity);
        d.option(|d| read_velocity(d, before))?
    });
    let (driving, field_violation, emergency_stop, operating_mode, charging) = if mask & FLAGS != 0
    {
        (
            d.bool()?,
            d.bool()?,
            read_emergency(d)?,
            read_mode(d)?,
            d.bool()?,
        )
    } else {
        let b = before.ok_or(NO_BEFORE)?;
        (
            b.driving,
            b.field_violation,
            b.emergency_stop,
            b.operating_mode,
            b.charging,
        )
    };
    let charge = field!(CHARGE, charge, d.float()?);
    let actions = field!(ACTIONS, actions, d.list(read_action)?);
    let instant_actions = field!(INSTANT_ACTIONS, instant_actions, d.list(read_action)?);
    let zone_requests = field!(
        ZONE_REQUESTS,
        zone_requests,
        d.option(|d| d.list(read_zone_request))?
    );
    let zone_sets = field!(ZONE_SETS, zone_sets, d.list(read_zone_set_info)?);
    let errors = field!(ERRORS, errors, d.list(read_error)?);
    let state = State {
        order_id,
        update_id,
        route,
        position,
        velocity,
        driving,
        field_violation,
        emergency_stop,
        operating_mode,
        charge,
        charging,
        actions,
        instant_actions,
        zone_requests,
        zone_sets,
        errors,
    };
    if mask != 0 || before.is_none() {
        previous.0.insert(robot, (seq, state.clone()));
    } else if let Some((kept, _)) = previous.0.get_mut(&robot) {
        *kept = seq;
    }
    Ok(FromRobot {
        header: Header { robot, seq, time },
        state,
    })
}

pub fn write_lift(e: &mut Encoder, lift: &FromLift, now: Millis) {
    let FromLift {
        header,
        map,
        car_door,
        door,
        occupant,
    } = *lift;
    let DeviceHeader { device, seq, time } = header;
    let LiftNo(device) = device;
    e.u32(device);
    e.u32(seq);
    write_time(e, time, now);
    e.index(map);
    e.u32(car_door);
    write_door(e, door);
    e.option(occupant, write_robot_no);
}

pub fn read_lift(d: &mut Decoder, now: Millis) -> Result<FromLift, Bad> {
    Ok(FromLift {
        header: DeviceHeader {
            device: LiftNo(d.u32()?),
            seq: d.u32()?,
            time: read_time(d, now)?,
        },
        map: d.index()?,
        car_door: d.u32()?,
        door: read_door(d)?,
        occupant: d.option(read_robot_no)?,
    })
}

pub fn write_job(e: &mut Encoder, job: &Job, now: Millis) {
    let Job {
        id,
        pickup,
        drop,
        created,
    } = *job;
    e.unsigned(id);
    e.index(pickup);
    e.index(drop);
    write_time(e, created, now);
}

pub fn read_job(d: &mut Decoder, now: Millis) -> Result<Job, Bad> {
    Ok(Job {
        id: d.unsigned()?,
        pickup: d.index()?,
        drop: d.index()?,
        created: read_time(d, now)?,
    })
}

fn write_zone_action(e: &mut Encoder, action: &ZoneAction) {
    let ZoneAction {
        kind,
        descriptor,
        blocking,
        parameters,
        retriable,
    } = action;
    e.text(kind.as_str());
    e.option(descriptor.as_deref(), Encoder::text);
    write_blocking(e, *blocking);
    e.option(parameters.as_deref(), |e, parameters| {
        e.list(parameters, |e, (key, value)| {
            e.text(key);
            // (a value is any JSON value, so it is written as JSON text)
            e.text(&value.to_string());
        });
    });
    e.option(*retriable, Encoder::bool);
}

fn read_zone_action(d: &mut Decoder) -> Result<ZoneAction, Bad> {
    Ok(ZoneAction {
        kind: ActionType::new(d.text()?),
        descriptor: d.option(read_arc)?,
        blocking: read_blocking(d)?,
        parameters: d.option(|d| {
            d.list(|d| {
                let key = read_arc(d)?;
                let value = serde_json::from_str(d.text()?)
                    .map_err(|_| "a value of a zone action is not JSON")?;
                Ok((key, value))
            })
        })?,
        retriable: d.option(Decoder::bool)?,
    })
}

fn write_zone(e: &mut Encoder, zone: &Zone) {
    let Zone {
        id,
        descriptor,
        vertices,
        kind,
    } = zone;
    e.text(id.as_str());
    e.option(descriptor.as_deref(), Encoder::text);
    e.list(vertices, |e, &(x, y)| {
        e.float(x);
        e.float(y);
    });
    match kind {
        ZoneKind::Blocked => e.byte(0),
        ZoneKind::LineGuided => e.byte(1),
        ZoneKind::Release { loss } => {
            e.byte(2);
            e.option(*loss, write_loss);
        }
        ZoneKind::CoordinatedReplanning => e.byte(3),
        ZoneKind::SpeedLimit { maximum_speed } => {
            e.byte(4);
            e.float(*maximum_speed);
        }
        ZoneKind::Action {
            entry,
            during,
            exit,
        } => {
            e.byte(5);
            for actions in [entry, during, exit] {
                e.list(actions, write_zone_action);
            }
        }
        ZoneKind::Priority { factor } => {
            e.byte(6);
            e.float(*factor);
        }
        ZoneKind::Penalty { factor } => {
            e.byte(7);
            e.float(*factor);
        }
        ZoneKind::Directed {
            direction,
            limitation,
        } => {
            e.byte(8);
            e.float(*direction);
            write_directed(e, *limitation);
        }
        ZoneKind::Bidirected {
            direction,
            limitation,
        } => {
            e.byte(9);
            e.float(*direction);
            write_bidirected(e, *limitation);
        }
    }
}

fn read_zone(d: &mut Decoder) -> Result<Zone, Bad> {
    let id = ZoneId::new(d.text()?);
    let descriptor = d.option(read_arc)?;
    let vertices = d.list(|d| Ok((d.float()?, d.float()?)))?;
    let kind = match d.byte()? {
        0 => ZoneKind::Blocked,
        1 => ZoneKind::LineGuided,
        2 => ZoneKind::Release {
            loss: d.option(read_loss)?,
        },
        3 => ZoneKind::CoordinatedReplanning,
        4 => ZoneKind::SpeedLimit {
            maximum_speed: d.float()?,
        },
        5 => ZoneKind::Action {
            entry: d.list(read_zone_action)?,
            during: d.list(read_zone_action)?,
            exit: d.list(read_zone_action)?,
        },
        6 => ZoneKind::Priority { factor: d.float()? },
        7 => ZoneKind::Penalty { factor: d.float()? },
        8 => ZoneKind::Directed {
            direction: d.float()?,
            limitation: read_directed(d)?,
        },
        9 => ZoneKind::Bidirected {
            direction: d.float()?,
            limitation: read_bidirected(d)?,
        },
        _ => return Err("an unknown zone type"),
    };
    Ok(Zone {
        id,
        descriptor,
        vertices,
        kind,
    })
}

pub fn write_zone_set(e: &mut Encoder, set: &ZoneSet) {
    let ZoneSet {
        id,
        map,
        descriptor,
        zones,
    } = set;
    e.text(id.as_str());
    e.index(*map);
    e.option(descriptor.as_deref(), Encoder::text);
    e.list(zones, write_zone);
}

pub fn read_zone_set(d: &mut Decoder) -> Result<ZoneSet, Bad> {
    Ok(ZoneSet {
        id: ZoneSetId::new(d.text()?),
        map: d.index()?,
        descriptor: d.option(read_arc)?,
        zones: d.list(read_zone)?,
    })
}

#[cfg(test)]
mod tests;
