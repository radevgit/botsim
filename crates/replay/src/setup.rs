//! Everything a controller is made from, as data: the first part of a recording.
//!
//! A [`Setup`] holds the site and the fleet ([`Config`]) with the layout as the
//! text of its LIF file, which traffic rules and dispatcher ([`Rules`],
//! [`Dispatch`]), and the set-up calls made on the controller, in their order
//! ([`Call`]). [`controller`] makes the controller it describes; a run that is
//! recorded makes its controller the same way. No other file is needed.

use std::sync::Arc;

use botsim_baseline::baseline_policy::BaselinePolicy;
use botsim_dispatcher::ChargePolicy;
use botsim_dispatcher::baseline::BaselineDispatcher;
use botsim_model::graph::Graph;
use botsim_model::robot::RobotId;
use botsim_model::spec::{Kinematics, Shape};
use botsim_model::time::Millis;
use botsim_model::zones::Zones;
use botsim_server::conflicts::{ConflictInput, NodeConflictInput, NodeNodeConflictInput};
use botsim_server::controller::Controller;
use botsim_server::cycle::{Config, FleetRobot};
use botsim_server::doors::Door;
use botsim_server::lifts::{Lift, LiftStop};
use botsim_server::policy::TrafficPolicy;
use botsim_server::traffic_policy::TrafficAdapter;
use botsim_space::{MapOptions, TypeRef};
use vdma_lif::Lif;

use crate::Error;
use crate::codec::{Bad, Decoder, Encoder};
use crate::messages::{read_zone_set, write_zone_set};

/// What a controller is made from.
#[derive(Debug, Clone)]
pub struct Setup {
    /// The layout as the text of a LIF file; [`Setup::config`] has its graph.
    pub lif: String,
    /// The site and the fleet.
    pub config: Config,
    /// The traffic rules.
    pub rules: Rules,
    pub dispatch: Dispatch,
    /// The set-up calls on the controller, in their order.
    pub calls: Vec<Call>,
}

/// The traffic rules, of those botsim has.
#[derive(Debug, Clone)]
pub enum Rules {
    /// The baseline's ([`BaselinePolicy`]).
    Baseline,
    /// The traffic module's ([`TrafficAdapter`]), which computes the conflict map
    /// of the layout for these robot types.
    Traffic {
        types: Vec<TypeRef>,
        options: MapOptions,
    },
}

/// The dispatcher, of those botsim has.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Dispatch {
    /// The baseline's ([`BaselineDispatcher`]).
    #[default]
    Baseline,
}

/// A set-up call on a controller.
#[derive(Debug, Clone, PartialEq)]
pub enum Call {
    /// [`Controller::set_load_weight`]
    LoadWeight(f64),
    /// [`Controller::set_wait_weight`]
    WaitWeight(f64),
    /// [`Controller::set_charging`]
    Charging(ChargePolicy),
    /// [`TrafficPolicy::conflicts`]
    Conflicts(Vec<ConflictInput>),
    /// [`TrafficPolicy::node_conflicts`]
    NodeConflicts(Vec<NodeConflictInput>),
    /// [`TrafficPolicy::node_node_conflicts`]
    NodeNodeConflicts(Vec<NodeNodeConflictInput>),
}

/// Make the set-up calls `calls` on `controller`, in their order.
pub fn apply<P: TrafficPolicy>(controller: &mut Controller<P>, calls: &[Call]) {
    for call in calls {
        match call {
            Call::LoadWeight(weight) => controller.set_load_weight(*weight),
            Call::WaitWeight(weight) => controller.set_wait_weight(*weight),
            Call::Charging(policy) => {
                controller.set_charging(*policy);
            }
            Call::Conflicts(list) => controller.policy_mut().conflicts(list),
            Call::NodeConflicts(list) => controller.policy_mut().node_conflicts(list),
            Call::NodeNodeConflicts(list) => controller.policy_mut().node_node_conflicts(list),
        }
    }
}

/// The controller that `setup` describes, ready for its first cycle.
///
/// The error says why the traffic rules cannot be made (no conflict map of the
/// layout).
pub fn controller(setup: &Setup) -> Result<Controller<Box<dyn TrafficPolicy>>, Error> {
    let Setup {
        lif,
        config,
        rules,
        dispatch,
        calls,
    } = setup;
    let policy: Box<dyn TrafficPolicy> = match rules {
        Rules::Baseline => Box::new(BaselinePolicy::new(&config.graph)),
        Rules::Traffic { types, options } => {
            let lif = read_lif(lif)?;
            let adapter = TrafficAdapter::new(&lif, types, options).map_err(Error::Setup)?;
            Box::new(adapter)
        }
    };
    let dispatcher = match dispatch {
        Dispatch::Baseline => Box::new(BaselineDispatcher::new()),
    };
    let mut controller = Controller::with_dispatcher(config, policy, dispatcher);
    apply(&mut controller, calls);
    Ok(controller)
}

fn read_lif(text: &str) -> Result<Lif, Error> {
    serde_json::from_str(text).map_err(|e| Error::Setup(format!("the layout is not LIF: {e}")))
}

fn graph_of(text: &str) -> Result<Graph, Error> {
    Graph::from_lif(&read_lif(text)?)
        .map_err(|e| Error::Setup(format!("the layout cannot be used: {e}")))
}

// ---------------------------------------------------------------------------
// writing

tags!(
    write_kinematics,
    read_kinematics,
    Kinematics,
    Differential = 0,
    Omnidirectional = 1,
    ThreeWheel = 2
);

fn write_shape(e: &mut Encoder, shape: Shape) {
    let values = match shape {
        Shape::Rect {
            cx,
            cy,
            half_length,
            half_width,
        } => {
            e.byte(0);
            vec![cx, cy, half_length, half_width]
        }
        Shape::Circle { cx, cy, radius } => {
            e.byte(1);
            vec![cx, cy, radius]
        }
    };
    values.into_iter().for_each(|v| e.float(v));
}

fn write_robot(e: &mut Encoder, robot: &FleetRobot) {
    let FleetRobot {
        id,
        series,
        kinematics,
        shape,
    } = robot;
    let RobotId {
        manufacturer,
        serial_number,
    } = id;
    e.text(manufacturer);
    e.text(serial_number);
    e.text(series);
    write_kinematics(e, *kinematics);
    write_shape(e, *shape);
}

fn write_zones(e: &mut Encoder, zones: &Zones) -> Result<(), Error> {
    let Zones {
        map,
        zone_set_id,
        zones,
    } = zones;
    e.index(*map);
    e.text(zone_set_id);
    // (in VDA 5050's own form, which the zones are kept in)
    let json = serde_json::to_string(zones)
        .map_err(|e| Error::Setup(format!("the zones cannot be written: {e}")))?;
    e.text(&json);
    Ok(())
}

fn write_lift(e: &mut Encoder, lift: &Lift) {
    let Lift { id, doors, stops } = lift;
    e.text(id);
    e.list(doors, |e, door| e.text(door));
    e.list(stops, |e, stop| {
        let LiftStop {
            node,
            door,
            waiting,
        } = *stop;
        e.index(node);
        e.index(door);
        e.option(waiting, Encoder::index);
    });
}

fn write_door(e: &mut Encoder, door: &Door) {
    let Door { id, edges } = door;
    e.text(id);
    e.list(edges, |e, &edge| e.index(edge));
}

fn write_type(e: &mut Encoder, kind: &TypeRef) {
    let TypeRef {
        name,
        manufacturer,
        series,
        kinematics,
        length,
        width,
        field_margin,
        field_from_factsheet,
        shape,
    } = kind;
    e.text(name);
    e.text(manufacturer);
    e.text(series);
    write_kinematics(e, *kinematics);
    e.float(*length);
    e.float(*width);
    e.float(*field_margin);
    e.bool(*field_from_factsheet);
    write_shape(e, *shape);
}

fn write_rules(e: &mut Encoder, rules: &Rules) {
    match rules {
        Rules::Baseline => e.byte(0),
        Rules::Traffic { types, options } => {
            e.byte(1);
            e.list(types, write_type);
            let MapOptions {
                sample_step,
                min_overlap,
                clearance,
                protective_field,
            } = *options;
            e.float(sample_step);
            e.float(min_overlap);
            e.float(clearance);
            e.bool(protective_field);
        }
    }
}

fn write_pairs(e: &mut Encoder, pairs: &[(String, String)]) {
    e.list(pairs, |e, (a, b)| {
        e.text(a);
        e.text(b);
    });
}

fn write_call(e: &mut Encoder, call: &Call) {
    match call {
        Call::LoadWeight(weight) => {
            e.byte(0);
            e.float(*weight);
        }
        Call::WaitWeight(weight) => {
            e.byte(1);
            e.float(*weight);
        }
        Call::Charging(policy) => {
            e.byte(2);
            let ChargePolicy {
                low,
                high,
                critical,
            } = *policy;
            e.float(low);
            e.float(high);
            e.float(critical);
        }
        Call::Conflicts(list) => {
            e.byte(3);
            e.list(list, |e, input| {
                let ConflictInput {
                    type_a,
                    type_b,
                    edges,
                } = input;
                e.text(type_a);
                e.text(type_b);
                write_pairs(e, edges);
            });
        }
        Call::NodeConflicts(list) => {
            e.byte(4);
            e.list(list, |e, input| {
                let NodeConflictInput {
                    type_node,
                    type_edge,
                    pairs,
                } = input;
                e.text(type_node);
                e.text(type_edge);
                write_pairs(e, pairs);
            });
        }
        Call::NodeNodeConflicts(list) => {
            e.byte(5);
            e.list(list, |e, input| {
                let NodeNodeConflictInput {
                    type_a,
                    type_b,
                    pairs,
                } = input;
                e.text(type_a);
                e.text(type_b);
                write_pairs(e, pairs);
            });
        }
    }
}

/// Write `setup`. An error if its graph is not the one of its LIF text, since the
/// file holds the text only.
pub(crate) fn write(e: &mut Encoder, setup: &Setup) -> Result<(), Error> {
    let Setup {
        lif,
        config,
        rules,
        dispatch,
        calls,
    } = setup;
    let Config {
        graph,
        robots,
        zones,
        zone_set,
        lifts,
        doors,
        start,
    } = config;
    if graph_of(lif)? != *graph {
        return Err(Error::Setup(
            "the layout text does not give the graph the controller has".into(),
        ));
    }
    e.text(lif);
    e.list(robots, write_robot);
    match zones {
        Some(zones) => {
            e.bool(true);
            write_zones(e, zones)?;
        }
        None => e.bool(false),
    }
    e.option(zone_set.as_deref(), write_zone_set);
    e.list(lifts, write_lift);
    e.list(doors, write_door);
    let Millis(start) = *start;
    e.signed(start);
    write_rules(e, rules);
    match dispatch {
        Dispatch::Baseline => e.byte(0),
    }
    e.list(calls, write_call);
    Ok(())
}

// ---------------------------------------------------------------------------
// reading

/// A size of a robot, or a distance, that the geometry can work with: finite and at
/// most `limit` either way.
fn within(value: f64, limit: f64) -> Result<f64, Bad> {
    if value.is_finite() && value.abs() <= limit {
        Ok(value)
    } else {
        Err("a size or distance of the setup is out of range")
    }
}

/// The largest length (m) of a robot's outline or of a conflict map's setting.
const LARGEST_SIZE: f64 = 1000.0;

/// The finest sample step (m) of a conflict map: a finer one would take memory
/// without bound.
const FINEST_STEP: f64 = 0.001;

fn read_size(d: &mut Decoder) -> Result<f64, Bad> {
    within(d.float()?, LARGEST_SIZE)
}

fn read_shape(d: &mut Decoder) -> Result<Shape, Bad> {
    match d.byte()? {
        0 => Ok(Shape::Rect {
            cx: read_size(d)?,
            cy: read_size(d)?,
            half_length: read_size(d)?,
            half_width: read_size(d)?,
        }),
        1 => Ok(Shape::Circle {
            cx: read_size(d)?,
            cy: read_size(d)?,
            radius: read_size(d)?,
        }),
        _ => Err("an unknown body outline"),
    }
}

fn read_robot(d: &mut Decoder) -> Result<FleetRobot, Bad> {
    Ok(FleetRobot {
        id: RobotId {
            manufacturer: d.string()?,
            serial_number: d.string()?,
        },
        series: d.string()?,
        kinematics: read_kinematics(d)?,
        shape: read_shape(d)?,
    })
}

fn read_zones(d: &mut Decoder, graph: &Graph) -> Result<Zones, Bad> {
    let map = d.index()?;
    if map >= graph.maps.len() {
        return Err("the zones are for a map the layout does not have");
    }
    let zone_set_id = d.string()?;
    let zones: Vec<vda5050::zone_set::Zone> =
        serde_json::from_str(d.text()?).map_err(|_| "the zones are not VDA 5050 zones")?;
    for zone in &zones {
        let corners: Vec<(f64, f64)> = zone.vertices.iter().map(|v| (v.x, v.y)).collect();
        botsim_model::zones::check_zone(&zone.zone_id, &corners, None)
            .map_err(|_| "a zone is no usable polygon")?;
    }
    Ok(Zones {
        map,
        zone_set_id,
        zones,
    })
}

fn read_lift(d: &mut Decoder, graph: &Graph) -> Result<Lift, Bad> {
    let node = |n: usize| {
        if n < graph.nodes.len() {
            Ok(n)
        } else {
            Err("a lift stop names a node the layout does not have")
        }
    };
    Ok(Lift {
        id: d.string()?,
        doors: d.list(Decoder::string)?,
        stops: d.list(|d| {
            Ok(LiftStop {
                node: node(d.index()?)?,
                door: d.index()?,
                waiting: d.option(Decoder::index)?.map(node).transpose()?,
            })
        })?,
    })
}

fn read_door(d: &mut Decoder) -> Result<Door, Bad> {
    Ok(Door {
        id: d.string()?,
        edges: d.list(Decoder::index)?,
    })
}

fn read_type(d: &mut Decoder) -> Result<TypeRef, Bad> {
    Ok(TypeRef {
        name: d.string()?,
        manufacturer: d.string()?,
        series: d.string()?,
        kinematics: read_kinematics(d)?,
        length: read_size(d)?,
        width: read_size(d)?,
        field_margin: read_size(d)?,
        field_from_factsheet: d.bool()?,
        shape: read_shape(d)?,
    })
}

fn read_rules(d: &mut Decoder) -> Result<Rules, Bad> {
    match d.byte()? {
        0 => Ok(Rules::Baseline),
        1 => {
            let types = d.list(read_type)?;
            let options = MapOptions {
                sample_step: read_size(d)?,
                min_overlap: read_size(d)?,
                clearance: read_size(d)?,
                protective_field: d.bool()?,
            };
            if options.sample_step < FINEST_STEP {
                return Err("the sample step of the conflict map is too fine");
            }
            Ok(Rules::Traffic { types, options })
        }
        _ => Err("unknown traffic rules"),
    }
}

fn read_pairs(d: &mut Decoder) -> Result<Vec<(String, String)>, Bad> {
    d.list(|d| Ok((d.string()?, d.string()?)))
}

fn read_call(d: &mut Decoder) -> Result<Call, Bad> {
    Ok(match d.byte()? {
        0 => Call::LoadWeight(d.float()?),
        1 => Call::WaitWeight(d.float()?),
        2 => Call::Charging(ChargePolicy {
            low: d.float()?,
            high: d.float()?,
            critical: d.float()?,
        }),
        3 => Call::Conflicts(d.list(|d| {
            Ok(ConflictInput {
                type_a: d.string()?,
                type_b: d.string()?,
                edges: read_pairs(d)?,
            })
        })?),
        4 => Call::NodeConflicts(d.list(|d| {
            Ok(NodeConflictInput {
                type_node: d.string()?,
                type_edge: d.string()?,
                pairs: read_pairs(d)?,
            })
        })?),
        5 => Call::NodeNodeConflicts(d.list(|d| {
            Ok(NodeNodeConflictInput {
                type_a: d.string()?,
                type_b: d.string()?,
                pairs: read_pairs(d)?,
            })
        })?),
        _ => return Err("an unknown set-up call"),
    })
}

pub(crate) fn read(d: &mut Decoder) -> Result<Setup, Error> {
    let lif = d.string()?;
    let graph = graph_of(&lif)?;
    let robots = d.list(read_robot)?;
    let zones = d.option(|d| read_zones(d, &graph))?;
    let zone_set = d.option(read_zone_set)?.map(Arc::new);
    let lifts = d.list(|d| read_lift(d, &graph))?;
    let doors = d.list(read_door)?;
    let start = Millis(d.signed()?);
    let rules = read_rules(d)?;
    let dispatch = match d.byte()? {
        0 => Dispatch::Baseline,
        _ => return Err(Error::from("an unknown dispatcher")),
    };
    let calls = d.list(read_call)?;
    Ok(Setup {
        lif,
        config: Config {
            graph,
            robots,
            zones,
            zone_set,
            lifts,
            doors,
            start,
        },
        rules,
        dispatch,
        calls,
    })
}
