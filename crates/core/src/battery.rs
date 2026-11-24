//! The battery of a robot (charging brief, step 1): how much energy it holds and what
//! drives, working and standing cost.
//!
//! A robot has no battery model unless one is set ([`crate::robot::Robot::set_battery`]);
//! it then reports 100 % for ever, as before. With a model the charge falls while the
//! robot drives (energy per metre), works at a station (power) and stands (power),
//! and the state reports `powerSupply.stateOfCharge`.

use std::fmt;

/// What a robot type draws and holds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatterySpec {
    /// Capacity (Wh), above 0.
    pub capacity_wh: f64,
    /// Energy used per metre driven (Wh/m), 0 or more.
    pub drive_wh_per_m: f64,
    /// Power while standing, rotating or waiting (W), 0 or more.
    pub idle_w: f64,
    /// Power while the robot works at a node (an action runs) (W), 0 or more.
    pub work_w: f64,
}

/// A value of a battery setting that cannot be used.
#[derive(Debug, Clone, PartialEq)]
pub struct BatteryError {
    pub name: &'static str,
    pub value: f64,
}

impl fmt::Display for BatteryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "battery {} is not usable: {}", self.name, self.value)
    }
}

impl std::error::Error for BatteryError {}

impl BatterySpec {
    /// Check the values (finite; the capacity above 0, the others 0 or more).
    pub fn check(&self) -> Result<(), BatteryError> {
        let bad = |name, value: f64| Err(BatteryError { name, value });
        if !(self.capacity_wh.is_finite() && self.capacity_wh > 0.0) {
            return bad("capacity_wh", self.capacity_wh);
        }
        for (name, v) in [
            ("drive_wh_per_m", self.drive_wh_per_m),
            ("idle_w", self.idle_w),
            ("work_w", self.work_w),
        ] {
            if !(v.is_finite() && v >= 0.0) {
                return bad(name, v);
            }
        }
        Ok(())
    }
}

/// A battery with its charge.
#[derive(Debug, Clone, PartialEq)]
pub struct Battery {
    spec: BatterySpec,
    charge_wh: f64,
}

impl Battery {
    /// A battery that holds `start_percent` of its capacity (clamped to 0..=100).
    pub fn new(spec: BatterySpec, start_percent: f64) -> Result<Battery, BatteryError> {
        spec.check()?;
        let p = if start_percent.is_finite() {
            start_percent.clamp(0.0, 100.0)
        } else {
            100.0
        };
        Ok(Battery {
            spec,
            charge_wh: spec.capacity_wh * p / 100.0,
        })
    }

    pub fn spec(&self) -> &BatterySpec {
        &self.spec
    }

    /// The charge in percent of the capacity.
    pub fn percent(&self) -> f64 {
        100.0 * self.charge_wh / self.spec.capacity_wh
    }

    /// The charge in Wh.
    pub fn charge_wh(&self) -> f64 {
        self.charge_wh
    }

    pub fn is_empty(&self) -> bool {
        self.charge_wh <= 0.0
    }

    pub fn is_full(&self) -> bool {
        self.charge_wh >= self.spec.capacity_wh
    }

    /// Charge for `dt` seconds with `power_w` watts (no losses, no taper), up to the
    /// capacity.
    pub fn charge(&mut self, dt: f64, power_w: f64) {
        self.charge_wh = (self.charge_wh + power_w * dt / 3600.0).min(self.spec.capacity_wh);
    }

    /// Use the energy of one step: `dt` seconds, `moved` metres driven, `working` if
    /// an action runs at a node. Driving costs energy per metre; otherwise the robot
    /// draws its working or standing power.
    pub fn drain(&mut self, dt: f64, moved: f64, working: bool) {
        let wh = if moved > 0.0 {
            moved * self.spec.drive_wh_per_m
        } else if working {
            self.spec.work_w * dt / 3600.0
        } else {
            self.spec.idle_w * dt / 3600.0
        };
        self.charge_wh = (self.charge_wh - wh).max(0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec() -> BatterySpec {
        BatterySpec {
            capacity_wh: 100.0,
            drive_wh_per_m: 0.5,
            idle_w: 36.0,
            work_w: 72.0,
        }
    }

    #[test]
    fn starts_at_the_given_percent_and_clamps() {
        assert_eq!(Battery::new(spec(), 40.0).unwrap().percent(), 40.0);
        assert_eq!(Battery::new(spec(), 150.0).unwrap().percent(), 100.0);
        assert_eq!(Battery::new(spec(), -3.0).unwrap().percent(), 0.0);
    }

    #[test]
    fn driving_costs_energy_per_metre_standing_and_working_power() {
        let mut b = Battery::new(spec(), 100.0).unwrap();
        b.drain(1.0, 10.0, false); // 10 m at 0.5 Wh/m
        assert!((b.charge_wh() - 95.0).abs() < 1e-9);
        b.drain(100.0, 0.0, false); // 100 s at 36 W = 1 Wh
        assert!((b.charge_wh() - 94.0).abs() < 1e-9);
        b.drain(100.0, 0.0, true); // 100 s at 72 W = 2 Wh
        assert!((b.charge_wh() - 92.0).abs() < 1e-9);
    }

    #[test]
    fn it_does_not_go_below_empty() {
        let mut b = Battery::new(spec(), 1.0).unwrap();
        b.drain(1.0, 100.0, false);
        assert!(b.is_empty());
        assert_eq!(b.percent(), 0.0);
    }

    #[test]
    fn unusable_values_are_refused() {
        let mut s = spec();
        s.capacity_wh = 0.0;
        assert_eq!(Battery::new(s, 50.0).unwrap_err().name, "capacity_wh");
        let mut s = spec();
        s.idle_w = -1.0;
        assert_eq!(Battery::new(s, 50.0).unwrap_err().name, "idle_w");
        let mut s = spec();
        s.drive_wh_per_m = f64::NAN;
        assert_eq!(Battery::new(s, 50.0).unwrap_err().name, "drive_wh_per_m");
    }
}
