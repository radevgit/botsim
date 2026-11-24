//! The timeline of the viewer: where in the run we are, and whether time moves.

/// Speeds to choose from, simulated seconds per real second.
pub const SPEEDS: [f64; 11] =
    [0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0];

/// A position on the timeline `start ..= end` (simulated seconds) that moves with
/// real time when it is playing.
#[derive(Debug, Clone, PartialEq)]
pub struct Playback {
    start: f64,
    end: f64,
    time: f64,
    speed: f64,
    playing: bool,
}

impl Playback {
    /// Paused at `start`, speed 1.
    pub fn new(start: f64, end: f64) -> Playback {
        let end = end.max(start);
        Playback {
            start,
            end,
            time: start,
            speed: 1.0,
            playing: false,
        }
    }

    pub fn start(&self) -> f64 {
        self.start
    }

    pub fn end(&self) -> f64 {
        self.end
    }

    pub fn time(&self) -> f64 {
        self.time
    }

    pub fn speed(&self) -> f64 {
        self.speed
    }

    pub fn is_playing(&self) -> bool {
        self.playing
    }

    pub fn at_end(&self) -> bool {
        self.time >= self.end
    }

    /// Go to a time; outside the run it is clamped.
    pub fn seek(&mut self, time: f64) {
        self.time = if time.is_finite() {
            time.clamp(self.start, self.end)
        } else {
            self.start
        };
    }

    /// Move `seconds` of simulated time (negative: back); clamped to the run.
    pub fn step(&mut self, seconds: f64) {
        self.seek(self.time + seconds);
    }

    /// Go to the start of the run.
    pub fn to_start(&mut self) {
        self.time = self.start;
    }

    /// Go to the end of the run (playing stops there).
    pub fn to_end(&mut self) {
        self.time = self.end;
        self.playing = false;
    }

    /// The next higher speed of [`SPEEDS`]; the top one stays.
    pub fn faster(&mut self) {
        if let Some(&s) = SPEEDS.iter().find(|&&s| s > self.speed) {
            self.speed = s;
        }
    }

    /// The next lower speed of [`SPEEDS`]; the lowest one stays.
    pub fn slower(&mut self) {
        if let Some(&s) = SPEEDS.iter().rev().find(|&&s| s < self.speed) {
            self.speed = s;
        }
    }

    /// Set the speed; it must be above 0, otherwise it is left as it is.
    pub fn set_speed(&mut self, speed: f64) {
        if speed.is_finite() && speed > 0.0 {
            self.speed = speed;
        }
    }

    /// Play, or pause. Playing at the end starts again from the start.
    pub fn toggle(&mut self) {
        if self.playing {
            self.playing = false;
        } else {
            if self.at_end() {
                self.time = self.start;
            }
            self.playing = true;
        }
    }

    /// Real time has passed: move on if playing. At the end it stops there.
    pub fn advance(&mut self, real_seconds: f64) {
        if !self.playing || !(real_seconds.is_finite() && real_seconds > 0.0) {
            return;
        }
        self.time += real_seconds * self.speed;
        if self.time >= self.end {
            self.time = self.end;
            self.playing = false;
        }
    }
}
