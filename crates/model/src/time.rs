//! Time: VDA 5050 writes ISO 8601 UTC timestamps; inside the program a time is a
//! number of milliseconds since the Unix epoch (1970-01-01T00:00:00Z).

/// A time in whole milliseconds since 1970-01-01T00:00:00Z.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Millis(pub i64);

impl Millis {
    /// The seconds that have passed from `earlier` to this time; 0 if this time
    /// is not later.
    pub fn seconds_since(self, earlier: Millis) -> f64 {
        self.millis_since(earlier) as f64 / 1000.0
    }

    /// The milliseconds that have passed from `earlier` to this time; 0 if this
    /// time is not later.
    pub fn millis_since(self, earlier: Millis) -> i64 {
        self.0.saturating_sub(earlier.0).max(0)
    }
}

/// Milliseconds since the Unix epoch for a UTC calendar time (month 1–12, day
/// 1–31; the caller checks the ranges).
pub fn unix_ms_from_utc(year: i64, month: i64, day: i64, hour: i64, min: i64, sec: i64) -> i64 {
    // days since 1970-01-01 (H. Hinnant's algorithm, inverse of `iso8601`)
    let y = year - i64::from(month <= 2);
    let era = y.div_euclid(400);
    let yoe = y.rem_euclid(400);
    let mp = if month > 2 { month - 3 } else { month + 9 };
    let doy = (153 * mp + 2) / 5 + day - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    let days = era * 146_097 + doe - 719_468;
    ((days * 24 + hour) * 60 + min) * 60_000 + sec * 1000
}

/// Milliseconds since the Unix epoch from an ISO 8601 UTC timestamp such as
/// `2026-11-20T10:00:00.250Z` (the milliseconds are optional). `None` if the text
/// is not of that form or a field is out of range (the year: 0 to 9999).
pub fn unix_ms_from_iso8601(s: &str) -> Option<i64> {
    let s = s.strip_suffix('Z')?;
    let (date, time) = s.split_once('T')?;
    let mut d = date.split('-');
    let (year, month, day) = (
        d.next()?.parse::<i64>().ok()?,
        d.next()?.parse::<i64>().ok()?,
        d.next()?.parse::<i64>().ok()?,
    );
    if d.next().is_some() {
        return None;
    }
    let (hms, frac) = match time.split_once('.') {
        Some((a, b)) => (a, Some(b)),
        None => (time, None),
    };
    let mut t = hms.split(':');
    let (h, m, sec) = (
        t.next()?.parse::<i64>().ok()?,
        t.next()?.parse::<i64>().ok()?,
        t.next()?.parse::<i64>().ok()?,
    );
    if t.next().is_some() {
        return None;
    }
    let ms = match frac {
        None => 0,
        Some(f) if !f.is_empty() && f.len() <= 9 && f.bytes().all(|c| c.is_ascii_digit()) => {
            // the first three digits are the milliseconds
            let padded = format!("{f:0<3}");
            padded[..3].parse::<i64>().ok()?
        }
        Some(_) => return None,
    };
    // (a year of more than four digits would also overflow the arithmetic)
    let valid = (0..=9999).contains(&year)
        && (1..=12).contains(&month)
        && (1..=31).contains(&day)
        && (0..=23).contains(&h)
        && (0..=59).contains(&m)
        && (0..=60).contains(&sec);
    valid.then(|| unix_ms_from_utc(year, month, day, h, m, sec) + ms)
}

/// `2026-11-20T10:00:00.250Z` from milliseconds since the Unix epoch. A time
/// before the year 0 or after the year 9999 gives a text that is no ISO 8601
/// timestamp of this form.
pub fn iso8601(unix_ms: i64) -> String {
    let secs = unix_ms.div_euclid(1000);
    let ms = unix_ms.rem_euclid(1000);
    let days = secs.div_euclid(86_400);
    let sod = secs.rem_euclid(86_400);
    // civil date from days since 1970-01-01 (H. Hinnant's algorithm)
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}.{ms:03}Z",
        sod / 3600,
        sod % 3600 / 60,
        sod % 60
    )
}

#[cfg(test)]
mod tests {
    use super::{iso8601, unix_ms_from_iso8601, unix_ms_from_utc};

    #[test]
    fn unix_ms_from_utc_is_the_inverse() {
        assert_eq!(unix_ms_from_utc(1970, 1, 1, 0, 0, 0), 0);
        assert_eq!(unix_ms_from_utc(2024, 2, 29, 12, 34, 56), 1_709_210_096_000);
        assert_eq!(unix_ms_from_utc(2026, 11, 20, 10, 0, 0), 1_795_168_800_000);
        for ms in [0, 951_782_400_000, 1_709_210_096_000, 4_102_444_799_000] {
            let s = iso8601(ms);
            let n = |a: usize, b: usize| s[a..b].parse::<i64>().unwrap();
            assert_eq!(
                unix_ms_from_utc(n(0, 4), n(5, 7), n(8, 10), n(11, 13), n(14, 16), n(17, 19)),
                ms
            );
        }
    }

    #[test]
    fn iso8601_known_dates() {
        assert_eq!(iso8601(0), "1970-01-01T00:00:00.000Z");
        assert_eq!(iso8601(1_000_000_000_000), "2001-09-09T01:46:40.000Z");
        // leap day, and a time with milliseconds
        assert_eq!(iso8601(1_709_210_096_250), "2024-02-29T12:34:56.250Z");
        assert_eq!(iso8601(1_795_168_800_000), "2026-11-20T10:00:00.000Z");
    }

    #[test]
    fn a_year_outside_four_digits_is_refused_without_overflow() {
        for bad in [
            "10000-01-01T00:00:00Z",
            "99999999999999999-01-01T00:00:00Z",
            "9223372036854775807-12-31T23:59:59.999Z",
            "9223372036854775808-01-01T00:00:00Z",
        ] {
            assert_eq!(unix_ms_from_iso8601(bad), None, "{bad}");
        }
        // the first and the last time that can be written
        for edge in ["0000-01-01T00:00:00.000Z", "9999-12-31T23:59:59.999Z"] {
            let ms = unix_ms_from_iso8601(edge).unwrap();
            assert_eq!(iso8601(ms), edge);
        }
    }
}
