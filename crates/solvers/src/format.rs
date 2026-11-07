//! What the solvers' formats share: reading numbers off the input array, and why an input is
//! refused, with one code per reason for the WebAssembly boundary.

use std::fmt;

/// Why a solver refused its input.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// A number isn't usable: not finite, not a whole number where one is needed, or out of
    /// range.
    BadNumber,
    /// The input ended early, or carried more than it said.
    BadLength,
    /// The input is for another version of the format.
    BadVersion,
    /// The route optimiser refused the problem.
    Route(route::Error),
    /// The nester refused the instance.
    Nest(nest::Error),
}

impl Error {
    /// The code the module returns, negated (-1 is an output buffer too small). One table for
    /// both solvers, so `ui/src/solvers.ts` keeps one list of messages.
    pub fn code(&self) -> i32 {
        match self {
            Error::BadNumber => 2,
            Error::BadLength => 3,
            Error::BadVersion => 4,
            Error::Route(e) => match e {
                route::Error::NoWays(_) => 10,
                route::Error::BadAfter(_) => 11,
                route::Error::BadFlip(_) => 12,
                route::Error::Cycle => 13,
                route::Error::BadTravel => 14,
                route::Error::NotFinite => 15,
            },
            Error::Nest(e) => match e {
                nest::Error::BadSize => 20,
                nest::Error::NoBins => 21,
                nest::Error::TooBig(_) => 22,
                nest::Error::TooMany => 23,
                nest::Error::Bug(_) => 24,
            },
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::BadNumber => write!(f, "a number in the input isn't usable"),
            Error::BadLength => write!(f, "the input is the wrong length"),
            Error::BadVersion => write!(f, "the input is for another version of the format"),
            Error::Route(e) => e.fmt(f),
            Error::Nest(e) => e.fmt(f),
        }
    }
}

impl std::error::Error for Error {}

impl From<route::Error> for Error {
    fn from(e: route::Error) -> Self {
        Error::Route(e)
    }
}

impl From<nest::Error> for Error {
    fn from(e: nest::Error) -> Self {
        Error::Nest(e)
    }
}

/// Whole numbers in an f64 are exact up to 2⁵³.
const EXACT: f64 = 9_007_199_254_740_992.0;

/// Reads an input array from the front, checking each number as it goes.
pub(crate) struct Reader<'a> {
    data: &'a [f64],
    at: usize,
}

impl<'a> Reader<'a> {
    pub(crate) fn new(data: &'a [f64]) -> Self {
        Self { data, at: 0 }
    }

    /// A finite number.
    pub(crate) fn num(&mut self) -> Result<f64, Error> {
        let v = *self.data.get(self.at).ok_or(Error::BadLength)?;
        self.at += 1;
        if v.is_finite() {
            Ok(v)
        } else {
            Err(Error::BadNumber)
        }
    }

    /// A whole number, of either sign.
    pub(crate) fn int(&mut self) -> Result<i64, Error> {
        let v = self.num()?;
        if v.fract() != 0.0 || v.abs() > EXACT {
            Err(Error::BadNumber)
        } else {
            Ok(v as i64)
        }
    }

    /// A whole number, not negative.
    pub(crate) fn whole(&mut self) -> Result<u64, Error> {
        u64::try_from(self.int()?).map_err(|_| Error::BadNumber)
    }

    /// 0 or 1.
    pub(crate) fn flag(&mut self) -> Result<bool, Error> {
        match self.int()? {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(Error::BadNumber),
        }
    }

    /// A whole number below `bound`.
    pub(crate) fn index(&mut self, bound: usize) -> Result<usize, Error> {
        let v = self.whole()?;
        usize::try_from(v)
            .ok()
            .filter(|&i| i < bound)
            .ok_or(Error::BadNumber)
    }

    pub(crate) fn indices(&mut self, count: usize, bound: usize) -> Result<Vec<usize>, Error> {
        (0..count).map(|_| self.index(bound)).collect()
    }

    /// A count of things still to come, each at least `width` numbers long, so a bad count
    /// can't ask for a huge allocation.
    pub(crate) fn count(&mut self, width: usize) -> Result<usize, Error> {
        let v = self.whole()?;
        let left = (self.data.len() - self.at) as u64;
        if v.saturating_mul(width.max(1) as u64) > left {
            return Err(Error::BadLength);
        }
        usize::try_from(v).map_err(|_| Error::BadLength)
    }

    /// The input must end here.
    pub(crate) fn end(&self) -> Result<(), Error> {
        if self.at == self.data.len() {
            Ok(())
        } else {
            Err(Error::BadLength)
        }
    }
}
