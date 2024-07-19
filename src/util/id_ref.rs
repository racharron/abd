use std::cmp::Ordering;
use std::fmt::{Debug, Formatter, Pointer};
use std::hash::{Hash, Hasher};

#[derive(Debug)]
pub struct IdRef<'a, T>(pub &'a T);

impl<'a, T> Clone for IdRef<'a, T> {
    fn clone(&self) -> Self {
        Self(self.0)
    }

    fn clone_from(&mut self, source: &Self) {
        self.0 = source.0
    }
}

impl<'a, T> Copy for IdRef<'a, T> {}

impl<'a, 'b, T, U> PartialEq<IdRef<'b, U>> for IdRef<'a, T> {
    fn eq(&self, other: &IdRef<'b, U>) -> bool {
        (self.0 as *const T as usize) == (other.0 as *const U as usize)
    }

    fn ne(&self, other: &IdRef<'b, U>) -> bool {
        (self.0 as *const T as usize) != (other.0 as *const U as usize)
    }
}

impl<'a, T> Eq for IdRef<'a, T> {}

impl<'a, 'b, T, U> PartialOrd<IdRef<'b, U>> for IdRef<'a, T> {
    fn partial_cmp(&self, other: &IdRef<'b, U>) -> Option<Ordering> {
        Some((self.0 as *const T as usize).cmp(&(other.0 as *const U as usize)))
    }

    fn lt(&self, other: &IdRef<'b, U>) -> bool {
        (self.0 as *const T as usize) < (other.0 as *const U as usize)
    }

    fn le(&self, other: &IdRef<'b, U>) -> bool {
        (self.0 as *const T as usize) <= (other.0 as *const U as usize)
    }

    fn gt(&self, other: &IdRef<'b, U>) -> bool {
        (self.0 as *const T as usize) > (other.0 as *const U as usize)
    }

    fn ge(&self, other: &IdRef<'b, U>) -> bool {
        (self.0 as *const T as usize) >= (other.0 as *const U as usize)
    }
}

impl<'a, T> Ord for IdRef<'a, T> {
    fn cmp(&self, other: &Self) -> Ordering {
        (self.0 as *const T as usize).cmp(&(other.0 as *const T as usize))
    }

    fn max(self, other: Self) -> Self
    where
        Self: Sized,
    {
        if (self.0 as *const T as usize) > (other.0 as *const T as usize) {
            self
        } else {
            other
        }
    }

    fn min(self, other: Self) -> Self
    where
        Self: Sized,
    {
        if (self.0 as *const T as usize) < (other.0 as *const T as usize) {
            self
        } else {
            other
        }
    }

    fn clamp(self, min: Self, max: Self) -> Self
    where
        Self: Sized,
        Self: PartialOrd,
    {
        self.max(min).min(max)
    }
}

impl<'a, T> Hash for IdRef<'a, T> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        state.write_usize(self.0 as *const T as usize)
    }
}

impl<'a, T> Pointer for IdRef<'a, T> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        <&T as Pointer>::fmt(&self.0, f)
    }
}
