#![allow(non_camel_case_types)]
mod arithmetic;
pub use arithmetic::{Mldivide, IsFrameOfReference, Transform};

use crate::{bone_to_tracker::Global, Tracker};
pub type gT<X> = Transform<Global, X>;
pub type tT<X> = Transform<Tracker<X>, X>;
