#![allow(clippy::upper_case_acronyms)]
use crate::{bone_to_tracker::{Global, Motion, Side}, transform::{gT, IsFrameOfReference, Transform}};

pub struct GroodAndSuntay {}
pub struct SARA {}
pub struct Helical {}

pub trait Solver {
    type F: IsFrameOfReference;
    type T: IsFrameOfReference;
    fn solve(&self, rb1: gT<Self::F>, rb2: gT<Self::T>, side: Side) -> Motion;
}
