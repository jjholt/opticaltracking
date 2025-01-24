use crate::{transform::{IsFrameOfReference, Transform}, RigidBody};

mod gns;
mod sara;
mod helical;

pub use gns::GroodAndSuntay;

type LocalFloat = f32;
pub trait Solver {
    fn solve(&self, one: RigidBody, other: RigidBody) -> Motion;
}
pub struct Motion {
    rx: LocalFloat,
    ry: LocalFloat,
    rz: LocalFloat,
    x: LocalFloat,
    y: LocalFloat,
    z: LocalFloat,
}
