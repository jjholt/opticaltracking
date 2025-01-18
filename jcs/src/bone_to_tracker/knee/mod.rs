#![allow(non_camel_case_types)]
mod rigid_body;

use crate::{transform::Transform, RigidBody, Tracker};
use super::Global;

pub type Tibia = RigidBody<1>;
pub type Femur = RigidBody<2>;
pub type Patella = RigidBody<3>;

type ttTt<'a> = Transform<'a, Tracker<Tibia>, Tibia>;
type gTtt<'a> = Transform<'a, Global, Tracker<Tibia>>;
type gTt<'a> = Transform<'a, Global, Tibia>;


type ftTf<'a> = Transform<'a, Tracker<Femur>, Femur>;
type gTft<'a> = Transform<'a, Global, Tracker<Femur>>;
type gTf<'a> = Transform<'a, Global, Femur>;


type ptTp<'a> = Transform<'a, Tracker<Patella>, Patella>;
type gTpt<'a> = Transform<'a, Global, Tracker<Patella>>;
type gTp<'a> = Transform<'a, Global, Patella>;
