#![allow(non_camel_case_types)]
mod rigid_body;

use crate::{transform::Transform, RigidBody, Tracker};
use super::Global;
use crate::data::Datum;

pub type Tibia = RigidBody<1>;
pub type Femur = RigidBody<2>;
pub type Patella = RigidBody<3>;

type ttTt = Transform<Tracker<Tibia>, Tibia>;
type gTtt = Transform<Global, Tracker<Tibia>>;
type gTt = Transform<Global, Tibia>;


type ftTf = Transform<Tracker<Femur>, Femur>;
type gTft = Transform<Global, Tracker<Femur>>;
type gTf = Transform<Global, Femur>;


type ptTp = Transform<Tracker<Patella>, Patella>;
type gTpt = Transform<Global, Tracker<Patella>>;
type gTp = Transform<Global, Patella>;

pub type DataTibia = Datum<Tracker<Tibia>>;
pub type DataFemur = Datum<Tracker<Femur>>;
pub type DataPatella = Datum<Tracker<Patella>>;
