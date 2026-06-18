use crate::{Bone, Landmark, Position, RigidBody};

use std::marker::PhantomData;

pub type Femur = RigidBody<1>;
pub type Tibia = RigidBody<2>;
pub type Patella = RigidBody<3>;

pub struct Medial;
pub struct Lateral;
pub struct Proximal;
pub struct Distal;

impl Bone for Femur {}
impl Bone for Tibia {}
impl Bone for Patella {}

impl Position for Medial {}
impl Position for Lateral {}
impl Position for Proximal {}
impl Position for Distal {}

impl Femur {
    pub fn medial() -> Landmark<Femur, Medial> {
        Landmark {
            bone: PhantomData,
            position: PhantomData,
        }
    }
    pub fn lateral() -> Landmark<Femur, Lateral> {
        Landmark {
            bone: PhantomData,
            position: PhantomData,
        }
    }
    pub fn proximal() -> Landmark<Femur, Proximal> {
        Landmark {
            bone: PhantomData,
            position: PhantomData,
        }
    }
}
