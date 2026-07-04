use std::marker::PhantomData;

use common::{Bone, Landmark, Position};

use crate::RigidBody;

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
impl Patella {
    pub fn medial() -> Landmark<Patella, Medial> {
        Landmark {
            bone: PhantomData,
            position: PhantomData,
        }
    }
    pub fn lateral() -> Landmark<Patella, Lateral> {
        Landmark {
            bone: PhantomData,
            position: PhantomData,
        }
    }
    pub fn distal() -> Landmark<Patella, Distal> {
        Landmark {
            bone: PhantomData,
            position: PhantomData,
        }
    }
}
impl Tibia {
    pub fn medial() -> Landmark<Tibia, Medial> {
        Landmark {
            bone: PhantomData,
            position: PhantomData,
        }
    }
    pub fn lateral() -> Landmark<Tibia, Lateral> {
        Landmark {
            bone: PhantomData,
            position: PhantomData,
        }
    }
    pub fn distal() -> Landmark<Tibia, Distal> {
        Landmark {
            bone: PhantomData,
            position: PhantomData,
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    #[cfg(feature = "knee")]
    fn creates_objects() {
        use crate::knee::Femur;
        use common::tracker::Marker;

        let fm = Femur::medial();
        let tracker = Marker::default().assign(fm);
    }
}
