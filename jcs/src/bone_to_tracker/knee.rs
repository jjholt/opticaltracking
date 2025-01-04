#![allow(non_camel_case_types)]
use crate::data::Position;
use crate::transform::{Mldivide, Transform};
use crate::{RigidBody, Tracker};
use nalgebra as na;

use super::{Bone, Global, Landmark, Side};
use super::orientation::*;

pub type RBTibia = RigidBody<1>;
pub type RBFemur = RigidBody<2>;
pub type RBPatella = RigidBody<3>;

pub struct Tibia {
    side: Side,
    medial: Landmark<RBTibia, Medial>,
    lateral: Landmark<RBTibia, Lateral>,
    distal: Landmark<RBTibia, Distal>,
}
pub struct Femur {
    side: Side,
    medial: Landmark<RBFemur, Medial>,
    lateral: Landmark<RBFemur, Lateral>,
    proximal: Landmark<RBFemur, Proximal>,
}
pub struct Patella {
    side: Side,
    medial: Landmark<RBPatella, Medial>,
    lateral: Landmark<RBPatella, Lateral>,
    distal: Landmark<RBPatella, Distal>,
}

type ttTt<'a> = Transform<'a, Tracker<RBTibia>, RBTibia>;
type gTtt<'a> = Transform<'a, Global, Tracker<RBTibia>>;
type gTt<'a> = Transform<'a, Global, RBTibia>;

impl Tibia {
    pub fn tracker_in_global(position: &Position) -> gTtt {
        Transform::<Global, Tracker<RBTibia>>::new(position.to_transform())
    }
    pub fn in_tracker(&self, tracker: &gTtt) -> ttTt {
        tracker.mldivide(&self.fixed_frame())
    }
    pub fn in_global(&self) -> gTt {
        todo!()
    }
    #[cfg(test)]
    pub fn new(side: Side, medial: Landmark<RBTibia, Medial>, lateral: Landmark<RBTibia, Lateral> , distal: Landmark<RBTibia, Distal>) -> Self {
        Self {side, medial, lateral, distal}
    }
}

type ftTf<'a> = Transform<'a, Tracker<RBFemur>, RBFemur>;
type gTft<'a> = Transform<'a, Global, Tracker<RBFemur>>;
type gTf<'a> = Transform<'a, Global, RBFemur>;

impl Femur {
    pub fn tracker_in_global(position: &Position) -> gTft {
        Transform::<Global, Tracker<RBFemur>>::new(position.to_transform())
    }
    pub fn in_tracker(&self, tracker: &gTft) -> ftTf {
        tracker.mldivide(&self.fixed_frame())
    }
    pub fn in_global(&self) -> gTf {
        self.fixed_frame()
    }
    #[cfg(test)]
    pub(crate) fn new(side: Side, medial: Landmark<RBFemur, Medial>, lateral: Landmark<RBFemur, Lateral>, proximal: Landmark<RBFemur, Proximal>) -> Self {
        Self {side, medial, lateral, proximal}
    }
}

type ptTp<'a> = Transform<'a, Tracker<RBPatella>, RBPatella>;
type gTpt<'a> = Transform<'a, Global, Tracker<RBPatella>>;
type gTp<'a> = Transform<'a, Global, RBPatella>;
impl Patella {
    pub fn tracker_in_global(position: &Position) -> gTpt {
        Transform::<Global, Tracker<RBPatella>>::new(position.to_transform())
    }
    pub fn in_tracker(&self, tracker: & gTpt ) -> ptTp {
        tracker.mldivide(&self.fixed_frame())
    }
    pub fn in_global(&self) -> gTp {
        todo!()
    }
}

impl Bone for Tibia {
    type Body = RBTibia;

    fn fixed_frame(&self) -> Transform<'_, Global, Self::Body> {
        let med = self.medial.translations();
        let lat = self.lateral.translations();
        let dist = self.distal.translations();
        let origin = (med + lat)/2.0;

        let tempk_ = na::Unit::new_normalize(origin - dist);
        let i_ = match self.side {
            Side::Right =>  na::Unit::new_normalize(lat - med),
            Side::Left => na::Unit::new_normalize(med - lat),
        };
        let data = transform_from(origin, tempk_, i_);
        let data = na::Transform3::from_matrix_unchecked(data);
        Transform::<Global, Self::Body>::new(data)
    }
}

impl Bone for Femur {
    type Body = RBFemur;

    fn fixed_frame(&self) -> Transform<'_, Global, Self::Body> {
        // These were modified to match the result observed in matlab
        let med = self.medial.translations();
        let lat = self.lateral.translations();
        let prox = self.proximal.translations();
        let origin = (med + lat)/2.0;

        let tempk_ = na::Unit::new_normalize(prox - origin);
        let i_ = match self.side { // These are inverted from expectation
            Side::Right => na::Unit::new_normalize(med - lat),
            Side::Left =>  na::Unit::new_normalize(lat - med),
        };
        let data = transform_from(origin, tempk_, i_);
        let data = na::Transform3::from_matrix_unchecked(data);
        Transform::<Global, Self::Body>::new(data)
    }
}

impl Bone for Patella {
    type Body = RBPatella;

    fn fixed_frame(&self) -> Transform<'_, Global, Self::Body> {
        let med = self.medial.translations();
        let lat = self.lateral.translations();
        let dist = self.distal.translations();
        let origin = (med + lat)/2.0;

        let tempk_ = na::Unit::new_normalize(origin - dist);
        let i_ = match self.side {
            Side::Right =>  na::Unit::new_normalize(lat - med),
            Side::Left => na::Unit::new_normalize(med - lat),
        };
        let data = transform_from(origin, tempk_, i_);
        let data = na::Transform3::from_matrix_unchecked(data);
        Transform::<Global, Self::Body>::new(data)
    }
}

fn transform_from(origin: na::Vector3<f64>, tempk_: na::Unit<na::Vector3<f64>>, i_: na::Unit<na::Vector3<f64>>) -> na::Matrix4<f64> {
    let j_ = na::Unit::new_normalize(tempk_.cross(&i_));
    let k_ = na::Unit::new_normalize(i_.cross(&j_));
    let m = na::Matrix3::from_columns( &[i_.into_inner(), j_.into_inner(), k_.into_inner()]);
    let rotation = na::Rotation3::from_matrix(&m);

    let translation = na::Matrix4::new_translation(&origin);
    translation * rotation.to_homogeneous()
}

#[cfg(test)]
mod test {

    use crate::data::ProbeData;

    use super::*;
    
    #[test]
    fn creates_femoral_tracker() {
        let femur_probe_data = ProbeData::new(0.9573733, -0.0372205, -0.1895465, 0.2147628, -149.371, -19.411, -2148.287);
        let position = Position::new(&femur_probe_data);
        let transform = Femur::tracker_in_global(&position);
        let mat = na::Matrix4::new( 0.8358,-0.3970, -0.3793, -149.4020, 0.4254, 0.9049, -0.0096, -19.4074, 0.3470, -0.1533, 0.9252, -2.1483e3, 0.0,  0.0, 0.0, 1.0); // Data generated using the matlab code
        let manual = na::Transform3::from_matrix_unchecked(mat);
        assert_relative_eq!(transform.inner(), &manual, epsilon = 5.0e-2)
    }

    #[test]
    fn tracker_transform() {
        // Landmark creation using Probe

        let side = Side::Right;
        let fm = ProbeData::new(0.822817363793104, 0.135707186206897, 0.440885412068966, -0.331890289655172, 15.3196551724138, -54.9971034482759, -2097.60234482759);
        let femur_medial = Landmark::<RBFemur, Medial>::new("Black Probe", "Probe", fm.into());
        println!("Femur Medial: {}", femur_medial);

        let fl = ProbeData::new(0.403151039655172, 0.474684148275862, 0.419551994827586, -0.66038950862069, 16.9156724137931, 16.2064655172414, -2059.31424137931);
        let femur_lateral = Landmark::<RBFemur, Lateral>::new("Black Probe", "Probe", fl.into());

        let fp = ProbeData::new(0.428051244067797, 0.466236162711864, 0.447185444067797, -0.631995389830509, -8.56891525423729, 15.8874915254237, -2131.43533898305);
        let femur_proximal= Landmark::<RBFemur, Proximal>::new("Black Probe", "Probe", fp.into());

        let femur = Femur::new(side, femur_medial, femur_lateral, femur_proximal);
        // Tracker
        let femur_probe_data = ProbeData::new(0.9573733, -0.0372205, -0.1895465, 0.2147628, -149.371, -19.411, -2148.287);
        let position = femur_probe_data.into();
        let g_t_ft = Femur::tracker_in_global(&position); // femoral tracker in global
        

        let t = femur.in_tracker(&g_t_ft);

        println!("{:#?}", femur.in_tracker(&g_t_ft));
        println!("Femur in tracker {:#?}", femur.in_global());


    }
}
