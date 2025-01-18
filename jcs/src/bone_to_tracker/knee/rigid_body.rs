use crate::data::{Position, ProbeData};
use crate::transform::{Mldivide, Transform};
use crate::{RigidBody, Tracker};
use super::{Tibia, Femur, Patella};

use nalgebra as na;

use crate::bone_to_tracker::{
    orientation::*,
    DefinedTracker, Landmark, Side, Global
};

impl<const ID: usize> RigidBody<ID> {
    pub fn new() -> RigidBody<ID> {
        Self::default()
    }
    pub fn set_medial(mut self, position: Position) -> Self {
        self.medial =  Some(Landmark::<RigidBody<ID>, Medial>::new("Black Probe", "Probe", position));
        self
    }
    pub fn set_lateral(mut self, position: Position) -> Self {
        self.lateral=  Some(Landmark::<RigidBody<ID>, Lateral>::new("Black Probe", "Probe", position));
        self
    }
    pub fn set_side(mut self, side: Side) -> Self {
        self.side = Some(side);
        self
    }
    pub fn tracker_in_global<'a>(position: Position) -> Transform<'a, Global, Tracker<Self>> {
        Transform::<Global, Tracker<Self>>::new(position.to_transform())
    }
}

impl Tibia {
    pub fn set_distal(mut self, position: Position) -> Self {
        self.distal = Some(Landmark::<Self, Distal>::new("Black Probe", "Probe", position));
        self
    }
}
impl Femur {
    pub fn set_proximal(mut self, position: Position) -> Self {
        self.proximal = Some(Landmark::<Self, Proximal>::new("Black Probe", "Probe", position));
        self
    }
}


impl DefinedTracker for Tibia {
    fn fixed_frame(&self) -> Option<Transform<'_, Global, Self>> {
        let med = self.medial.as_ref()?.translations();
        let lat = self.lateral.as_ref()?.translations();
        let dist = self.distal.as_ref()?.translations();
        let origin = (med + lat)/2.0;

        let tempk_ = na::Unit::new_normalize(origin - dist);
        let i_ = match self.side? {
            Side::Right =>  na::Unit::new_normalize(lat - med),
            Side::Left => na::Unit::new_normalize(med - lat),
        };
        let data = transform_from(origin, tempk_, i_);
        let data = na::Transform3::from_matrix_unchecked(data);
        Some(Transform::<Global, Self>::new(data))
    }
}

impl DefinedTracker for Femur {
    fn fixed_frame(&self) -> Option<Transform<'_, Global, Self>> {
        // These were modified to match the result observed in matlab
        let med = self.medial.as_ref()?.translations();
        let lat = self.lateral.as_ref()?.translations();
        let prox = self.proximal.as_ref()?.translations();
        let origin = (med + lat)/2.0;

        let tempk_ = na::Unit::new_normalize(prox - origin);
        let i_ = match self.side? { // These are inverted from expectation
            Side::Right => na::Unit::new_normalize(med - lat),
            Side::Left =>  na::Unit::new_normalize(lat - med),
        };
        let data = transform_from(origin, tempk_, i_);
        let data = na::Transform3::from_matrix_unchecked(data);
        Some(Transform::<Global, Self>::new(data))
    }
    
}

impl DefinedTracker for Patella {
    fn fixed_frame(&self) -> Option<Transform<'_, Global, Self>> {
        let med = self.medial.as_ref()?.translations();
        let lat = self.lateral.as_ref()?.translations();
        let dist = self.distal.as_ref()?.translations();
        let origin = (med + lat)/2.0;

        let tempk_ = na::Unit::new_normalize(origin - dist);
        let i_ = match self.side? {
            Side::Right =>  na::Unit::new_normalize(lat - med),
            Side::Left => na::Unit::new_normalize(med - lat),
        };
        let data = transform_from(origin, tempk_, i_);
        let data = na::Transform3::from_matrix_unchecked(data);
        Some(Transform::<Global, Self>::new(data))
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
        let transform = Femur::tracker_in_global(position);
        let mat = na::Matrix4::new( 0.8358,-0.3970, -0.3793, -149.4020, 0.4254, 0.9049, -0.0096, -19.4074, 0.3470, -0.1533, 0.9252, -2.1483e3, 0.0,  0.0, 0.0, 1.0); // Data generated using the matlab code
        let manual = na::Transform3::from_matrix_unchecked(mat);
        assert_relative_eq!(transform.inner(), &manual, epsilon = 5.0e-2)
    }

    #[test]
    fn tracker_transform() {
        // Landmark creation using Probe

        let side = Side::Right;
        let fm = ProbeData::new(0.822817363793104, 0.135707186206897, 0.440885412068966, -0.331890289655172, 15.3196551724138, -54.9971034482759, -2097.60234482759);
        let fl = ProbeData::new(0.403151039655172, 0.474684148275862, 0.419551994827586, -0.66038950862069, 16.9156724137931, 16.2064655172414, -2059.31424137931);
        let fp = ProbeData::new(0.428051244067797, 0.466236162711864, 0.447185444067797, -0.631995389830509, -8.56891525423729, 15.8874915254237, -2131.43533898305);
        
        let femur = Femur::new()
            .set_side(side)
            .set_medial(fm.into())
            .set_lateral(fl.into())
            .set_proximal(fp.into());
        // Tracker
        let femur_probe_data = ProbeData::new(0.9573733, -0.0372205, -0.1895465, 0.2147628, -149.371, -19.411, -2148.287);
        let position = femur_probe_data.into();

        let g_t_ft = Femur::tracker_in_global(position);
        
        println!("{:#?}", femur.in_tracker(&g_t_ft));
        println!("Femur in tracker {:#?}", femur.in_global());
    }
}
