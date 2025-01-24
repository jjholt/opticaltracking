use super::{Femur, Patella, Tibia};
use crate::data::{Datum, ProbeData, ProbeRawData};
use crate::transform::{Mldivide, Transform};
use crate::{RigidBody, Tracker};

use nalgebra as na;

use crate::bone_to_tracker::{orientation::*, DefinedTracker, Global, Landmark, Side};

impl<const ID: usize> RigidBody<ID> {
    pub fn new() -> RigidBody<ID> {
        Self::default()
    }
    pub fn set_medial(mut self, probe_data: ProbeData) -> Self {
        self.medial = Some(Landmark::<RigidBody<ID>, Medial>::new( "Black Probe", "Probe", probe_data));
        self
    }
    pub fn set_tracker(mut self, probe_data: ProbeData ) -> Self {
        self.tracker = Some(Transform::<Global, Tracker<RigidBody<ID>>>::new(probe_data.to_transform()));
        self
    }
    pub fn set_lateral(mut self, probe_data: ProbeData) -> Self {
        self.lateral = Some(Landmark::<RigidBody<ID>, Lateral>::new( "Black Probe", "Probe", probe_data));
        self
    }
    pub fn set_side(mut self, side: Side) -> Self {
        self.side = Some(side);
        self
    }
    pub fn tracker_in_global(probe_data: ProbeData) -> Transform<Global, Tracker<Self>> {
        Transform::<Global, Tracker<RigidBody<ID>>>::new(probe_data.to_transform())
    }
}
impl <const ID: usize> RigidBody<ID> where Self: DefinedTracker {
    pub fn take_datum(&self, datum: Datum<Tracker<Self>>) -> Option<Transform<Global, RigidBody<ID>>> {
        let a = datum.to_transform();
        let b = self.tracker.as_ref()?.mldivide(&self.fixed_frame()?);
        Some(a * b)
    }
    pub fn origin(&self) -> na::Point3<f32> {
        self.in_tracker()
            .inner()
            .transform_point(&self.tracker.as_ref().unwrap().translation())
    }
    fn in_tracker(&self) -> Transform<Tracker<Self>, Self> {
        self.tracker.as_ref().unwrap().mldivide(&self.fixed_frame().unwrap())
    }
}

impl Tibia {
    pub fn set_distal(mut self, probe_data: ProbeData) -> Self {
        self.distal = Some(Landmark::<Tibia, Distal>::new( "Black Probe", "Probe", probe_data));
        self
    }
}
impl Femur {
    pub fn set_proximal(mut self, probe_data: ProbeData) -> Self {
        self.proximal = Some(Landmark::<Femur, Proximal>::new( "Black Probe", "Probe", probe_data));
        self
    }
}

impl DefinedTracker for Tibia {
    fn fixed_frame(&self) -> Option<Transform<Global, Self>> {
        let med = self.medial.as_ref()?.translations();
        let lat = self.lateral.as_ref()?.translations();
        let dist = self.distal.as_ref()?.translations();
        let origin = (med + lat) / 2.0;

        let tempk_ = na::Unit::new_normalize(origin - dist);
        let i_ = match self.side? {
            Side::Right => na::Unit::new_normalize(lat - med),
            Side::Left => na::Unit::new_normalize(med - lat),
        };
        let data = transform_from(origin, tempk_, i_);
        let data = na::Transform3::from_matrix_unchecked(data);
        Some(Transform::<Global, Self>::new(data))
    }
}

impl DefinedTracker for Femur {
    fn fixed_frame(&self) -> Option<Transform<Global, Self>> {
        // These were modified to match the result observed in matlab
        let med = self.medial.as_ref()?.translations();
        let lat = self.lateral.as_ref()?.translations();
        let prox = self.proximal.as_ref()?.translations();
        let origin = (med + lat) / 2.0;

        let tempk_ = na::Unit::new_normalize(prox - origin);
        let i_ = match self.side? {
            // These are inverted from expectation
            Side::Right => na::Unit::new_normalize(med - lat),
            Side::Left => na::Unit::new_normalize(lat - med),
        };
        let data = transform_from(origin, tempk_, i_);
        let data = na::Transform3::from_matrix_unchecked(data);
        Some(Transform::<Global, Self>::new(data))
    }
}

impl DefinedTracker for Patella {
    fn fixed_frame(&self) -> Option<Transform<Global, Self>> {
        let med = self.medial.as_ref()?.translations();
        let lat = self.lateral.as_ref()?.translations();
        let dist = self.distal.as_ref()?.translations();
        let origin = (med + lat) / 2.0;

        let tempk_ = na::Unit::new_normalize(origin - dist);
        let i_ = match self.side? {
            Side::Right => na::Unit::new_normalize(lat - med),
            Side::Left => na::Unit::new_normalize(med - lat),
        };
        let data = transform_from(origin, tempk_, i_);
        let data = na::Transform3::from_matrix_unchecked(data);
        Some(Transform::<Global, Self>::new(data))
    }
}

fn transform_from(
    origin: na::Vector3<f32>,
    tempk_: na::Unit<na::Vector3<f32>>,
    i_: na::Unit<na::Vector3<f32>>,
) -> na::Matrix4<f32> {
    let j_ = na::Unit::new_normalize(tempk_.cross(&i_));
    let k_ = na::Unit::new_normalize(i_.cross(&j_));
    let m = na::Matrix3::from_columns(&[i_.into_inner(), j_.into_inner(), k_.into_inner()]);
    let rotation = na::Rotation3::from_matrix(&m);

    let translation = na::Matrix4::new_translation(&origin);
    translation * rotation.to_homogeneous()
}

#[cfg(test)]
mod test {

    use crate::data::ProbeRawData;

    use super::*;

    #[test]
    fn creates_femoral_tracker() {
        let femur_raw_probe_data = ProbeRawData::new(
            0.9573733, -0.0372205, -0.1895465, 0.2147628, -149.371, -19.411, -2148.287,
        );
        let probe_data = ProbeData::new(&femur_raw_probe_data);
        let transform = Femur::tracker_in_global(probe_data);
        let mat = na::Matrix4::new(
            0.8358, -0.3970, -0.3793, -149.402, 0.4254, 0.9049, -0.0096, -19.4074, 0.3470,
            -0.1533, 0.9252, -2.1483e3, 0.0, 0.0, 0.0, 1.0,
        ); // Data generated using the matlab code
        let manual = na::Transform3::from_matrix_unchecked(mat);
        assert_relative_eq!(transform.inner(), &manual, epsilon = 5.0e-2)
    }

    #[test]
    fn tracker_transform() {
        // Landmark creation using Probe

        let side = Side::Right;
        let fm = ProbeRawData::new(0.8228, 0.1357, 0.4408, -0.3318, 15.3196, -54.9971, -2097.6023);
        let fl = ProbeRawData::new(0.4031, 0.4746, 0.4195, -0.6603, 16.9156, 16.2064, -2059.3142);
        let fp = ProbeRawData::new(0.4280, 0.4662, 0.4471, -0.6319, -8.5689, 15.8874, -2131.4353);

        let femur = Femur::new()
            .set_side(side)
            .set_medial(fm.into())
            .set_lateral(fl.into())
            .set_proximal(fp.into());
        // Tracker
        let femur_probe_data = ProbeRawData::new(0.9573733, -0.0372205, -0.1895465, 0.2147628, -149.371, -19.411, -2148.287);
        let position = femur_probe_data.into();

        let g_t_ft = Femur::tracker_in_global(position);
        println!("Femur in tracker {}", femur.in_global().unwrap());
    }
}
