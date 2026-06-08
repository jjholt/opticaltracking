use input::{Config, Marker};

use crate::RigidBody;

pub struct Tracker<'a, const N: usize> {
    rigid_body: &'a RigidBody<N>,
    label: String,
    tx: Vec<f32>,
    ty: Vec<f32>,
    tz: Vec<f32>,
    q0: Vec<f32>,
    qx: Vec<f32>,
    qy: Vec<f32>,
    qz: Vec<f32>,
}

impl<'a, const N: usize> Tracker<'a, N> {
    pub fn new(marker: Marker, config: &Config) -> Self {
        Self {
            rigid_body: todo!(),
            label: marker.label,
            tx: marker.tx,
            ty: marker.ty,
            tz: marker.tz,
            q0: marker.q0,
            qx: marker.qx,
            qy: marker.qy,
            qz: marker.qz,
        }
    }
}
