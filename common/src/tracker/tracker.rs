use std::marker::PhantomData;

use crate::tracker::Marker;
// use input::{Config};

use crate::{Bone, Landmark, Position};

pub struct Tracker<B: Bone, P: Position> {
    bone: PhantomData<B>,
    position: PhantomData<P>,
    label: String,
    tx: Vec<f32>,
    ty: Vec<f32>,
    tz: Vec<f32>,
    q0: Vec<f32>,
    qx: Vec<f32>,
    qy: Vec<f32>,
    qz: Vec<f32>,
}

impl <B: Bone, P: Position> Tracker <B, P>{
    // pub fn new(marker: Marker, config: &Config) -> Self {
    //     Self {
    //         bone: PhantomData,
    //         position: PhantomData,
    //         label: marker.label,
    //         tx: marker.tx,
    //         ty: marker.ty,
    //         tz: marker.tz,
    //         q0: marker.q0,
    //         qx: marker.qx,
    //         qy: marker.qy,
    //         qz: marker.qz,
    //     }
    // }

    pub fn from_marker(marker: Marker, landmark: Landmark<B, P>) -> Tracker<B,P> {
        Self {
            bone: PhantomData,
            position: PhantomData,
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

