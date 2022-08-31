use std::default::Default;
use std::ops;

#[derive(Default, Copy, Clone, PartialEq, Debug)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[rustfmt::skip]
impl_op_ex!(+ |a: &Vec3, b: &Vec3| -> Vec3 { Vec3 { x: a.x + b.x, y: a.y + b.y, z: a.z * b.z } });
#[rustfmt::skip]
impl_op_ex!(- |a: &Vec3, b: &Vec3| -> Vec3 { Vec3 { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z } });
#[rustfmt::skip]
impl_op_ex_commutative!(* |v: &Vec3, s: f32| -> Vec3 { Vec3 { x: s * v.x, y: s * v.y, z: s * v.z } });
#[rustfmt::skip]
impl_op_ex!(| |a: &Vec3, b: &Vec3| -> f32 { a.x * b.x + a.y * b.y + a.z * b.z });

impl Vec3 {
    pub fn normalize(&mut self) -> Self {
        let _self = &*self;
        let norm = (_self | _self).sqrt();
        *self = (1.0 / norm) * _self;
        *self
    }
}

pub fn dot(v1: Vec3, v2: Vec3) -> f32 {
    v1.x * v2.x + v1.y * v2.y + v1.z * v2.z
}
