use std::default::Default;
use std::iter::zip;
use std::ops;
use std::ops::{Index, IndexMut};

#[derive(Default, Copy, Clone, PartialEq, Debug)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[rustfmt::skip]
impl_op_ex!(+ |a: &Vec3, b: &Vec3| -> Vec3 { Vec3 { x: a.x + b.x, y: a.y + b.y, z: a.z * b.z } });
#[rustfmt::skip]
impl_op_ex_commutative!(+ |a: &Vec3, b: f32| -> Vec3 { Vec3 { x: a.x + b, y: a.y + b, z: a.z * b } });
#[rustfmt::skip]
impl_op_ex!(- |a: &Vec3, b: &Vec3| -> Vec3 { Vec3 { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z } });
#[rustfmt::skip]
impl_op_ex_commutative!(* |v: &Vec3, s: f32| -> Vec3 { Vec3 { x: s * v.x, y: s * v.y, z: s * v.z } });
#[rustfmt::skip]
impl_op_ex!(/ |v: &Vec3, s: f32| -> Vec3 { Vec3 { x: v.x / s, y: v.y / s, z: v.z / s } });
#[rustfmt::skip]
impl_op_ex!(| |a: &Vec3, b: &Vec3| -> f32 { a.x * b.x + a.y * b.y + a.z * b.z });
#[rustfmt::skip]
impl_op_ex!(^ |a: &Vec3, b: &Vec3| -> Vec3 { Vec3 {
    x: a.y * b.z - a.z * b.y, 
    y: a.z * b.x - a.x * b.z,
    z: a.x * b.y - a.y * b.x
}});


impl Vec3 {
    pub fn normalize(&mut self) -> Self {
        let _self = &*self;
        let norm = (_self | _self).sqrt();
        *self = (1.0 / norm) * _self;
        *self
    }

    pub fn length(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
}

#[macro_export]
macro_rules! vec3 {
    ( $x:expr, $y:expr, $z:expr ) => {
        Vec3 {
            x: $x,
            y: $y,
            z: $z,
        }
    };
}

impl Index<usize> for Vec3 {
    type Output = f32;

    fn index(&self, idx: usize) -> &Self::Output {
        match idx {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("Invalid index"),
        }
    }
}

impl IndexMut<usize> for Vec3 {
    fn index_mut(&mut self, idx: usize) -> &mut Self::Output {
        match idx {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            i => panic!("Invalid index {}, must be 0, 1 or 2", i),
        }
    }
}

pub fn dot(v1: Vec3, v2: Vec3) -> f32 {
    v1.x * v2.x + v1.y * v2.y + v1.z * v2.z
}

#[derive(Default, Copy, Clone, PartialEq, Debug)]
pub struct Mat3 {
    elems: [f32; 9],
}

impl Index<usize> for Mat3 {
    type Output = f32;

    fn index(&self, idx: usize) -> &Self::Output {
        assert!(idx < 9, "Invalid index {}, should be < 9", idx);
        &self.elems[idx]
    }
}

impl Index<(usize, usize)> for Mat3 {
    type Output = f32;

    fn index(&self, idx: (usize, usize)) -> &Self::Output {
        assert!(
            idx.0 < 3 && idx.1 < 3,
            "Invalid indices ({}, {}), both row and col should be < 3",
            idx.0,
            idx.1
        );
        &self.elems[3 * idx.0 + idx.1]
    }
}

impl IndexMut<usize> for Mat3 {
    fn index_mut(&mut self, idx: usize) -> &mut Self::Output {
        assert!(idx < 9, "Invalid index {}, should be < 9", idx);
        &mut self.elems[idx]
    }
}

impl IndexMut<(usize, usize)> for Mat3 {
    fn index_mut(&mut self, idx: (usize, usize)) -> &mut Self::Output {
        assert!(
            idx.0 < 3 && idx.1 < 3,
            "Invalid indices ({}, {}), both row and col should be < 3",
            idx.0,
            idx.1
        );
        &mut self.elems[3 * idx.0 + idx.1]
    }
}

#[derive(Default, Copy, Clone, PartialEq, Debug)]
pub struct Mat4 {
    elems: [f32; 16],
}

#[macro_export]
macro_rules! mat3 {
    ( $a:expr, $b:expr, $c:expr, $d:expr, $e:expr, $f:expr, $g:expr, $h:expr, $i:expr ) => {
        Mat3 {
            elems: [$a, $b, $c, $d, $e, $f, $g, $h, $i],
        }
    };
}

impl Mat3 {
    pub fn new() -> Self {
        Mat3 { elems: [0f32; 9] }
    }

    pub fn new_diag(x: f32) -> Self {
        let mut m = Mat3 { elems: [0f32; 9] };
        m[(0, 0)] = x;
        m[(1, 1)] = x;
        m[(2, 2)] = x;
        m
    }

    pub fn eye() -> Self {
        Self::new_diag(1.0)
    }

    pub fn transpose(&self) -> Self {
        let mut m = Mat3::new();
        for i in 0..3 {
            for j in 0..3 {
                m[(i, j)] = self[(j, i)];
            }
        }
        m
    }

    #[rustfmt::skip]
    pub fn det(&self) -> f32 {
        self[(0, 0)] * (self[(1, 1)] * self[(2, 2)] - self[(2, 1)] * self[(1, 2)]) +
        self[(1, 0)] * (self[(2, 1)] * self[(0, 2)] - self[(2, 2)] * self[(0, 1)]) +
        self[(2, 0)] * (self[(0, 1)] * self[(1, 2)] - self[(1, 1)] * self[(0, 2)])
    }

    #[rustfmt::skip]
    pub fn cofactor(&self) -> Self {
        mat3![
            self[4] * self[8] - self[7] * self[5], self[5] * self[6] - self[3] * self[8], self[3] * self[7] - self[6] * self[4],
            self[7] * self[2] - self[1] * self[8], self[0] * self[8] - self[6] * self[2], self[6] * self[1] - self[0] * self[7],
            self[1] * self[5] - self[4] * self[2], self[2] * self[3] - self[5] * self[0], self[0] * self[4] - self[3] * self[1]
        ]
    }

    pub fn inverse(&self) -> Self {
        (1.0 / self.det()) * self.cofactor().transpose()
    }

    pub fn norm(&self) -> f32 {
        let mut n = 0.0;
        for e in self.elems {
            n += e * e;
        }
        n.sqrt()
    }

    pub fn trace(&self) -> f32 {
        self[0] + self[4] + self[8]
    }

    pub fn diag(&self) -> Vec3 {
        vec3![self[0], self[4], self[8]]
    }

    pub fn row(&self, i: usize) -> Vec3 {
        vec3![self[(i, 0)], self[(i, 1)], self[(i, 2)]]
    }

    pub fn col(&self, i: usize) -> Vec3 {
        vec3![self[(0, i)], self[(1, i)], self[(2, i)]]
    }
}

impl_op_ex!(+|a: &Mat3, b: &Mat3| -> Mat3 {
    let mut c = Mat3::new();
    for e in zip(&mut c.elems, zip(a.elems, b.elems)) {
        *e.0 = e.1.0 + e.1.1;
    }
    c
});

impl_op_ex!(-|a: &Mat3, b: &Mat3| -> Mat3 {
    let mut c = Mat3::new();
    for e in zip(&mut c.elems, zip(a.elems, b.elems)) {
        *e.0 = e.1 .0 - e.1 .1;
    }
    c
});

impl_op_ex!(*|a: &Mat3, b: &Mat3| -> Mat3 {
    let mut c = Mat3::new();
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                c[(i, j)] += a[(i, k)] * b[(k, j)];
            }
        }
    }
    c
});

impl_op_ex!(*|m: &Mat3, v: &Vec3| -> Vec3 {
    let mut v1: Vec3 = Default::default();
    for i in 0..3 {
        v1[i] = m.row(i) | v;
    }
    v1
});

impl_op_ex_commutative!(*|m: &Mat3, s: &f32| -> Mat3 {
    let mut m1: Mat3 = Default::default();
    for e in zip(&mut m1.elems, m.elems) {
        *e.0 = s * e.1;
    }
    m1
});

impl_op_ex!(*|v1: &Vec3, v2: &Vec3| -> Mat3 {
    let mut m: Mat3 = Default::default();
    for i in 0..3 {
        for j in 0..3 {
            m[(i, j)] = v1[i] * v2[j];
        }
    }
    m
});

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_mult() {
        let a = Mat3::eye();
        let b = Mat3::eye();
        let c = a * b;
        assert_eq!(c, Mat3::eye());
    }

    #[test]
    fn mul_eye3_vec3() {
        let m = Mat3::eye();
        let v = vec3![1.0, 2.0, 3.0];
        let v1 = m * v;
        assert_eq!(v1, v);
    }

    #[test]
    fn determinant() {
        let m = Mat3::eye();
        assert_eq!(m.det(), 1.0);

        let m = mat3![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        assert_eq!(m.det(), 0.0);

        let m = mat3![6.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        assert_eq!(m.det(), -15.0);

        let m = mat3![-5.0, -2.0, 2.0, 0.0, 1.0, 3.0, -5.0, -7.0, -9.0];
        assert_eq!(m.det(), -20.0);
    }

    #[test]
    fn cofactor() {
        let m1 = mat3![1.0, 3.0, 1.0, 1.0, 1.0, 2.0, 2.0, 3.0, 4.0];
        let m2 = mat3![-2.0, 0.0, 1.0, -9.0, 2.0, 3.0, 5.0, -1.0, -2.0];
        assert_eq!(m1.cofactor(), m2);
    }

    #[test]
    fn inverse() {
        let m = mat3![-5.0, -2.0, 2.0, 0.0, 1.0, 3.0, -5.0, -7.0, -9.0];
        assert!((m.inverse() * m - Mat3::eye()).norm() < 1e-6);
    }
}
