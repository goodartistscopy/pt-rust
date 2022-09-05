#![allow(dead_code, unused)]


#[macro_use]
extern crate impl_ops;

pub mod image;
pub mod linalg;
pub mod geometry;

use rand::Rng;
use std::cmp::max;

use crate::image::*;
use crate::linalg::*;
use crate::geometry::*;

fn main() {
    let camera = Camera::new(2.0, 30.0, (512, 512));
    let sphere = Sphere {
        center: Vec3 {
            x: 0.0,
            y: 0.0,
            z: -3.0,
        },
        radius: 1.0,
    };

    const LIGHT: Vec3 = Vec3 {
        x: 1.0,
        y: 2.0,
        z: 3.0,
    };
    const NUM_SPHERES: u32 = 10;

    let mut rng = rand::thread_rng();

    struct ColoredSphere(Sphere, [f32;3]);
    let mut spheres = Vec::new();
    for i in 0..NUM_SPHERES {
        let x = rng.gen_range(-10.0..10.0);
        let z = rng.gen_range(-15.0..-10.0);
        let radius = rng.gen_range(0.5..2.0);
        let color = rng.gen::<[f32; 3]>();
        spheres.push(
            ColoredSphere(
                Sphere {
                    center: Vec3 {
                        x,
                        y: radius - 1.0,
                        z,
                    },
                    radius,
                },
                color,
                ));
    }
    let image = camera.trace_rays(|ray| {
        let mut lambda_closest: f32 = f32::MAX;
        let mut sphere_closest = None;
        for sphere in &spheres {
            if let Some(lambda) = sphere.0.intersect(ray) {
                if (lambda < lambda_closest) {
                    lambda_closest = lambda;
                    sphere_closest = Some(sphere);
                }
            }
        }

        let mut color = [1.0, 1.0, 1.0];
        if let (Some(ColoredSphere(sphere,color)), true) = (sphere_closest, lambda_closest < f32::MAX){
            let hit = ray.orig + lambda_closest * ray.dir;
            let hit_normal = (hit - sphere.center).normalize();
            let l = (LIGHT - hit).normalize();
            let alb = (l | hit_normal).max(0.0);
            [
                (alb * color[0] * 255.0) as u8,
                (alb * color[1] * 255.0) as u8,
                (alb * color[2] * 255.0) as u8,
                255,
            ];
            let n = (hit_normal + 1.0) / 2.0;
            [ (n.x * 255.0) as u8, (n.y * 255.0) as u8, (n.z * 255.0) as u8, 255]
        }
        else
        {
            [100, 100, 100, 255]
        }
    });

    image.save_ppm("test.ppm").expect("error");
}
