#![allow(dead_code, unused)]

pub mod image;
pub mod linalg;

#[macro_use]
extern crate impl_ops;

use rand::Rng;
use std::cmp::max;

use crate::image::*;
use crate::linalg::*;

#[derive(Debug)]
struct Rect {
    left: f32,
    right: f32,
    top: f32,
    bottom: f32,
}

impl Rect {
    pub fn new_centered(x0: f32, y0: f32, width: f32, height: f32) -> Rect {
        let w2 = width / 2.0;
        let h2 = height / 2.0;
        Rect {
            left: x0 - w2,
            right: x0 + w2,
            top: y0 + h2,
            bottom: y0 - h2,
        }
    }

    pub fn width(&self) -> f32 {
        self.right - self.left
    }

    pub fn height(&self) -> f32 {
        self.top - self.bottom
    }
}

#[derive(Debug)]
struct Camera {
    focal: f32,
    image_plane: Rect,
    resolution: (u16, u16),
}

impl Camera {
    pub fn new(focal: f32, hfov: f32, resolution: (u16, u16)) -> Camera {
        Camera {
            focal,
            image_plane: Rect::new_centered(0.0, 0.0, 2.0, 2.0),
            resolution,
        }
    }

    pub fn trace_rays(&self, intersect_scene: impl Fn(&Ray) -> [u8; 4]) -> Image {
        let mut img = Image::new(self.resolution.0, self.resolution.1);

        let mut ray = Ray {
            orig: Default::default(),
            dir: Vec3 {
                x: 0.0,
                y: 0.0,
                z: -self.focal,
            },
        };

        for y in 0..img.height {
            for x in 0..img.width {
                ray.dir.x = self.image_plane.left
                    + ((x as f32) / (img.width as f32)) * self.image_plane.width();
                ray.dir.y = self.image_plane.top
                    - ((y as f32) / (img.height as f32)) * self.image_plane.height();

                let col = intersect_scene(&ray);
                img.put_pixel(x, y, &col);
            }
        }
        img
    }
}

pub struct Ray {
    orig: Vec3,
    dir: Vec3,
}

struct Sphere {
    center: Vec3,
    radius: f32,
}

fn intersect(sphere: &Sphere, ray: &Ray) -> Option<(Vec3, f32)> {
    let oc = ray.orig - sphere.center;
    let a = ray.dir | ray.dir;
    let b = oc | ray.dir;
    let c = (oc | oc) - sphere.radius * sphere.radius;
    let delta = b * b - a * c;

    if delta < 0.0 {
        None
    } else {
        let lambda1 = (-b - delta.sqrt()) / a;
        //let lambda2 = (-b + sqrt(delta)) / a;
        Some((ray.orig + lambda1 * ray.dir, lambda1))
    }
}

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
        x: 5.0,
        y: 15.0,
        z: 0.0,
    };
    const NUM_SPHERES: u32 = 10;

    let mut rng = rand::thread_rng();

    let mut spheres = Vec::new();
    for i in 0..NUM_SPHERES {
        let x = rng.gen_range(-10.0..10.0);
        let z = rng.gen_range(-10.0..-5.0);
        let radius = rng.gen_range(0.5..2.0);
        let color = rng.gen::<[f32; 3]>();
        spheres.push((
            Sphere {
                center: Vec3 { x, y: radius, z },
                radius,
            },
            color,
        ));
    }
    let image = camera.trace_rays(|ray| {
        let mut lambda_closest: f32 = f32::MAX;
        let mut p_closest: Vec3 = Default::default();
        let mut n_closest: Vec3 = Default::default();
        let mut color = [1.0, 1.0, 1.0];
        for sphere in &spheres {
            match intersect(&sphere.0, ray) {
                Some((p, lambda)) => {
                    if (lambda < lambda_closest) {
                        lambda_closest = lambda;
                        p_closest = p;
                        n_closest = (p - sphere.0.center).normalize();
                        color = sphere.1;
                    }
                }
                None => (),
            }
        }
        if (lambda_closest < f32::MAX) {
            let l = (LIGHT - p_closest).normalize();
            let alb = (l | n_closest).max(0.0);
            [
                (alb * color[0] * 255.0) as u8,
                (alb * color[1] * 255.0) as u8,
                (alb * color[2] * 255.0) as u8,
                255,
            ]
        } else {
            [100, 100, 100, 255]
        }
    });

    //println!("camera = {:?}", camera);

    image.save_ppm("test.ppm").expect("error");
}
