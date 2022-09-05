use crate::linalg::*;
use crate::image::Image;

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
pub struct Camera {
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

    pub fn trace_rays(
        &self,
        /*transform: &Mat4, */ intersect_scene: impl Fn(&Ray) -> [u8; 4],
        ) -> Image {
        let mut img = Image::new(self.resolution.0, self.resolution.1);

        let mut ray = Ray {
            orig: Default::default(),
            //orig: xform * <Vec3 as Default>::default(),
            dir: Vec3 {
                x: 0.0,
                y: 0.0,
                z: -self.focal,
            },
        };

        // let top_left = xform * vec4![self.image_plane.left, self.image_plane.top, -focal, 1.0];
        // let top_right = xform * vec4![self.image_plane.left, self.image_plane.top, -focal, 1.0];
        // let bottom_left = xform * vec4![self.image_plane.left, self.image_plane.top, -focal, 1.0];
        // let vright = (top_right - top_left).xyz() / (img.width as f32);
        // let vbottom = (bottom_left - top_left).xyz() / (img.width as f32);

        for y in 0..img.height {
            for x in 0..img.width {
                ray.dir.x = self.image_plane.left
                    + ((x as f32) / (img.width as f32)) * self.image_plane.width();
                ray.dir.y = self.image_plane.top
                    - ((y as f32) / (img.height as f32)) * self.image_plane.height();

                // ray.dir = top_left.vec3() + (x as f32) * vright + (y as f32) * vbottom;

                let col = intersect_scene(&ray);
                img.put_pixel(x, y, &col);
            }
        }
        img
    }
}

pub struct Ray {
    pub orig: Vec3,
    pub dir: Vec3,
}

#[derive(Clone, Copy)]
pub struct Sphere {
    pub center: Vec3,
    pub radius: f32,
}

pub trait Intersectable
{
    fn intersect(&self, ray: &Ray) -> Option<f32>;
}

impl Intersectable for Sphere {
    fn intersect(&self, ray: &Ray) -> Option<f32> {
        let oc = ray.orig - self.center;
        let a = ray.dir | ray.dir;
        let b = oc | ray.dir;
        let c = (oc | oc) - self.radius * self.radius;
        let delta = b * b - a * c;

        if delta < 0.0 {
            None
        } else {
            let lambda1 = (-b - delta.sqrt()) / a;
            //let lambda1 = (-b + delta.sqrt()) / a;
            Some(lambda1)
        }
    }
}


pub struct Triangle {
    pub v0: Vec3,
    pub v1: Vec3,
    pub v2: Vec3,
}

const EPSILON: f32 = 1e-6;

impl Intersectable for Triangle {
    fn intersect(&self, ray: &Ray) -> Option<f32> {
        let e1 = self.v1 - self.v0; 
        let e2 = self.v2 - self.v0;
        let p = ray.orig ^ e2;
        let det = e1 | p;
        #[cfg(backface_culling)]
        if det < EPSILON {
            return None;
        }
        #[cfg(not(backface_culling))]
        if det.abs() < EPSILON {
            return None;
        }
    
        let inv_det = 1.0 / det;
        let t = ray.orig - self.v0;
        let u = inv_det * (t | p);
        if u < 0.0 || u > 1.0 {
            return None;
        }

        let q = t ^ e1;
        let v = inv_det * (ray.dir | q);
        if v < 0.0 || v + u > 1.0 {
            return None;
        }

        Some(inv_det * (e2 | q))
    }
}



