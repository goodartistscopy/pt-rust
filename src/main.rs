#![allow(dead_code, unused)]


#[macro_use]
extern crate impl_ops;

pub mod image;
pub mod geometry;

use rand::Rng;
use std::cmp::max;

use crate::image::*;
use crate::geometry::*;

extern crate nalgebra as na;
type Vector3 = na::Vector3<f32>;
use na::vector;

fn cosine_weighted_hemisphere_sample() -> (Vector3, f32)
{
    let mut rng = rand::thread_rng();
    let ksi = rng.gen::<[f32;2]>();
    let cos_theta = (1.0f32 - ksi[0]).sqrt();
    let sin_theta = ksi[0].sqrt();
    let phi = ksi[1] * 2.0 * std::f32::consts::PI;

    let dir = vector![sin_theta * phi.cos(), sin_theta * phi.sin(), cos_theta];
    (dir, cos_theta / std::f32::consts::PI)
}

fn compute_local_frame_xform(normal: &Vector3) -> Matrix3 {
    let up = if normal.z.abs() < 0.9 { vector![0.0, 0.0, 1.0] } else { vector![1.0, 0.0, 0.0] };
    let t = up.cross(&normal).normalize();
    let b = normal.cross(&t);
    Matrix3::from_columns(&[t, b, *normal])
}

fn main() {
    let camera = Camera::new(1.5, 30.0, (512, 512));

    const LIGHT: Vector3 = vector![1.0, 2.0, 3.0];
    const NUM_ITEMS: u32 = 1000;

    let mut rng = rand::thread_rng();

    struct GeomItem<'a>(Box::<dyn Intersectable + 'a>, [f32;3]);

    let mut triangles = Vec::new();
    let mut items = Vec::new();

    for i in 0..NUM_ITEMS {
        let x = rng.gen_range(-10.0..10.0);
        let z = rng.gen_range(-15.0..-10.0);
        let radius = rng.gen_range(0.5..2.0);
        let color = rng.gen::<[f32; 3]>();
        items.push(GeomItem(
            Box::new(Sphere { center: vector!(x, radius -1.0, z),
                    radius }),
            color)
        );
    }

    let with_bvh = true;

    const S:f32 = 2.0;
    for i in 0..NUM_ITEMS {
        let x = rng.gen_range(-5.0..5.0);
        let y = rng.gen_range(-5.0..5.0);
        let z = rng.gen_range(-20.0..-10.0);
        let d = rng.gen::<[f32; 3]>();
        let v0 = vector!(x + S * d[0], y + S * d[1], z + S * d[2]);
        let d = rng.gen::<[f32; 3]>();
        let v1 = vector!(x + S * d[0], y + S * d[1], z + S * d[2]);
        let d = rng.gen::<[f32; 3]>();
        let v2 = vector!(x + S * d[0], y + S * d[1], z + S * d[2]);
        let color = rng.gen::<[f32; 3]>().map(|x| 0.2 + x * 0.8);
        if !with_bvh {
            items.push(GeomItem(
                    Box::new(Triangle::new(v0, v1, v2)),
                    color)
                      );
        }
        let tri = Triangle::new(v0 + Vector3::from_element(0.1), v1 + Vector3::from_element(0.1), v2 + Vector3::from_element(0.1));
        triangles.push(tri);
    }

    use std::time::Instant;

    // construction benchmarking
    const NUM_RUNS: u128 = 10;
    let mut sum_runs = 0;
    for i in 0..NUM_RUNS {
        let start = Instant::now();
        let bvh = Box::new(BVH::new(&mut triangles));
        let run_duration = Instant::now().duration_since(start);
        sum_runs += run_duration.as_micros();
    }
    sum_runs /= NUM_RUNS;
    println!("BVH construciton time: {} µs", sum_runs);

    let bvh = Box::new(BVH::new(&mut triangles));
    println!("BVH depth = {}", bvh.depth);
    //println!("BVH = {}", bvh);

    // draw BVH bounding boxes
    if (false) {
        let depth = bvh.depth;
        let aabbs = bvh.get_aabbs_up_to_depth(depth);
        println!("{} AABBs at depth {}", aabbs.len(), depth);
        for bbox in aabbs {
            let color = rng.gen::<[f32; 3]>(); 
            items.push(GeomItem(Box::new(bbox), color));
        }
    }
    
    if with_bvh {
        items.push(GeomItem(bvh, [1.0, 0.0, 0.0]));
    }

//    let image = camera.trace_rays_tiled(4, |ray| {
    let image = camera.trace_rays_tiled_mt(16, &|ray| {
        let mut closest_item = None;
        let mut closest_surface = Default::default();
        let mut tmin = f32::MAX;
        for item in &items {
            if let Some((t, surface_data)) = item.0.surface_query(ray) {
                if (t < tmin) {
                    closest_item = Some(item);
                    closest_surface = surface_data;
                    tmin = t;
                }
            }
        }

        const LIGHT: Vector3 = vector![0.0, 1.0, 0.0];
        if let Some(GeomItem(item, color)) = closest_item {
            let local_xform = compute_local_frame_xform(&closest_surface.normal);

            let num_shadow_samples = 100;
            let mut albedo = 0.0;
            let hit_point = ray.orig + (tmin * ray.dir) + (1e-4 * closest_surface.normal);
            for i in 0..num_shadow_samples {
                let sample = cosine_weighted_hemisphere_sample();
                let dir = local_xform * sample.0;
                let shadow_ray = Ray::new(hit_point, dir);

                use std::ptr::eq;
                let mut iter = items.iter();
                let mut blocked = false;
                let mut j = 0;
                while let Some(block_item) = iter.next() {
                    if block_item.0.hit_query(&shadow_ray).is_some() /*&& !eq(block_item.0.as_ref(), item.as_ref())*/ {
                        blocked = true;
                        break;
                    }
                    j += 1;
                }

                if (!blocked) {
                    albedo += if shadow_ray.dir.y > 0.0 { 1.0 } else { 0.0 };
                }
            }
            albedo /= num_shadow_samples as f32;

            //let albedo = (closest_surface.normal | (vector![0.0, 10.0, 0.0]).normalize()).max(0.0);
            [
                (albedo * color[0] * 255.0) as u8,
                (albedo * color[1] * 255.0) as u8,
                (albedo * color[2] * 255.0) as u8,
                255,
            ]
        }
        else
        {
            [100, 100, 100, 255]
        }
    });

    image.save_ppm("test.ppm").expect("error");
}

