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
    let camera = Camera::new(1.5, 30.0, (512, 512));

    const LIGHT: Vec3 = Vec3 {
        x: 1.0,
        y: 2.0,
        z: 3.0,
    };
    const NUM_ITEMS: u32 = 50024;

    let mut rng = rand::thread_rng();

    struct GeomItem<'a>(Box::<dyn Intersectable + 'a>, [f32;3]);
    let mut triangles = Vec::new();
    let mut items = Vec::new();
    // for i in 0..10 {
    //     let x = rng.gen_range(-10.0..10.0);
    //     let z = rng.gen_range(-15.0..-10.0);
    //     let radius = rng.gen_range(0.5..2.0);
    //     let color = rng.gen::<[f32; 3]>();
    //     items.push(GeomItem(
    //         Box::new(Sphere { center: vec3!(x, radius -1.0, z),
    //                 radius }),
    //         color)
    //     );
    // }

    let with_bvh = true;

    const S:f32 = 2.0;
    for i in 0..NUM_ITEMS {
        let x = rng.gen_range(-5.0..5.0);
        let y = rng.gen_range(-5.0..5.0);
        let z = rng.gen_range(-20.0..-10.0);
        let d = rng.gen::<[f32; 3]>();
        let v0 = vec3!(x + S * d[0], y + S * d[1], z + S * d[2]);
        let d = rng.gen::<[f32; 3]>();
        let v1 = vec3!(x + S * d[0], y + S * d[1], z + S * d[2]);
        let d = rng.gen::<[f32; 3]>();
        let v2 = vec3!(x + S * d[0], y + S * d[1], z + S * d[2]);
        let color = rng.gen::<[f32; 3]>();
        if !with_bvh {
            items.push(GeomItem(
                    Box::new(Triangle::new(v0, v1, v2)),
                    color)
                      );
        }
        let tri = Triangle::new(v0+0.1, v1+0.1, v2+0.1);
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
        if let Some(GeomItem(item, color)) = closest_item {

            let albedo = (closest_surface.normal | (vec3![0.0, 10.0, 0.0]).normalize()).max(0.0);
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

