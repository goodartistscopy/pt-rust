use std::fmt;
use std::collections::VecDeque;
use std::cell::Cell;
use std::sync::{Arc, Mutex};
use std::thread;

use crate::linalg::*;
use crate::image::{Image, LinearImage, TiledImage};

pub use super::vec3;

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
        ) -> LinearImage {
        let mut img = LinearImage::new(self.resolution.0, self.resolution.1);


        let mut ray = Ray::new(Default::default(), vec3![0.0, 0.0, -self.focal]);

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

                ray.update_inverse_dir();
                    
                let col = intersect_scene(&ray);
                img.put_pixel(x, y, &col);
            }
        }
        img
    }

    pub fn trace_rays_tiled(
        &self, tile_size: u16,
        /*transform: &Mat4, */ intersect_scene: impl Fn(&Ray) -> [u8; 4],
        ) -> TiledImage {
        let mut img = TiledImage::new(self.resolution.0, self.resolution.1, tile_size);


        let mut ray = Ray::new(Default::default(), vec3![0.0, 0.0, -self.focal]);

        // let top_left = xform * vec4![self.image_plane.left, self.image_plane.top, -focal, 1.0];
        // let top_right = xform * vec4![self.image_plane.left, self.image_plane.top, -focal, 1.0];
        // let bottom_left = xform * vec4![self.image_plane.left, self.image_plane.top, -focal, 1.0];
        // let vright = (top_right - top_left).xyz() / (img.width as f32);
        // let vbottom = (bottom_left - top_left).xyz() / (img.width as f32);

        let (width, height) = img.get_size();
        for ty in (0..height).step_by(tile_size.into()) {
            for tx in (0..width).step_by(tile_size.into()) {
                for y in ty..ty+tile_size {
                    for x in tx..tx+tile_size {
                        ray.dir.x = self.image_plane.left
                            + ((x as f32) / (width as f32)) * self.image_plane.width();
                        ray.dir.y = self.image_plane.top
                            - ((y as f32) / (height as f32)) * self.image_plane.height();

                        // ray.dir = top_left.vec3() + (x as f32) * vright + (y as f32) * vbottom;

                        ray.update_inverse_dir();

                        let col = intersect_scene(&ray);
                        img.put_pixel(x, y, &col);
                    }}
            }
        }
        img
    }

    pub fn trace_rays_tiled_mt(
        &self, tile_size: u16,
        intersect_scene: &(impl Fn(&Ray) -> [u8; 4] + std::marker::Sync),
        ) -> TiledImage {

        struct TileScheduler
        {
            next_tile: Mutex::<Option::<(u16, u16)>>,
            num_tiles: (u16, u16)
        };

        impl TileScheduler {
            pub fn new(num_tiles: (u16, u16)) -> Self
            {
                Self { next_tile: Mutex::new(Some((0,0))), num_tiles }
            }

            pub fn get_next(&self) -> Option::<(u16,u16)> {
                let mut tile = self.next_tile.lock().unwrap();
                let cur_tile = *tile;
                *tile = match *tile {
                    None => None,
                    Some((x,y)) if x + 1 < self.num_tiles.0 => Some((x+1, y)),
                    Some((x,y)) if y + 1 < self.num_tiles.1 => Some((0, y+1)),
                    _ => None
                };
                cur_tile
            }
        }

        let num_threads = thread::available_parallelism().map_or(2, |x| { x.get() });
        let img = Arc::new(Mutex::new(TiledImage::new(self.resolution.0, self.resolution.1, tile_size)));

        let (num_tiles, width, height) = {
            let img = img.lock();
            (((img.as_ref().unwrap().get_size().0 + tile_size - 1) / tile_size,
              (img.as_ref().unwrap().get_size().1 + tile_size - 1) / tile_size),
             img.as_ref().unwrap().get_size().0,
             img.as_ref().unwrap().get_size().1)
        };

        println!("Using {} threads to process {} tiles", num_threads, num_tiles.0 * num_tiles.1);

        let sched = Arc::new(TileScheduler::new(num_tiles));
        thread::scope(|s| {
            for i in 0..num_threads {
                let img = Arc::clone(&img);
                let sched = Arc::clone(&sched);
                s.spawn(move ||{
                    let mut ray = Ray::new(Default::default(), vec3![0.0, 0.0, -self.focal]);
                    while let Some((tx, ty)) = sched.get_next() {
                        //println!("#{} | tile = {},{}", i, tx, ty);
                        let x0 = tx * tile_size;
                        let y0 = ty * tile_size;
                        for y in 0..tile_size {
                            for x in 0..tile_size {

                                ray.dir.x = self.image_plane.left
                                    + (((x0 + x) as f32) / (width as f32)) * self.image_plane.width();
                                ray.dir.y = self.image_plane.top
                                    - (((y0 + y) as f32) / (height as f32)) * self.image_plane.height();

                                ray.update_inverse_dir();

                                let col = intersect_scene(&ray);

                                img.lock().unwrap().put_pixel_from_tile(tx, ty, x, y, &col);
                            }
                        }
                    }
                });
            }
        });

        let x = img.lock().unwrap().clone(); x
    }
}

pub struct Ray {
    pub orig: Vec3,
    pub dir: Vec3,
    one_over_dir: Vec3
}

impl Ray {
    pub fn new(orig: Vec3, dir: Vec3) -> Ray
    {
        Ray { orig, dir, one_over_dir: 1.0 / dir }
    }

    fn update_inverse_dir(&mut self)
    {
        self.one_over_dir = 1.0 / self.dir;
    }
}

#[derive(Clone, Copy)]
pub struct Sphere {
    pub center: Vec3,
    pub radius: f32,
}

pub trait Intersectable : Sync
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

#[derive(Clone, Copy, Debug)]
pub struct Triangle {
    pub v0: Vec3,
    pub v1: Vec3,
    pub v2: Vec3,
    center: Vec3
}

impl Triangle {
    const EPSILON: f32 = 1e-10; // XXX move to intersectable ? (would still be specializable)

    pub fn new(v0: Vec3, v1: Vec3, v2: Vec3) -> Self {
        let center = (v0 + v1 + v2) / 3.0;
        Triangle { v0, v1, v2, center }
    }
}



impl Intersectable for Triangle {
    fn intersect(&self, ray: &Ray) -> Option<f32> {
        let e1 = self.v1 - self.v0; 
        let e2 = self.v2 - self.v0;
        let p = ray.dir ^ e2;
        let det = e1 | p;
        #[cfg(backface_culling)]
        if det < Self::EPSILON {
            return None;
        }
        #[cfg(not(backface_culling))]
        if det.abs() < Self::EPSILON {
            return None;
        }
    
        let inv_det = 1.0 / det;
        let t = ray.orig - self.v0;
        let u = inv_det * (t | p);
        if u < 0.0 || u > 1.0 {
            return None
        }

        let q = t ^ e1;
        let v = inv_det * (ray.dir | q);
        if v < 0.0 || v + u > 1.0 {
            return None;
        }

        Some(inv_det * (e2 | q))
    }
}

#[derive(Clone, Copy)]
pub struct AABB
{
    pub min: Vec3,
    pub max: Vec3
}

impl Default for AABB
{
    fn default() -> Self {
        AABB {
            min: vec3!(f32::MAX),
            max: vec3!(f32::MIN)
        }
    }
}

impl Intersectable for AABB {
    fn intersect(&self, ray: &Ray) -> Option<f32> {
        let tx1 = (self.min.x - ray.orig.x) * ray.one_over_dir.x;
        let tx2 = (self.max.x - ray.orig.x) * ray.one_over_dir.x;
        let mut tmin = tx1.min(tx2);
        let mut tmax = tx1.max(tx2);
        let ty1 = (self.min.y - ray.orig.y) * ray.one_over_dir.y;
        let ty2 = (self.max.y - ray.orig.y) * ray.one_over_dir.y;
        tmin = tmin.max(ty1.min(ty2));
        tmax = tmax.min(ty1.max(ty2));
        let tz1 = (self.min.z - ray.orig.z) * ray.one_over_dir.z;
        let tz2 = (self.max.z - ray.orig.z) * ray.one_over_dir.z;
        tmin = tmin.max(tz1.min(tz2));
        tmax = tmax.min(tz1.max(tz2));
        if tmax >= tmin && tmax > 0.0 { // && tmin < ray.t
            Some(tmin)
        }
        else {
            None
        }
    }
}

#[derive(Clone, Copy)]
pub enum BVHNodeContent {
    Internal { first_child: u32 },
    Leaf { offset: u32, count: u32 }
}

// Experimented with this smaller node (32 bytes vs 36)
// But the friendlier cache utilisation does not translate
// into eithr faster builds nor travversals
// #[derive(Clone, Copy)]
// pub struct BVHNodeCompactContent {
//     tri_or_child_offset: u32,
//     tri_count: u32
// }

#[derive(Clone, Copy)]
pub struct BVHNode
{
    pub bounds: AABB,
    pub content: BVHNodeContent
}

pub struct BVH<'a> {
    triangles: &'a [Triangle], 
    triangle_ids: Vec::<usize>,
    nodes: Vec::<BVHNode>,
    pub depth: u32
}

impl <'a> BVH<'a>
{
    const ROOT_NODE:usize = 0;

    pub fn new(triangles: &'a mut[Triangle]) -> Self
    {
        let num_triangles = triangles.len();
        let mut bvh = BVH { triangles, triangle_ids: (0..num_triangles).into_iter().collect(), nodes: Vec::with_capacity(2 * num_triangles), depth: 1 };
        let mut root = BVHNode { bounds : Default::default(), content: BVHNodeContent::Leaf { offset: 0, count: num_triangles as u32}};

        bvh.update_bounds(&mut root);
        bvh.nodes.push(root);
        bvh.subdivide(Self::ROOT_NODE as u32, 1);

        bvh
    }

    #[inline]
    fn update_bounds(&self, node: &mut BVHNode) {
        if let BVHNodeContent::Leaf{ offset, count } = node.content {
            for idx in offset..offset+count {
                let idx = idx as usize;
                node.bounds.min = node.bounds.min.min(&self.triangles[self.triangle_ids[idx]].v0);
                node.bounds.min = node.bounds.min.min(&self.triangles[self.triangle_ids[idx]].v1);
                node.bounds.min = node.bounds.min.min(&self.triangles[self.triangle_ids[idx]].v2);
                node.bounds.max = node.bounds.max.max(&self.triangles[self.triangle_ids[idx]].v0);
                node.bounds.max = node.bounds.max.max(&self.triangles[self.triangle_ids[idx]].v1);
                node.bounds.max = node.bounds.max.max(&self.triangles[self.triangle_ids[idx]].v2);
            }
        }
    }

    fn subdivide(&mut self, node_idx: u32, depth: u32)
    {
        self.depth = self.depth.max(depth);
        let node = self.nodes[node_idx as usize];
        if let BVHNodeContent::Leaf{ offset, count } = node.content {
            if (count >= 2) {
                // determine split axis
                let extent = node.bounds.max - node.bounds.min;
                let mut axis = 0;
                if extent.y > extent.x {
                    axis = 1;
                }
                if extent.z > extent[axis] {
                    axis = 2;
                }
                let split_pos = 0.5 * (node.bounds.min[axis] + node.bounds.max[axis]);

                // split the triangle list
                let mut i:usize = offset.try_into().unwrap();
                let mut j:usize = (offset + count - 1).try_into().unwrap();
                while i <= j && j > 0 {
                    if self.triangles[self.triangle_ids[i]].center[axis] <= split_pos {
                        (self.triangle_ids[j], self.triangle_ids[i]) = (self.triangle_ids[i], self.triangle_ids[j]);
                        j -= 1;
                    }
                    else {
                        i += 1;
                    }
                }

                let left_count:u32 = i as u32 - offset;
                #[cfg(debug_assertions)]
                println!("node {}: split {} at {}/{}", node_idx, "xyz".chars().nth(axis).unwrap(), left_count, count);
                
                if left_count > 0 && left_count < count {
                    // create children
                    let mut left_child = BVHNode { bounds: Default::default(), content: BVHNodeContent::Leaf { offset, count: left_count } };
                    let mut right_child = BVHNode { bounds: Default::default(), content: BVHNodeContent::Leaf { offset: offset + left_count, count: count - left_count} };
                    self.update_bounds(&mut left_child);
                    self.update_bounds(&mut right_child);
                    let next_idx:u32 = self.nodes.len() as u32;
                    self.nodes.push(left_child);
                    self.nodes.push(right_child);
                    // update the leaf node to an internal one
                    self.nodes[node_idx as usize].content = BVHNodeContent::Internal { first_child: next_idx };
                    // recurse
                    self.subdivide(next_idx, depth + 1);
                    self.subdivide(next_idx+1, depth + 1);
                }
            }
                
        }
    }

    fn intersect_node(&self, node: &BVHNode, ray: &Ray) -> Option<f32> {
        if let Some(lambda) = node.bounds.intersect(ray) {
            match node.content {
                BVHNodeContent::Internal { first_child } => {
                    let left_hit = self.intersect_node(&self.nodes[first_child as usize], ray);
                    let right_hit = self.intersect_node(&self.nodes[(first_child + 1) as usize], ray);
                    if let Some(left_lambda) = left_hit {
                        if let Some(right_lambda) = right_hit {
                            Some(left_lambda.min(right_lambda))
                        } else {
                            left_hit
                        }
                    } else {
                        right_hit
                    }
                }
                BVHNodeContent::Leaf { offset, count } => {
                    let mut closest_hit = None;
                    for i in offset..offset+count {
                        if let Some(hit) = self.triangles[self.triangle_ids[i as usize]].intersect(ray) {
                            closest_hit = Some(closest_hit.unwrap_or(f32::MAX).min(hit))
                        }
                    }
                    closest_hit
                }
            }
        } else {
            None
        }
    }

    pub fn get_aabbs_up_to_depth(&self, depth: u32) -> Vec::<AABB> {
        let mut aabbs = Vec::new();

        let mut curr_nodes = VecDeque::from([(&self.nodes[Self::ROOT_NODE], 0)]);
        while !curr_nodes.is_empty() {
            let (&BVHNode { bounds, content }, node_depth) = curr_nodes.pop_front().unwrap();
            if node_depth < depth {
                match content {
                    BVHNodeContent::Leaf { offset, count } => aabbs.push(bounds),
                    BVHNodeContent::Internal { first_child } => {
                        curr_nodes.push_back((&self.nodes[first_child as usize], node_depth + 1));
                        curr_nodes.push_back((&self.nodes[(first_child + 1) as usize], node_depth + 1));
                    }
                }
            } else if node_depth == depth {
                aabbs.push(bounds);
            } 
        }
        aabbs
    }

}

impl Intersectable for BVH<'_> {
    fn intersect(&self, ray: &Ray) -> Option<f32> {
        self.intersect_node(&self.nodes[Self::ROOT_NODE], ray)
    }
}

impl fmt::Display for BVH<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "BVH: {} nodes, depth: {}", self.nodes.len(), self.depth);
        let mut curr_nodes = VecDeque::from([(&self.nodes[Self::ROOT_NODE], Self::ROOT_NODE as u32, 0)]);
        while !curr_nodes.is_empty() {
            if let Some((node, idx, depth)) = curr_nodes.pop_front()
            {
                for _ in 0..depth {
                    write!(f, " ");
                }
                write!(f, "{}: ", idx);
                match node.content {
                    BVHNodeContent::Internal { first_child } => {
                        writeln!(f, "Internal node -> {} , {}", first_child, first_child + 1);
                        curr_nodes.push_back((&self.nodes[first_child as usize], first_child, depth + 1));
                        curr_nodes.push_back((&self.nodes[(first_child + 1) as usize], first_child + 1, depth + 1));
                    },
                    BVHNodeContent::Leaf { offset, count } =>
                    {
                        writeln!(f, "Leaf node with {} primitives", count);
                    }
                };
            }
        }
        Ok(())
    }
}
