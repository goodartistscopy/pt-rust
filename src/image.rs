use std::fs::File;
use std::io::{BufWriter, Write};

pub trait Image
{
    fn get_size(&self) -> (u16, u16);

    fn put_pixel(&mut self, x: u16, y: u16, color: &[u8; 4]);

    fn get_pixel_mut(&mut self,  x: u16, y: u16) -> &mut [u8; 4];
    
    fn get_pixel(&self,  x: u16, y: u16) -> [u8; 4];

    fn save_ppm(&self, filepath: &str) -> std::io::Result<()> {
        let mut f = BufWriter::new(File::create(filepath)?);

        let (width, height) = self.get_size();
        write!(f, "P3\n{} {}\n255\n\n", width, height)?;

        for y in 0..width {
            for x in 0..height {
                let pixel = self.get_pixel(x, y);
                writeln!(f, "{} {} {}", pixel[0], pixel[1], pixel[2])?;
            }
        }

        Ok(())
    }
}

pub struct LinearImage {
    pub width: u16,
    pub height: u16,
    data: Vec<[u8; 4]>,
}

impl LinearImage {
    pub fn new(width: u16, height: u16) -> Self {
        let size = width as usize * height as usize;
        let mut image = LinearImage {
            width,
            height,
            data: Vec::with_capacity(size),
        };
        image.data.resize(size, [0u8;4]);
        // unsafe {
        //     image.data.set_len(size);
        // }
        image
    }
}

impl Image for LinearImage {

    #[inline]
    fn get_size(&self) -> (u16, u16) {
        (self.width, self.height)
    }

    #[inline]
    fn put_pixel(&mut self, x: u16, y: u16, color: &[u8; 4]) {
        debug_assert!((x < self.width) && (y < self.height));
        self.data[y as usize * self.width as usize + x as usize] = *color;
    }

    #[inline]
    fn get_pixel(&self, x: u16, y: u16) -> [u8; 4] {
        debug_assert!((x < self.width) && (y < self.height));
        
        unsafe {
            *self.data.get_unchecked(y as usize * self.width as usize + x as usize)
        }
    }

    #[inline]
    fn get_pixel_mut(&mut self, x: u16, y: u16) -> &mut [u8; 4] {
        debug_assert!((x < self.width) && (y < self.height));
        
        unsafe {
            self.data.get_unchecked_mut(y as usize * self.width as usize + x as usize)
        }
    }
}

#[derive(Debug, Clone)]
pub struct TiledImage {
    width: u16,
    height: u16,
    logical_width: u16,
    logical_height: u16,
    num_tiles: (u16, u16), // number of tiles in each dimensions
    pub tile_size: u16,
    tile_offset: usize, // number of pixels in a tile
    data: Vec<[u8; 4]>,
}

impl TiledImage {
    pub fn new(width: u16, height: u16, tile_size: u16) -> TiledImage {
        let logical_width = width;
        let logical_height = height;
        let num_tiles = ((width + tile_size - 1) / tile_size, (height + tile_size - 1) / tile_size);
        let width = num_tiles.0 * tile_size;
        let height = num_tiles.1 * tile_size;
        let size = width as usize * height as usize;
        let tile_offset = (tile_size * tile_size).into();
        let mut image = TiledImage {
            width,
            height,
            logical_width,
            logical_height,
            num_tiles,
            tile_size,
            tile_offset,
            data: Vec::with_capacity(size),
        };
        image.data.resize(size, [0u8;4]);
        image
    }

    #[inline]
    pub fn put_pixel_from_tile(&mut self, tile_x: u16, tile_y: u16, local_x: u16, local_y: u16, color: &[u8; 4]) {
        // accessing the padding is allowed, avoiding additional tests
        debug_assert!(tile_x < self.num_tiles.0 && tile_y < self.num_tiles.1 && local_x < self.tile_size && local_y < self.tile_size);

        let tile_start: usize = (((tile_y as usize * self.num_tiles.0 as usize) + tile_x as usize) * self.tile_offset).into();
        self.data[tile_start + local_y as usize * self.tile_size as usize + local_x as usize] = *color;
    }
}

impl Image for TiledImage {
    #[inline]
    fn get_size(&self) -> (u16, u16) {
        (self.logical_width, self.logical_height)
    }

    #[inline]
    fn put_pixel(&mut self, x: u16, y: u16, color: &[u8; 4]) {
        // accessing the padding is allowed, avoiding additional tests
        debug_assert!((x < self.width) && (y < self.height));

        let (tx, ty) = (x / self.tile_size, y / self.tile_size);
        let (lx, ly) = (x % self.tile_size, y % self.tile_size);
        let tile_start: usize = (((ty as usize * self.num_tiles.0 as usize) + tx as usize) * self.tile_offset).into();
        self.data[tile_start + ly as usize * self.tile_size as usize + lx as usize] = *color;
    }

    #[inline]
    fn get_pixel(&self, x: u16, y: u16) -> [u8; 4] {
        debug_assert!((x < self.logical_width) && (y < self.logical_height));
        
        let (tx, ty) = (x / self.tile_size, y / self.tile_size);
        let (lx, ly) = (x % self.tile_size, y % self.tile_size);
        let tile_start: usize = (((ty as usize * self.num_tiles.0 as usize) + tx as usize) * self.tile_offset).into();
        unsafe {
            *self.data.get_unchecked(tile_start + ly as usize * self.tile_size as usize + lx as usize)
        }
    }

    #[inline]
    fn get_pixel_mut(&mut self, x: u16, y: u16) -> &mut [u8; 4] {
        debug_assert!((x < self.logical_width) && (y < self.logical_height));
        
        let (tx, ty) = (x / self.tile_size, y / self.tile_size);
        let (lx, ly) = (x % self.tile_size, y % self.tile_size);
        let tile_start: usize = (((ty as usize * self.num_tiles.0 as usize) + tx as usize) * self.tile_offset).into();
        unsafe {
            self.data.get_unchecked_mut(tile_start + ly as usize * self.tile_size as usize + lx as usize)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create_tiled_image() {
        let img = TiledImage::new(16,16, 4);
        assert!(img.logical_width == img.width && img.logical_height == img.height);
        assert!(img.num_tiles == (4, 4));

        let img = TiledImage::new(9, 7, 4);
        assert!(img.logical_width != img.width && img.logical_height != img.height);
        assert!(img.num_tiles == (3, 2));
    }
}
