use std::fs::File;
use std::io::{BufWriter, Write};

#[derive(Debug)]
pub struct Image {
    pub width: u16,
    pub height: u16,
    data: Vec<[u8; 4]>,
}

impl Image {
    pub fn new(width: u16, height: u16) -> Self {
        let size = width as usize * height as usize;
        let mut image = Image {
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

    pub fn put_pixel(&mut self, x: u16, y: u16, color: &[u8; 4]) {
        debug_assert!((x < self.width) && (y < self.height));
        self.data[y as usize * self.width as usize + x as usize] = *color;
    }

    pub fn save_ppm(&self, filepath: &str) -> std::io::Result<()> {
        let mut f = BufWriter::new(File::create(filepath)?);

        write!(f, "P3\n{} {}\n255\n\n", self.width, self.height)?;

        for &pixel in &self.data {
            writeln!(f, "{} {} {}", pixel[0], pixel[1], pixel[2])?;
        }

        Ok(())
    }
}
