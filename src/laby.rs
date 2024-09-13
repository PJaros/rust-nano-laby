use core::u32::MAX as u32_MAX;
// use esp_hal::rng::Rng;
use heapless::String;
use rand::{Rng, SeedableRng};


const MAX_RX: usize = 51;
const MAX_RY: usize = 51;
// const MAX_RX: usize = 101;
// const MAX_RY: usize = 101;
const MAX_ARR_SIZE: usize = MAX_RX * MAX_RY;
const MAX_POS_SIZE: usize =
      (MAX_RX - 3) / 2
    * (MAX_RY - 3) / 2;
const MAX_STR_SIZE: usize =
      ((MAX_RX - 1) / 2 + 2)  // +2 for \r\n
    * ((MAX_RY - 1) / 2)
    * 4;                      // 4 bytes for utf-8

pub struct Laby {
    size_x: isize,
    size_y: isize,
    real_x: isize,
    real_y: isize,
    arr: [u8; MAX_ARR_SIZE],
    dirs: [isize; 4],
}

impl Laby {
    pub fn new(size_x: isize, size_y: isize, rng: impl Rng) -> Self {
        let mut li = Self {
            size_x: 0,
            size_y: 0,
            real_x: 0,
            real_y: 0,
            arr: [0; MAX_ARR_SIZE],
            dirs: [0; 4],
        };
        // li.generate(size_x, size_y, rng);
        li
    }

    pub fn generate(&mut self, size_x: isize, size_y: isize, mut rng: impl Rng) {
        let real_x: isize = size_x + 2;
        let real_y: isize = size_y + 2;
        self.size_x = size_x;
        self.size_y = size_y;
        self.real_x = real_x;
        self.real_y = real_y;
        self.dirs = [-real_x, -1_isize, 1_isize, real_x];

        for i in 0..MAX_ARR_SIZE {
            self.arr[i] = 0;
        }
        for y in 1..self.size_y + 1 {
            for x in 1..self.size_x + 1 {
                self.arr[(x + y * real_x) as usize] = 1; 
            }
        }

        let mut jumpPos = [0_usize; MAX_POS_SIZE];
        let mut jumpNum = 0_i32;
        let mut pos: isize = 2 + 2 * self.real_x;     
        self.arr[pos as usize] = 0;

        loop {
            loop {
                let mut avaiDir = [0_isize; 4];
                let mut avaiFound = 0_usize;
                for i in 0..self.dirs.len() {
                    let dir = self.dirs[i];
                    let lookPos = (pos + 2 * dir) as usize;
                    if self.arr[lookPos] != 0 {
                        avaiDir[avaiFound] = dir;
                        avaiFound += 1;
                    }
                }

                let dir = match avaiFound {
                    0 => {break;},
                    1 => avaiDir[0],
                    _ => {
                        let num = (rng.next_u32() as f32) / (u32_MAX as f32);
                        let slot = (num * avaiFound as f32) as usize;
                        avaiDir[slot]
                    }
                };
                jumpPos[jumpNum as usize] = pos as usize;
                jumpNum += 1;

                for _ in 0..2 {
                    pos += dir;
                    self.arr[pos as usize] = 0;
                }
            }
            match jumpNum {
                0 => {break;}
                _ => {
                    jumpNum -= 1;
                    pos = jumpPos[jumpNum as usize] as isize;
                }
            }
        }
    }
}

// impl core::fmt::uDisplay for Laby {

impl core::fmt::Display for Laby {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        let mut s: String<MAX_STR_SIZE> = String::new();
        let mut dxy = [0_u8; 4];
        let mut count = 0;
        for y in (1..self.size_y + 1).step_by(2) {
            for x in (1..self.size_x + 1).step_by(2) {
                let pos = (x + y * self.real_x) as isize;
                for i in 0..self.dirs.len() {
                    dxy[i] = self.arr[(pos + self.dirs[i]) as usize];
                }
                let c = match dxy {
                    [0, 0, 0, 0] => ' ',
                    [0, 0, 0, _] => '╷',
                    [0, 0, _, 0] => '╶',
                    [0, 0, _, _] => '┌',
                    [0, _, 0, 0] => '╴',
                    [0, _, 0, _] => '┐',
                    [0, _, _, 0] => '─',
                    [0, _, _, _] => '┬',
                    [_, 0, 0, 0] => '╵',
                    [_, 0, 0, _] => '│',
                    [_, 0, _, 0] => '└',
                    [_, 0, _, _] => '├',
                    [_, _, 0, 0] => '┘',
                    [_, _, 0, _] => '┤',
                    [_, _, _, 0] => '┴',
                    [_, _, _, _] => '┼',
                };
                count += 1;
                s.push(c).unwrap();
            }
            s.push('\r').unwrap();
            s.push('\n').unwrap();
            count += 2;
        }
        write!(f, "Laby, real: {}x{}, size: {}x{}, count: {}, MAX_STR_SIZE: {}\r\n{}", 
            self.real_x, self.real_y, self.size_x, self.size_y, count, MAX_STR_SIZE, s
        )
    }
}