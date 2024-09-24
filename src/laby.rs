use arduino_hal::prelude::*;
use core::convert::Infallible;
use rand::Rng;
use ufmt::uWrite;

pub const MAX_RX: usize = 35;
pub const MAX_RY: usize = 35;
pub const MAX_ARR_SIZE: usize = (MAX_RX * MAX_RY).div_ceil(8);
pub const MAX_POS_SIZE: usize = (MAX_RX - 3) / 2 * (MAX_RY - 3) / 2;

pub struct Laby {
    pub size_x: isize,
    pub size_y: isize,
    pub real_x: isize,
    pub real_y: isize,
    arr: [u8; MAX_ARR_SIZE],
    pub dirs: [isize; 4],
    // max_used_jump: i32,
}

impl Laby {
    pub fn new(size_x: isize, size_y: isize) -> Self {
        let real_x: isize = size_x + 2;
        let real_y: isize = size_y + 2;

        Self {
            size_x,
            size_y,
            real_x,
            real_y: real_y,
            arr: [0; MAX_ARR_SIZE],
            dirs: [-real_x, -1_isize, 1_isize, real_x],
            // max_used_jump: 0,
        }
    }

    pub fn change_size(&mut self, size_x: isize, size_y: isize) {
        let real_x: isize = size_x + 2;
        let real_y: isize = size_y + 2;

        self.size_x = size_x;
        self.size_y = size_y;
        self.real_x = real_x;
        self.real_y = real_y;
        for i in 0..MAX_ARR_SIZE {
            self.arr[i] = 0;
        }
        self.dirs = [-real_x, -1_isize, 1_isize, real_x];
    }

    #[inline]
    pub fn get_byte_bit_pos(&self, pos: usize) -> (usize, usize) {
        (pos / 8, pos % 8)
    }

    pub fn set_0(&mut self, pos: usize) {
        let (byte_pos, bit_pos) = self.get_byte_bit_pos(pos);
        let mut byte_value = self.arr[byte_pos];
        byte_value &= !(1 << bit_pos);
        self.arr[byte_pos] = byte_value;
    }

    pub fn set_1(&mut self, pos: usize) {
        let (byte_pos, bit_pos) = self.get_byte_bit_pos(pos);
        let mut byte_value = self.arr[byte_pos];
        byte_value |= 1 << bit_pos;
        self.arr[byte_pos] = byte_value;
    }

    pub fn read(&self, pos: usize) -> bool {
        let (byte_pos, bit_pos) = self.get_byte_bit_pos(pos);
        let mut byte_value = self.arr[byte_pos];
        byte_value &= 1 << bit_pos;
        byte_value != 0
    }

    pub fn generate(&mut self, rng: &mut impl Rng) {
        for i in 0..MAX_ARR_SIZE {
            self.arr[i] = 0;
        }
        for y in 1..self.size_y + 1 {
            for x in 1..self.size_x + 1 {
                let pos = (x + y * self.real_x) as usize;
                self.set_1(pos);
            }
        }
        let mut jump_pos = [0_usize; MAX_POS_SIZE];
        let mut jump_num = 0_usize;
        let mut pos: isize = 2 + 2 * self.real_x;
        self.set_0(pos as usize);

        loop {
            loop {
                let mut avai_dir = [0_isize; 4];
                let mut avai_found = 0_usize;
                for i in 0..self.dirs.len() {
                    let dir = self.dirs[i];
                    let look_pos = (pos + 2 * dir) as usize;

                    if self.read(look_pos) == true {
                        avai_dir[avai_found] = dir;
                        avai_found += 1;
                    }
                }

                #[rustfmt::skip]
                let dir = match avai_found {
                    0 => {break;},
                    1 => avai_dir[0],
                    _ => {
                        let slot = rng.gen_range(0..avai_found);
                        jump_pos[jump_num] = pos as usize;
                        jump_num += 1;
                        avai_dir[slot]
                    }
                };

                for _ in 0..2 {
                    pos += dir;
                    self.set_0(pos as usize);
                }
            }
            #[rustfmt::skip]
            match jump_num {
                0 => {break;}
                _ => {
                    jump_num -= 1;
                    pos = jump_pos[jump_num] as isize;
                }
            }
            // self.max_used_jump = (jump_num as i32).max(self.max_used_jump);
        }
        let pos = self.size_x - 1 + self.real_x * self.size_y;
        self.set_0(pos as usize);
    }

    pub fn print<U: uWrite>(self: &Laby, serial: &mut U)
    where
        U: uWrite<Error = Infallible>,
    {
        let mut dxy: [u8; 4] = [0_u8; 4];
        for y in (1..self.size_y + 1).step_by(2) {
            for x in (1..self.size_x + 1).step_by(2) {
                let pos = (x + y * self.real_x) as isize;
                for i in 0..self.dirs.len() {
                    let look_pos = (pos + self.dirs[i]) as usize;
                    dxy[i] = match self.read(look_pos) {
                        true => 1,
                        false => 0,
                    }
                }
                let c = match dxy {
                    [0, 0, 0, 0] => " ",
                    [0, 0, 0, _] => "╷",
                    [0, 0, _, 0] => "╶",
                    [0, 0, _, _] => "┌",
                    [0, _, 0, 0] => "╴",
                    [0, _, 0, _] => "┐",
                    [0, _, _, 0] => "─",
                    [0, _, _, _] => "┬",
                    [_, 0, 0, 0] => "╵",
                    [_, 0, 0, _] => "│",
                    [_, 0, _, 0] => "└",
                    [_, 0, _, _] => "├",
                    [_, _, 0, 0] => "┘",
                    [_, _, 0, _] => "┤",
                    [_, _, _, 0] => "┴",
                    [_, _, _, _] => "┼",
                };
                ufmt::uwrite!(serial, "{}", c).unwrap_infallible();
            }
            ufmt::uwriteln!(serial, "\r").unwrap_infallible();
        }
    }
}
