
pub use stm32f3;



pub mod Compass{
    pub use crate::my_api::i2c_mod;
    pub const MAGNOMETER: u8 = 0b0011110;
    pub const CRA_REG_M: u8 = 0x00;
    pub const MR_REG_M: u8 = 0x02;
    pub const OUT_X_H_M: u8 = 0x03;

    pub struct compass{
        pub x: i32,
        pub y: i32,
        pub z: i32,
        offset_x: f32,
        offset_y: f32,
        offset_z: f32,
    }

    impl compass{
        pub fn new(my_i2c_mod: &i2c_mod::i2c_func, i2c1: &stm32f3::stm32f303::I2C1) -> Self{
            my_i2c_mod.write(&i2c1, MAGNOMETER, CRA_REG_M, 0b10011100); //set to 220 Hz, and temp sensor on
            my_i2c_mod.write(&i2c1, MAGNOMETER, MR_REG_M, 0x00); //continious conversion

            compass{
                x: 0,
                y: 0,
                z: 0,
                offset_x: 0.0,
                offset_y: 0.0,
                offset_z: 0.0,
            }
        }

        pub fn calibrate(&mut self, my_i2c_mod: &i2c_mod::i2c_func, i2c1: &stm32f3::stm32f303::I2C1) {
            let mut offset_data: [i32; 3] = [0,0,0];
            for i in 0..100{
                let temp_data = self.get_data(my_i2c_mod, i2c1);
                offset_data[0] += temp_data.0;
                offset_data[1] += temp_data.1;
                offset_data[2] += temp_data.2;
            }
            self.offset_x = offset_data[0] as f32 / 100.0;
            self.offset_y = offset_data[1] as f32 / 100.0;
            self.offset_z = offset_data[2] as f32 / 100.0;
        }

        pub fn get_data(&mut self, i2c: &i2c_mod::i2c_func, i2c1: &stm32f3::stm32f303::I2C1) -> (i32, i32, i32){
            let mut data: [u8; 6] = [0,0,0,0,0,0];
            i2c.read(&i2c1, MAGNOMETER, OUT_X_H_M, 6, &mut data);

            let x_data: i32 = {let result = ((data[0] as i32) << 8) + (data[1] as i32); if result >= 32768 {-65536+result} else {result}};
            let y_data: i32 = {let result = ((data[2] as i32) << 8) + (data[3] as i32); if result >= 32768 {-65536+result} else {result}};
            let z_data: i32 = {let result = ((data[4] as i32) << 8) + (data[5] as i32); if result >= 32768 {-65536+result} else {result}};

            self.x = x_data;
            self.y = y_data;
            self.z = z_data;


            (x_data,y_data,z_data)
        }
    } 
}

pub mod Accelerometer{
    pub use crate::my_api::i2c_mod;
    const ACCELEROMETER: u8 = 0b0011001;
    const CRA_REG1_A: u8 = 0x20;
    const CTRL_REG4_A: u8 = 0x23;
    const OUT_X_H_A: u8 = 0x28 | 1<<7; // the 7th bit sets it to autoincrement

    pub struct accelerometer{
        pub x: i32,
        pub y: i32,
        pub z: i32,
        offset_x: f32,
        offset_y: f32,
        offset_z: f32,
    }

    impl accelerometer{
        pub fn new(my_i2c_mod: &i2c_mod::i2c_func, i2c1: &stm32f3::stm32f303::I2C1) -> Self{
            my_i2c_mod.write(&i2c1, ACCELEROMETER, CRA_REG1_A, 0b01110111); //turn on the accelerometer
            my_i2c_mod.write(&i2c1, ACCELEROMETER, CTRL_REG4_A, 0b00010000); //accelerometer = +/- 4G

            accelerometer{
                x: 0,
                y: 0,
                z: 0,
                offset_x: 0.0,
                offset_y: 0.0,
                offset_z: 0.0,
            }
        }

        fn calibrate(&mut self, my_i2c_mod: &i2c_mod::i2c_func, i2c1: &stm32f3::stm32f303::I2C1) {
            let mut offset_data: [i32; 3] = [0,0,0];
            for i in 0..100{
                let temp_data = self.get_data(my_i2c_mod, i2c1);
                offset_data[0] += temp_data.0;
                offset_data[1] += temp_data.1;
                offset_data[2] += temp_data.2;
            }
            self.offset_x = offset_data[0] as f32 / 100.0;
            self.offset_y = offset_data[1] as f32 / 100.0;
            self.offset_z = offset_data[2] as f32 / 100.0;
        }

        pub fn get_data(&mut self, i2c: &i2c_mod::i2c_func, i2c1: &stm32f3::stm32f303::I2C1) -> (i32, i32, i32){
            let mut data: [u8; 6] = [0,0,0,0,0,0];
            i2c.read(&i2c1, ACCELEROMETER, (OUT_X_H_A), 6, &mut data);

            let x_data: i32 = {let result = ((data[1] as i32) << 8) + (data[0] as i32); if result >= 32768 {-65536+result-self.offset_x as i32} else {result-self.offset_x as i32}};
            let y_data: i32 = {let result = ((data[3] as i32) << 8) + (data[2] as i32); if result >= 32768 {-65536+result-self.offset_y as i32} else {result-self.offset_y as i32}};
            let z_data: i32 = {let result = ((data[5] as i32) << 8) + (data[4] as i32); if result >= 32768 {-65536+result-self.offset_z as i32} else {result-self.offset_z as i32}};

            self.x = x_data;
            self.y = y_data;
            self.z = z_data;


            (x_data,y_data,z_data)
        }
    } 
}

pub mod Gyro{
    pub use crate::my_api::spi_mod;
    const WHO_AM_I: u8 = 0x0F;
    const CTRL_REG1: u8 = 0x20;
    const CTRL_REG4: u8 = 0x23;
    const OUT_TEMP: u8 = 0x26;
    const OUT_X_L: u8 = 0x28;
    const OUT_X_H: u8 = 0x29;
    const OUT_Y_L: u8 = 0x2A;
    const OUT_Y_H: u8 = 0x2B;
    const OUT_Z_L: u8 = 0x2C;
    const OUT_Z_H: u8 = 0x2D;

    pub struct gyro{
        pub x: i16,
        pub y: i16,
        pub z: i16,
        offset_x: i32,
        offset_y: i32,
        offset_z: i32,
        pub temp: i8
    }

    impl gyro{
        pub fn new(my_spi_mod: &spi_mod::spi_func, spi1: &stm32f3::stm32f303::SPI1) -> Self {
            gyro{
                x: 0,
                y: 0,
                z: 0,
                offset_x: 0,
                offset_y: 0,
                offset_z: 0,
                temp: 0,
            }
        }

        pub fn init(self, spi: &spi_mod::spi_func, spi1: &stm32f3::stm32f303::SPI1, gpioe: &stm32f3::stm32f303::GPIOE) -> Self{
            self.cs_low(&gpioe);
            spi.write_data(&spi1, CTRL_REG1, 0x3F); //power on the gyro 
            self.cs_high(&gpioe);
            
            self.cs_low(&gpioe);
            spi.write_data(&spi1, CTRL_REG4, 0x10); //power on the gyro 
            self.cs_high(&gpioe);
            
            self
        }

        pub fn who_am_i(&self, spi: &spi_mod::spi_func, spi1: &stm32f3::stm32f303::SPI1, gpioe: &stm32f3::stm32f303::GPIOE) -> u8 {
            self.cs_low(&gpioe);
            let data = spi.read_register(&spi1, WHO_AM_I);
            self.cs_high(&gpioe);
            data
        }

        pub fn read_temp(&mut self, spi: &spi_mod::spi_func, spi1: &stm32f3::stm32f303::SPI1, gpioe: &stm32f3::stm32f303::GPIOE) -> i8 {
            self.cs_low(&gpioe);
            self.temp = spi.read_register(&spi1, OUT_TEMP) as i8;
            self.cs_high(&gpioe);
            self.temp

        }


        pub fn read_data(&mut self, spi: &spi_mod::spi_func, spi1: &stm32f3::stm32f303::SPI1, gpioe: &stm32f3::stm32f303::GPIOE) -> [f32;3]{
            let mut data: [u8;6] = [0,0,0,0,0,0];
            
            self.cs_low(&gpioe);
            //spi.read_multiple_register(&spi1, OUT_X_L, &mut data);

            data[0] = spi.read_register(&spi1, OUT_X_L);
            self.cs_high(&gpioe);
            self.cs_low(&gpioe);
            data[1] = spi.read_register(&spi1, OUT_X_H);
            self.cs_high(&gpioe);
            self.cs_low(&gpioe);

            data[2] = spi.read_register(&spi1, OUT_Y_L);
            self.cs_high(&gpioe);
            self.cs_low(&gpioe);
            data[3] = spi.read_register(&spi1, OUT_Y_H);
            self.cs_high(&gpioe);
            self.cs_low(&gpioe);

            data[4] = spi.read_register(&spi1, OUT_Z_L);
            self.cs_high(&gpioe);
            self.cs_low(&gpioe);
            data[5] = spi.read_register(&spi1, OUT_Z_H);
            self.cs_high(&gpioe);


            self.x = ((data[0] as u16) + ((data[1] as u16) << 8)) as i16;
            self.y = ((data[2] as u16) + ((data[3] as u16) << 8)) as i16;
            self.z = ((data[4] as u16) + ((data[5] as u16) << 8)) as i16;

            [self.x as f32 * 0.0175, self.y as f32 * 0.0175, self.z as f32 * 0.0175]          
            
        }

        fn cs_low(&self, gpioe: &stm32f3::stm32f303::GPIOE){
            gpioe.odr.modify(|_,w| w.odr3().clear_bit());
        }

        fn cs_high(&self, gpioe: &stm32f3::stm32f303::GPIOE){
            gpioe.odr.modify(|_,w| w.odr3().set_bit());
        }

    }
}