
pub use stm32f3;

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
