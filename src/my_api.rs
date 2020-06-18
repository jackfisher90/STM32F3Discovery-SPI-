pub use stm32f3;


/// If you change the cpu speed you also need to change 
/// - the timngr register for the i2c
/// - the prescaler for the timer 
/// - the speedr prescalar for the cpu


pub mod led_mod{
    pub struct Led{
        led_num: u8,
        turned_on: bool,
    }

    impl Led{
        pub fn new(gpioe: &stm32f3::stm32f303::GPIOE, rcc: &stm32f3::stm32f303::RCC, led_num: u8) -> Self{
            rcc.ahbenr.modify(|_,w| w.iopeen().set_bit()); //enables clock
            match led_num{
                7 => gpioe.moder.modify(|_,w| w.moder8().bits(0b01)),
                0 => gpioe.moder.modify(|_,w| w.moder9().bits(0b01)),
                1 => gpioe.moder.modify(|_,w| w.moder10().bits(0b01)),
                2 => gpioe.moder.modify(|_,w| w.moder11().bits(0b01)),
                3 => gpioe.moder.modify(|_,w| w.moder12().bits(0b01)),
                4 => gpioe.moder.modify(|_,w| w.moder13().bits(0b01)),
                5 => gpioe.moder.modify(|_,w| w.moder14().bits(0b01)),
                6 => gpioe.moder.modify(|_,w| w.moder15().bits(0b01)),
                _ => gpioe.odr.modify(|_,w| w.odr8().set_bit()),
            };

            Led{
                led_num,
                turned_on: false,
            }

        }

        pub fn on(&mut self, gpioe: &stm32f3::stm32f303::GPIOE){
            self.turned_on = true;
            match self.led_num {
                7 => gpioe.odr.modify(|_,w| w.odr8().set_bit()),
                0 => gpioe.odr.modify(|_,w| w.odr9().set_bit()),
                1 => gpioe.odr.modify(|_,w| w.odr10().set_bit()),
                2 => gpioe.odr.modify(|_,w| w.odr11().set_bit()),
                3 => gpioe.odr.modify(|_,w| w.odr12().set_bit()),
                4 => gpioe.odr.modify(|_,w| w.odr13().set_bit()),
                5 => gpioe.odr.modify(|_,w| w.odr14().set_bit()),
                6 => gpioe.odr.modify(|_,w| w.odr15().set_bit()),
                _ => gpioe.odr.modify(|_,w| w.odr8().set_bit()),
            };

            
        }

        pub fn off(&mut self, gpioe: &stm32f3::stm32f303::GPIOE) {
            self.turned_on = false;
            match self.led_num {
                7 => gpioe.odr.modify(|_,w| w.odr8().clear_bit()),
                0 => gpioe.odr.modify(|_,w| w.odr9().clear_bit()),
                1 => gpioe.odr.modify(|_,w| w.odr10().clear_bit()),
                2 => gpioe.odr.modify(|_,w| w.odr11().clear_bit()),
                3 => gpioe.odr.modify(|_,w| w.odr12().clear_bit()),
                4 => gpioe.odr.modify(|_,w| w.odr13().clear_bit()),
                5 => gpioe.odr.modify(|_,w| w.odr14().clear_bit()),
                6 => gpioe.odr.modify(|_,w| w.odr15().clear_bit()),
                _ => gpioe.odr.modify(|_,w| w.odr8().set_bit()),
            };

            
        }

    }

    pub fn init_all(gpioe: &stm32f3::stm32f303::GPIOE, rcc: &stm32f3::stm32f303::RCC) -> [Led; 8] {
        [Led::new(gpioe, rcc,0), Led::new(gpioe, rcc,1), Led::new(gpioe, rcc,2), Led::new(gpioe, rcc,3),
         Led::new(gpioe, rcc,4), Led::new(gpioe, rcc,5), Led::new(gpioe, rcc,6), Led::new(gpioe, rcc,7)]
    }

    pub fn all_setup_led (gpioe: &stm32f3::stm32f303::GPIOE, rcc: &stm32f3::stm32f303::RCC){
        rcc.ahbenr.modify(|_,w| w.iopeen().set_bit()); //enables clock
        gpioe.moder.modify(|_,w| { 
            w.moder8().bits(0b01);
            w.moder9().bits(0b01);
            w.moder10().bits(0b01);
            w.moder11().bits(0b01);
            w.moder12().bits(0b01);
            w.moder13().bits(0b01);
            w.moder14().bits(0b01);
            w.moder15().bits(0b01)
        });
    }

    pub fn all_led_on (gpioe: &stm32f3::stm32f303::GPIOE) {
        gpioe.odr.modify(|_,w| {
            w.odr8().set_bit();
            w.odr9().set_bit();
            w.odr10().set_bit();
            w.odr11().set_bit();
            w.odr12().set_bit();
            w.odr13().set_bit();
            w.odr14().set_bit();
            w.odr15().set_bit()
        });
    }   

    pub fn all_led_off  (gpioe: &stm32f3::stm32f303::GPIOE) {
        gpioe.odr.modify(|_,w| {
            w.odr8().clear_bit();
            w.odr9().clear_bit();
            w.odr10().clear_bit();
            w.odr11().clear_bit();
            w.odr12().clear_bit();
            w.odr13().clear_bit();
            w.odr14().clear_bit();
            w.odr15().clear_bit()
        });
        }
    
}

pub mod delay_mod{
    pub struct timer{}

    impl timer{
        pub fn new(rcc: &stm32f3::stm32f303::RCC, tim6: &stm32f3::stm32f303::TIM6) -> Self {
            rcc.apb1enr.modify(|_,w| w.tim6en().set_bit()); //turns on the clock
            tim6.cr1.modify(|_,w| {
                w.opm().set_bit();//one pulse mode
                w.cen().clear_bit()
            });//disables clock during setup

            tim6.psc.modify(|_,w| w.psc().bits(35_999));
            timer{}
        }

        pub fn delay(&self, tim6: &stm32f3::stm32f303::TIM6, delay: u16){
            tim6.arr.write(|w| unsafe{w.arr().bits(delay*2)}); //set the delay amount //not sure why unsafe
            tim6.cr1.modify(|_,w| w.cen().set_bit()); //start timer
            while !tim6.sr.read().uif().bit_is_set() {} //wait until complete
            tim6.sr.modify(|_, w| w.uif().clear_bit()); // set status register to 0

        }
    }
}

pub mod i2c_mod{
    pub struct i2c_func{}
    
    impl i2c_func{
        pub fn new(rcc: &stm32f3::stm32f303::RCC, gpiob: &stm32f3::stm32f303::GPIOB, i2c: &stm32f3::stm32f303::I2C1) -> Self {
            rcc.ahbenr.modify(|_,w| w.iopben().set_bit()); //enable gpiob clock
            rcc.apb1enr.modify(|_,w| w.i2c1en().set_bit()); //enable i2c1 clock
            rcc.apb1rstr.modify(|_,w| w.i2c1rst().set_bit());
            rcc.apb1rstr.modify(|_,w| w.i2c1rst().clear_bit());

            //PB6 -> SCL     // PB7 -> SDA
            gpiob.moder.modify(|_,w| w.moder6().bits(0b10).moder7().bits(0b10)); //alternate function mode
            gpiob.pupdr.modify(|_,w| unsafe{w.pupdr6().bits(0b01).pupdr7().bits(0b01)}); //pull up resister
            gpiob.otyper.modify(|_,w| w.ot6().set_bit().ot7().set_bit()); //open drain output
            gpiob.ospeedr.modify(|_,w| w.ospeedr6().bits(0b11).ospeedr7().bits(0b11)); //high speed
            gpiob.afrl.modify(|_,w| w.afrl6().bits(0b0100).afrl7().bits(0b0100)); //alternate function 4

            i2c.cr1.modify(|_,w| w.pe().clear_bit());

            i2c.timingr.modify(|_,w| {
                w.presc().bits(0); // all settings from page 849 on port mapping
                w.scll().bits(122); // standard mode at 72MHz cpu and 100kHz i2c
                w.sclh().bits(37);
                w.sdadel().bits(0);
                w.scldel().bits(14)
            });

            i2c.cr1.write(|w| {
                w.anfoff().clear_bit(); //enable analogue filter
                w.nostretch().clear_bit();
                w.txie().set_bit(); //enable interrupt registers
                w.rxie().set_bit()
            });

            i2c.cr1.modify(|_,w| w.pe().set_bit()); //enable preipheral

            i2c_func{}
        }

        pub fn read(&self, i2c: &stm32f3::stm32f303::I2C1, device_address: u8, register_address: u8, request_length: u8, rx_data: &mut [u8]) {
            let mut _test_register: bool = false;
            i2c.cr2.modify(|_,w| {
                w.sadd().bits(u16::from(device_address << 1)); //set device address
                w.nbytes().bits(1); //amount of bytes to send
                w.rd_wrn().clear_bit(); //set as a read operation
                w.autoend().clear_bit()
            });

            i2c.cr2.modify(|_,w| w.start().set_bit()); //send start signal

            while i2c.isr.read().txis().bit_is_set() {} //wait for txis to register to be set

            i2c.txdr.modify(|_,w| w.txdata().bits(register_address)); // Send the address of the register that we want to read: IRA_REG_M

            while i2c.isr.read().txe().bit_is_clear() {} // Wait until transfer complete
        
            
            i2c.cr2.modify(|_, w| {
                w.nbytes().bits(request_length); //set 
                w.rd_wrn().set_bit();
                w.autoend().set_bit()
            });

            i2c.cr2.modify(|_,w| w.start().set_bit());
            
            for count in 0..request_length{
                // Wait until we have received the contents of the register
                while i2c.isr.read().rxne().bit_is_clear() {}
        
                // Broadcast STOP (automatic because of `AUTOEND = 1`)
                rx_data[count as usize] = i2c.rxdr.read().rxdata().bits();
            }
        }

        pub fn arduino_read(&self, i2c: &stm32f3::stm32f303::I2C1, device_address: u8, request_length: u8, rx_data: &mut [u8]) {
            let mut _test_register: bool = false;
            i2c.cr2.modify(|_,w| {
                w.sadd().bits(u16::from(device_address << 1)); //set device address
                w.nbytes().bits(0); //amount of bytes to send
                w.rd_wrn().set_bit(); //set as a read operation
                w.autoend().set_bit()
            });

            i2c.cr2.modify(|_,w| w.start().set_bit()); //send start signal

            while i2c.isr.read().txis().bit_is_set() {} //wait for txis to register to be set

            i2c.cr2.modify(|_, w| {
                w.nbytes().bits(request_length); //set 
                w.rd_wrn().set_bit();
                w.autoend().set_bit()
            });

            i2c.cr2.modify(|_,w| w.start().set_bit());

            for count in 0..request_length{
                // Wait until we have received the contents of the register
                while i2c.isr.read().rxne().bit_is_clear() {}
        
                // Broadcast STOP (automatic because of `AUTOEND = 1`)
                rx_data[count as usize] = i2c.rxdr.read().rxdata().bits();
            }

        }

        pub fn write(&self, i2c: &stm32f3::stm32f303::I2C1, device_address: u8, register_address: u8, tx_data: u8) {
            let mut _test_register: bool = false;
            i2c.cr2.modify(|_,w| {
                w.sadd().bits(u16::from(device_address << 1)); //set device address
                w.nbytes().bits(2); //amount of bytes to send
                w.rd_wrn().clear_bit(); //set as a read operation
                w.autoend().clear_bit()
            });

            i2c.cr2.modify(|_,w| w.start().set_bit()); //send start signal

            while i2c.isr.read().txis().bit_is_set() {} //wait for txis to register to be set

            i2c.txdr.modify(|_,w| w.txdata().bits(register_address)); // Send the address of the register that we want to read: IRA_REG_M

            while i2c.isr.read().txe().bit_is_clear() {} // Wait until transfer complete
        
            i2c.txdr.modify(|_,w| w.txdata().bits(tx_data)); // Send the address of the register that we want to read: IRA_REG_M

            while i2c.isr.read().txe().bit_is_clear() {
                _test_register = i2c.isr.read().nackf().bits();
            } // Wait until transfer complete
            
            i2c.cr2.modify(|_,w| w.stop().set_bit()); //send start signal
        }
    }
}

pub mod spi_mod{
    pub struct spi_func{}

    impl spi_func{
        pub fn new(rcc: &stm32f3::stm32f303::RCC, gpioa: &stm32f3::stm32f303::GPIOA, gpioe: &stm32f3::stm32f303::GPIOE, spi: &stm32f3::stm32f303::SPI1) -> Self {
            rcc.apb2enr.modify(|_,w| w.spi1en().set_bit()); //enable spi clock

            rcc.ahbenr.modify(|_,w| w.iopaen().set_bit()); // enable gpio clocks
            rcc.ahbenr.modify(|_,w| w.iopeen().set_bit());

            gpioe.moder.modify(|_,w| w.moder3().bits(0b01)); //CS pin
            gpioe.otyper.modify(|_,w| w.ot3().clear_bit());
            gpioe.odr.modify(|_,w| w.odr3().set_bit()); //bring pe3 high to disable gyro com

            // PA5 -> SCL  // PA6 -> MISO  // PA7 -> MOSI
            gpioa.moder.modify(|_,w| {
                w.moder5().bits(0b10); // output type
                w.moder6().bits(0b10);
                w.moder7().bits(0b10)});

            gpioa.otyper.modify(|_,w| {
                w.ot5().clear_bit();// push pull type
                w.ot6().clear_bit();
                w.ot7().clear_bit()});

            gpioa.ospeedr.modify(|_,w| {
                w.ospeedr5().bits(0b11);// fast mode
                w.ospeedr6().bits(0b11);
                w.ospeedr7().bits(0b11)});

            gpioa.afrl.modify(|_,w| {
                w.afrl5().bits(0b0101);//Alternate function 5
                w.afrl6().bits(0b0101);
                w.afrl7().bits(0b0101)
            });

            spi.cr2.modify(|_,w| {
                unsafe{w.ds().bits(0b0111)}; // data size = 8 bits
                w.errie().clear_bit();
                w.txeie().set_bit(); //enable interupt flags   
                w.rxneie().set_bit();
                w.ssoe().clear_bit() //set to single master mode
            });

            spi.cr1.modify(|_,w| {
                //w.bidimode().clear_bit(); //bidirectional mode
                w.ssm().set_bit(); // hardware slave management disabled
                w.ssi().set_bit(); // slave not selected    -------------------------------
                w.br().bits(0b111); // bring rate from 72MHz to 72/16MHz = 4.5MHz as gyro cant go above 10MHz
                w.mstr().set_bit(); // set to master mode
                w.cpol().set_bit(); // set to mode 3 to work for the l3gd20
                w.cpha().set_bit()
            }); 

            spi.cr1.modify(|_,w| w.spe().set_bit()); // enable spi

            spi_func{}
            
        }

        fn check_faults(&self, spi1: &stm32f3::stm32f303::SPI1) -> bool {
            if spi1.sr.read().ovr().bit_is_set(){ //checks through all the flags
                false
            }else if spi1.sr.read().modf().bit_is_set(){
                false
            }else if spi1.sr.read().crcerr().bit_is_set(){
                false
            }else if spi1.sr.read().udr().bit_is_set(){
                false
            } else {
                true
            }
        }

        pub fn write(&self, spi1: &stm32f3::stm32f303::SPI1, data: u16) {
            if self.check_faults(&spi1){
                spi1.dr.modify(|_,w| w.dr().bits(data)); //write to the dr register
                while spi1.sr.read().bsy().bit_is_set(){}
            }
        }

        pub fn read(&self, spi1: &stm32f3::stm32f303::SPI1) -> u8 {
            if self.check_faults(&spi1){
                while spi1.sr.read().bsy().bit_is_set(){}
                return (spi1.dr.read().dr().bits() >> 8) as u8 //read from the dr register
            }
            0
            
        }

        pub fn write_data(&self, spi1: &stm32f3::stm32f303::SPI1, reg_address: u8, data: u8) {
            let total_data: u16 = ((data as u16) << 8) + (reg_address as u16);
            self.write(&spi1, total_data);
        }

        pub fn read_register(&self, spi1: &stm32f3::stm32f303::SPI1, reg_address: u8) -> u8{
            self.write(&spi1, (reg_address | 0x80) as u16); //set the bit to read mode and send
            self.write(&spi1, (0x00) as u16); 
            self.read(&spi1) //read the result
        }

        pub fn read_multiple_register(&self, spi1: &stm32f3::stm32f303::SPI1, reg_address: u8, rx_data: &mut [u8]){
            self.write(&spi1, (reg_address | 0x80 | 0x40) as u16);
            for d in rx_data.iter_mut(){
                self.write(&spi1, 0x00);
                *d = self.read(&spi1);
            }

        }
    
    }
}

pub mod motors_mod{

    pub struct all_motors{
        pub motor1: motor_data,
        pub motor2: motor_data,
        pub motor3: motor_data,
        pub motor4: motor_data,

        overall_speed: u16,
    }

    pub struct motor_data{
        number: u8,
        speed: u16,
        armed: bool,
    }


    impl all_motors{
        pub fn new(rcc: &stm32f3::stm32f303::RCC, tim4: &stm32f3::stm32f303::TIM4, gpiod: &stm32f3::stm32f303::GPIOD) -> Self{
            rcc.apb1enr.modify(|_,w| w.tim4en().set_bit());
            rcc.ahbenr.modify(|_,w| w.iopden().set_bit());

            gpiod.moder.modify(|_,w| {
                w.moder12().bits(0b10); // alternate functions
                w.moder13().bits(0b10);
                w.moder14().bits(0b10);
                w.moder15().bits(0b10)
            }); 
            gpiod.otyper.modify(|_,w| {//push pull
                w.ot12().clear_bit();
                w.ot13().clear_bit();
                w.ot14().clear_bit();
                w.ot15().clear_bit()
            }); 
            gpiod.ospeedr.modify(|_,w| {
                w.ospeedr12().bits(0b11); //high speed
                w.ospeedr13().bits(0b11);
                w.ospeedr14().bits(0b11);
                w.ospeedr15().bits(0b11)
            });
            gpiod.afrh.modify(|_,w| {
                w.afrh12().bits(0b0010);//af 2
                w.afrh13().bits(0b0010);
                w.afrh14().bits(0b0010);
                w.afrh15().bits(0b0010)
            }); 

            tim4.ccmr1_output_mut().modify(|_,w| {
                w.oc1m().bits(0b0110);// pwm mode 1
                w.oc1pe().set_bit();// enable autopreload
                w.oc2m().bits(0b0110);
                w.oc2pe().set_bit()
            });

            tim4.ccmr2_output_mut().modify(|_,w| {
                w.oc3m().bits(0b0110);// pwm mode 1
                w.oc3pe().set_bit();// enable autopreload
                w.oc4m().bits(0b0110);
                w.oc4pe().set_bit()
            }); 
            tim4.cr1.modify(|_,w| w.arpe().set_bit());

            tim4.ccer.modify(|_, w| {
                w.cc1e().set_bit(); // link to pin
                w.cc1p().clear_bit(); // active high
                w.cc1np().clear_bit(); // setup as output

                w.cc2e().set_bit(); // link to pin
                w.cc2p().clear_bit(); // active high
                w.cc2np().clear_bit(); // setup as output

                w.cc3e().set_bit(); // link to pin
                w.cc3p().clear_bit(); // active high
                w.cc3np().clear_bit(); // setup as output

                w.cc4e().set_bit(); // link to pin
                w.cc4p().clear_bit(); // active high
                w.cc4np().clear_bit() // setup as output
            });

            // for pwm you need 50hz. 72,000,000 / 50 = 1,440,000
            // 72(psc) * 20,000(arr) = 1,440,000
            // this makes the total time per count 1 micor-second and time to reload 20 millisecond
            
            tim4.psc.modify(|_,w| w.psc().bits(72)); //prescale to 1MHz
            tim4.arr.modify(|_,w| w.arr().bits(20_000)); // run up to value
            tim4.ccr4.modify(|_,w| w.ccr().bits(0)); // change pwm width
            tim4.egr.write(|w| w.ug().set_bit());
            tim4.cr1.modify(|_,w| w.cen().set_bit()); // start running
            


            all_motors{
                motor1: motor_data{number: 1, speed: 0, armed: true},
                motor2: motor_data{number: 2, speed: 0, armed: true},
                motor3: motor_data{number: 3, speed: 0, armed: true},
                motor4: motor_data{number: 4, speed: 0, armed: true},
                overall_speed: 0,
            }
        }

        pub fn arm(&self, tim4: &stm32f3::stm32f303::TIM4){
            self.change_width(&tim4, 500, &self.motor1);
            self.change_width(&tim4, 500, &self.motor2);
            self.change_width(&tim4, 500, &self.motor3);
            self.change_width(&tim4, 500, &self.motor4);
        }

        pub fn change_speed(&self, tim4: &stm32f3::stm32f303::TIM4, speed: u16, motor_number: &motor_data){
            let pwm_width = (speed*10) + 1000;
            self.change_width(&tim4, pwm_width, &motor_number);
        }

        fn change_width(&self, tim4: &stm32f3::stm32f303::TIM4, width: u16, motor_number: &motor_data){
            match motor_number.number{
                1 => tim4.ccr1.modify(|_,w| w.ccr().bits(width)),
                2 => tim4.ccr2.modify(|_,w| w.ccr().bits(width)),
                3 => tim4.ccr3.modify(|_,w| w.ccr().bits(width)),
                4 => tim4.ccr4.modify(|_,w| w.ccr().bits(width)),
                _ => (), 
            }
            
            
        }
    }
}

pub mod uart_mod{
    pub struct uart_func{
        data: u8,
        pub sbus_packet: [u8; 25],
    }

    impl uart_func{
        pub fn new(rcc: &stm32f3::stm32f303::RCC, uart: &stm32f3::stm32f303::USART1, gpioa: &stm32f3::stm32f303::GPIOA) -> Self {
            rcc.ahbenr.modify(|_,w| {
                w.iopcen().set_bit(); //enable gpio E port clock
                w.dma1en().set_bit() //enable dma
            }); 

            rcc.apb2enr.modify(|_,w| w.usart1en().set_bit()); // enable usart1 clock

            rcc.apb2rstr.modify(|_,w| w.usart1rst().set_bit()); // reset the usart clock
            rcc.apb2rstr.modify(|_,w| w.usart1rst().clear_bit());

            // GPIO SETUP

            //pin pa9 -> Tx & pa10 -> Rx
            gpioa.moder.modify(|_,w| {
                w.moder9().bits(0b10); // alternate function mode 
                w.moder10().bits(0b10)
            });

            gpioa.otyper.modify(|_,w| w.ot9().clear_bit()); //output push/pull

            gpioa.ospeedr.modify(|_,w| {
                w.ospeedr9().bits(0b11)//high speed mode
            }); 

            gpioa.afrh.modify(|_,w| {
                w.afrh9().bits(0b0111); //alternate function 7
                w.afrh10().bits(0b0111)
            });

            // USART SETUP

            uart.brr.write(|w| w.brr().bits(720)); //from 72MHz to 100,000KHz for SBUS
            uart.cr1.modify(|_,w| {
                w.txeie().set_bit(); // tx interupt enable
                w.rxneie().set_bit(); // rx interrupt enable
                w.re().set_bit(); //enable Rx
                w.te().set_bit() //enable Tx
            });

            // uart.cr2.modify(|_,w| {
            //     //w.msbfirst().set_bit();
            //     //w.rxinv().set_bit(); // inverse rx isgnal 
            //     w.stop().bits(0b10) // 2 stop bits                
            // });

            uart.cr3.modify(|_,w| {
                w.ovrdis().set_bit(); // disable overrun flag
                w.eie().clear_bit(); // disable interrupt enable
                w.dmar().set_bit() // enable dma on the rx pin
            }); 

            uart.cr1.modify(|_,w| w.ue().set_bit()); //enable uart
        

            uart_func{
                data: 0,
                sbus_packet: [0;25],
            }
        }

        pub fn setup_dma(&self, dma1: &stm32f3::stm32f303::DMA1, buffer_address: u32, sbus_length: usize) {
            //DMA SETUP

            dma1.ch5.cr.modify(|_,w| {
                w.pl().bits(0b10); // priority high
                unsafe{w.psize().bits(0b00)}; // peripheral 8 bits
                w.minc().set_bit(); // memory increment
                w.circ().set_bit(); // circular mode
                w.dir().clear_bit(); // read from perihperal
                w.tcie().set_bit() // transfer complete interrupt
            });

            dma1.ch5.ndtr.modify(|_,w| w.ndt().bits(sbus_length as u16)); // 25 bytes in an sbus signal
            dma1.ch5.par.modify(|_,w| w.pa().bits(0x40013824)); // base address for usart1 rdr register from 
            dma1.ch5.mar.modify(|_,w| w.ma().bits(buffer_address)); // address of buffer for storage

            dma1.ch5.cr.modify(|_,w| w.en().set_bit());
        }

        pub fn read(&mut self, uart: &stm32f3::stm32f303::USART1) -> u8{
            while uart.isr.read().rxne().bit_is_clear(){}
            self.data = uart.rdr.read().rdr().bits() as u8;
            self.data
        }

        pub fn read_all_sbus(&mut self, uart: &stm32f3::stm32f303::USART1) -> [u8;25] {
            //uart.rqr.modify(|_,w| w.rxfrq().set_bit());
            //while uart.rdr.read().rdr().bits() != 0x0F{}
            for byte in self.sbus_packet.iter_mut(){
                while uart.isr.read().rxne().bit_is_clear(){} 
                *byte = uart.rdr.read().rdr().bits() as u8;
            }

            self.sbus_packet
        }

        pub fn send(&mut self, uart: &stm32f3::stm32f303::USART1, data: u8) {
            uart.tdr.write(|w| unsafe{w.tdr().bits(data as u16)});
            while uart.isr.read().tc().bit_is_clear(){}
        }

    }
}
