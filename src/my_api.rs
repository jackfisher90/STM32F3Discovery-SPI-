pub use stm32f3;


/// If you change the cpu speed you also need to change 
/// - the timngr register for the i2c
/// - the prescaler for the timer 
/// - the speedr prescalar for the cpu


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
