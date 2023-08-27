//! Serial interface loopback test
//!
//! You have to short the TX and RX pins to make this program work

#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use embedded_graphics::{
    image::{Image, ImageRaw},
    pixelcolor::BinaryColor,
    prelude::*,
    Drawable,
};
use stm32f1xx_hal::{
    gpio,
    pac,
    pac::interrupt,
    pac::USART3,
    i2c::{BlockingI2c, DutyCycle, Mode},
    prelude::*,
    serial::{Rx, Serial, Tx},
};

static mut RX: Option<Rx<USART3>> = None;
static mut TX: Option<Tx<USART3>> = None;
static mut SCL: Option<stm32f1xx_hal::gpio::gpiob::PB8<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>>> = None;
static mut SDA: Option<stm32f1xx_hal::gpio::gpiob::PB9<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>>> = None;
static mut DISPLAY: Option<Ssd1306<I2CInterface<BlockingI2c<pac::I2C1, (gpio::Pin<'B', 8, gpio::Alternate<gpio::OpenDrain>>, gpio::Pin<'B', 9, gpio::Alternate<gpio::OpenDrain>>)>>, DisplaySize128x32, ssd1306::mode::BufferedGraphicsMode<DisplaySize128x32>>> = None;


#[entry]
unsafe fn main() -> ! {
    // Get access to the device specific peripherals from the peripheral access crate
    let p = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = p.FLASH.constrain();
    let rcc = p.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Prepare the alternate function I/O registers
    let mut afio = p.AFIO.constrain();

    // Prepare the GPIOB peripheralgoogle_images_download
    let mut gpiob = p.GPIOB.split();

    let  scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let  sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);



    let i2c = BlockingI2c::i2c1(
        p.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 400_000.Hz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        1000,
        10,
        1000,
        1000,google_images_download
    );

    
    let interface = I2CDisplayInterface::new(i2c);
    let  mut display: Ssd1306<I2CInterface<BlockingI2c<pac::I2C1, (gpio::Pin<'B', 8, gpio::Alternate<gpio::OpenDrain>>, gpio::Pin<'B', 9, gpio::Alternate<gpio::OpenDrain>>)>>, DisplaySize128x32, ssd1306::mode::BufferedGraphicsMode<DisplaySize128x32>> = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate180)
    .into_buffered_graphics_mode();
    // USART3
    let tx = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    let rx = gpiob.pb11;

    // Set up the usart device. Takes ownership over the USART register and tx/rx pins. The rest of
    // the registers are used to enable and configure the device.
    let (mut tx, mut rx) =
        Serial::new(p.USART3, (tx, rx), &mut afio.mapr, 9600.bps(), &clocks).split();
    tx.listen();
    rx.listen();
    rx.listen_idle();

    cortex_m::interrupt::free(|_| unsafe {
        TX.replace(tx);
        RX.replace(rx);
        SCL.replace(scl);
        SDA.replace(sda);
        DISPLAY.replace(display);
       
    });
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART3);
    }

    loop {
        cortex_m::asm::wfi()
        
    }
    
}
const BUFFER_LEN: usize = 256;
static mut BUFFER: &mut [u8; BUFFER_LEN] = &mut [0; BUFFER_LEN];
static mut WIDX: usize = 0;





unsafe fn write(buf: &[u8]) {
    if let Some(tx) = TX.as_mut() {
        buf.iter()
            .for_each(|w| if let Err(_err) = nb::block!(tx.write(*w)) {})
    }    
    
}



#[interrupt]
unsafe fn USART3() {
    cortex_m::interrupt::free(|_| {
        if let Some(rx) = RX.as_mut() {
            if rx.is_rx_not_empty() {
                if let Ok(w) = nb::block!(rx.read()) {
                    BUFFER[WIDX] = w;
                    WIDX += 1;
                    if WIDX >= BUFFER_LEN - 1 {
                        write(&BUFFER[..]);
                        WIDX = 0;
                       
                        let raw: ImageRaw<BinaryColor> = ImageRaw::new(BUFFER, 43);
                        let im = Image::new(&raw, Point::new(43, 0));
                        im.draw(&mut DISPLAY).unwrap();
                        DISPLAY.expect("REASON").flush().unwrap()

                        
                    }
                }
                rx.listen_idle();
            } else if rx.is_idle() {
                rx.unlisten_idle();
                write(&BUFFER[0..WIDX]);
                WIDX = 0;
            }
        }
    })
}

