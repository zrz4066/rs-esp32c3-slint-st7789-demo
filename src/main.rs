#![no_std]
#![no_main]
extern crate core;

extern crate alloc;
use alloc::boxed::Box;
use alloc::rc::Rc;
use hal::gpio::{Event, Gpio9, Input, PullUp};

use core::cell::RefCell;
use core::mem::MaybeUninit;
use critical_section::Mutex;
use display_interface_spi::SPIInterfaceNoCS;

use esp_backtrace as _;
use esp_println::println;
use hal::spi::master::{dma, Spi};
use hal::{
    clock::{ClockControl, CpuClock},
    peripherals::Peripherals,
    prelude::*,
    spi::SpiMode,
    systimer::SystemTimer,
    timer::TimerGroup,
    Delay, Rtc, IO,
};
use hal::{
    interrupt,
    peripherals::{self},
    riscv,
};

use mipidsi::Display;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

static BUTTON: Mutex<RefCell<Option<Gpio9<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));

fn init_heap() {
    const HEAP_SIZE: usize = 190 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

slint::include_modules!();

#[entry]
fn main() -> ! {
    init_heap();

    println!("esp32c3 ");
    slint::platform::set_platform(Box::new(EspBackend::default()))
        .expect("backend already initialized");

    let main_window = Recipe::new().unwrap();

    let strong = main_window.clone_strong();
    let timer = slint::Timer::default();
    timer.start(
        slint::TimerMode::Repeated,
        core::time::Duration::from_millis(1000),
        move || {
            if strong.get_counter() <= 0 {
                strong.set_counter(25);
            } else {
                strong.set_counter(0);
            }
        },
    );

    main_window.run().unwrap();

    panic!("The MCU demo should not quit");
}

#[derive(Default)]
pub struct EspBackend {
    window: RefCell<Option<Rc<slint::platform::software_renderer::MinimalSoftwareWindow>>>,
}

impl slint::platform::Platform for EspBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
            slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
        );
        self.window.replace(Some(window.clone()));
        Ok(window)
    }

    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_millis(
            SystemTimer::now() / (SystemTimer::TICKS_PER_SECOND / 1000),
        )
    }

    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        let peripherals = Peripherals::take();
        let system = peripherals.SYSTEM.split();
        let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();

        let mut rtc = Rtc::new(peripherals.RTC_CNTL);
        let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
        let mut wdt0 = timer_group0.wdt;
        let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
        let mut wdt1 = timer_group1.wdt;

        rtc.rwdt.disable();
        wdt0.disable();
        wdt1.disable();

        // Initialize the timers used for Wifi
        // ANCHOR: wifi_init
        let timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;
        

        let mut delay = Delay::new(&clocks);
        let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
        // Set GPIO2 as an output, and set its state high initially.
        let mut led = io.pins.gpio2.into_push_pull_output();

        // Set GPIO9 as an input
        let mut button = io.pins.gpio9.into_pull_up_input();

        println!("button init.");
        button.listen(Event::FallingEdge);

        // ANCHOR: critical_section
        critical_section::with(|cs| BUTTON.borrow_ref_mut(cs).replace(button));
        // ANCHOR_END: critical_section
        // ANCHOR: interrupt
        interrupt::enable(peripherals::Interrupt::GPIO, interrupt::Priority::Priority2).unwrap();
        // ANCHOR_END: interrupt
        unsafe {
            riscv::interrupt::enable();
        }

        let clk = io.pins.gpio6;
        let sdo = io.pins.gpio7;
        let cs = io.pins.gpio5;

        let spi = Spi::new_no_miso(
            peripherals.SPI2,
            clk,
            sdo,
            cs,
            60u32.MHz(),
            SpiMode::Mode0,
            &clocks,
        );
        println!("spi init.");

        let dc = io.pins.gpio4.into_push_pull_output();
        let rst = io.pins.gpio8.into_push_pull_output();

        let di = SPIInterfaceNoCS::new(spi, dc);
        let display = mipidsi::Builder::st7789(di)
            .with_display_size(240, 240)
            .with_window_offset_handler(|_| (0, 0))
            .with_framebuffer_size(240, 240)
            .with_invert_colors(mipidsi::ColorInversion::Inverted)
            .init(&mut delay, Some(rst))
            .unwrap();

        println!("display init.");
        let mut bl = io.pins.gpio13.into_push_pull_output();
        bl.set_high().unwrap();

        let size = slint::PhysicalSize::new(240, 240);

        self.window.borrow().as_ref().unwrap().set_size(size);

        let mut buffer_provider = DrawBuffer {
            display,
            buffer: &mut [slint::platform::software_renderer::Rgb565Pixel::default(); 240],
        };
        println!("Start busy loop on main");

        loop {
            slint::platform::update_timers_and_animations();

            if let Some(window) = self.window.borrow().clone() {
                window.draw_if_needed(|renderer| {
                    renderer.render_by_line(&mut buffer_provider);
                });
                if window.has_active_animations() {
                    continue;
                }
            }
            led.toggle().unwrap();
        }
    }

    fn debug_log(&self, arguments: core::fmt::Arguments) {
        println!("{}", arguments);
    }
}

struct DrawBuffer<'a, Display> {
    display: Display,
    buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}

impl<DI: display_interface::WriteOnlyDataCommand, RST: embedded_hal::digital::v2::OutputPin>
    slint::platform::software_renderer::LineBufferProvider
    for &mut DrawBuffer<'_, Display<DI, mipidsi::models::ST7789, RST>>
{
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [slint::platform::software_renderer::Rgb565Pixel]),
    ) {
        let buffer = &mut self.buffer[range.clone()];

        render_fn(buffer);

        // We send empty data just to get the device in the right window
        self.display
            .set_pixels(
                range.start as u16,
                line as _,
                range.end as u16,
                line as u16,
                buffer
                    .iter()
                    .map(|x| embedded_graphics_core::pixelcolor::raw::RawU16::new(x.0).into()),
            )
            .unwrap();
    }
}

#[interrupt]
fn GPIO() {
    critical_section::with(|cs| {
        println!("GPIO interrupt");
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();
    });
}
