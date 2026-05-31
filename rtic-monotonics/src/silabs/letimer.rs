//! [`Monotonic`](rtic_time::Monotonic) implementation for EFR32's 24 bit LETimer ("Low Energy Timer") peripheral.
//!
//! Always runs at a fixed rate of 32768 Hz, which is a resolution of 30.518 µs.
//!
//! # Example
//!
//! ```
//! use rtic_monotonics::silabs::prelude::*;
//!
//! // Create the type `Mono`. It will manage the LETimer peripheral,
//! // which is a 24-bit timer, powered by a 32768Hz oscillator (by default).
//! // You can optionally specify a different frequency (e.g. 8092) by passing
//! // it to the `efr32_letimer_monotonic!` macro.
//! silabs_letimer_monotonic!(Mono, 8092);
//!
//! fn init() {
//!     # // This is normally provided by the selected PAC
//!     # let peripherals = unsafe { core::mem::transmute(()) };
//!     #
//!     // Start the monotonic - passing ownership of an efr32mg22_pac object for
//!     // LETimer, and temporary access of the clock management unit.
//!     Mono::start(peripherals.letimer0_ns, peripherals.cmu_ns);
//! }
//!
//! async fn usage() {
//!     loop {
//!          // You can use the monotonic to get the time...
//!          let timestamp = Mono::now();
//!          // ...and you can use it to add a delay to this async function
//!          Mono::delay(100.millis()).await;
//!     }
//! }
//! ```

/// Common definitions and traits for using the silabs letimer monotonic
pub mod prelude {
    pub use crate::silabs;

    pub use crate::Monotonic;

    pub use crate::fugit::{self, ExtU64, ExtU64Ceil};
}

use crate::{rtic_time::timer_queue::TimerQueue, TimerQueueBackend};
use efr32mg22_pac::*;

/// Timer implementing [`TimerQueueBackend`].
pub struct TimerBackend;

const U24_MAX: u32 = 16_777_215;

impl TimerBackend {
    /// Starts the monotonic timer.
    ///
    /// **Do not use this function directly.**
    ///
    /// Use the prelude macros instead.
    pub fn _start(timer: Letimer0Ns, cmu: &CmuNs, tick_rate_hz: u32) {
        // enable required bus clock
        cmu.clken0().modify(|_, w| w.letimer0().set_bit());

        // configure prescaling
        timer.ctrl().write(|w| match tick_rate_hz {
            32_768 => w.cntpresc().div1(),
            16_384 => w.cntpresc().div2(),
            8_192 => w.cntpresc().div4(),
            4_096 => w.cntpresc().div8(),
            2_048 => w.cntpresc().div16(),
            1_024 => w.cntpresc().div32(),
            512 => w.cntpresc().div64(),
            256 => w.cntpresc().div128(),
            _ => ::core::panic!("Timer cannot run at desired tick rate!"),
        });

        // enable timer and interrupts
        timer.en().write(|w| w.en().set_bit());
        timer.cmd().write(|w| w.start().set_bit());
        timer.ien().modify(|_, w| w.comp0().set_bit());

        TIMER_QUEUE.initialize(Self {});

        unsafe {
            // TODO
            // set_monotonic_prio(efr32mg22_pac::NVIC_PRIO_BITS, Interrupt::LETIMER0);
            NVIC::unmask(Interrupt::LETIMER0);
        }
    }

    fn letimer() -> &'static letimer0_ns::RegisterBlock {
        unsafe { &*Letimer0Ns::ptr() }
    }
}

static TIMER_QUEUE: TimerQueue<TimerBackend> = TimerQueue::new();

impl TimerQueueBackend for TimerBackend {
    type Ticks = u32;

    fn now() -> Self::Ticks {
        let timer = Self::letimer();
        // The LEtimer is counting downwards, from U24_MAX to 0

        let now = timer.cnt().read().cnt().bits();
        if now == 0 {
            0 // timer not started yet
        } else {
            U24_MAX - now
        }
    }

    fn set_compare(instant: Self::Ticks) {
        Self::letimer()
            .comp0()
            .write(|w| unsafe { w.comp0().bits(U24_MAX - instant) });
    }

    fn clear_compare_flag() {
        // unfortunately the LETIMER0_IF_CLR flag is not part of the SVD/PAC
        let letimer0_if_clear = (Letimer0Ns::ptr() as u32 + 0x2034) as *mut u32;
        unsafe {
            letimer0_if_clear.write_volatile(0b1 /* CC0 clear is at bit 0 */)
        };

        // disable compare
        Self::letimer()
            .comp0()
            .write(|w| unsafe { w.comp0().bits(0) });
    }

    fn pend_interrupt() {
        NVIC::pend(Interrupt::LETIMER0);
    }

    fn timer_queue() -> &'static TimerQueue<Self> {
        &TIMER_QUEUE
    }
}

/// Create an EFR32 LETIMER based monotonic and register the necessary interrupt for it.
///
/// See [`crate::monotonic`] for more details.
///
/// # Arguments
///
/// * `name` - The name that the monotonic type will have.
#[macro_export]
macro_rules! silabs_letimer_monotonic {
    ($name:ident) => {
        $crate::silabs_letimer_monotonic!($name, 32_768);
    };
    ($name:ident, $tick_rate_hz:expr) => {
        use $crate::{fugit, rtic_time};

        /// A `Monotonic` based on the EFR32's LETIMER peripheral.
        pub struct $name;

        impl $name {
            /// Starts the `Monotonic`.
            ///
            /// This method must be called only once.
            pub fn start(timer: efr32mg22_pac::Letimer0Ns, cmu: &efr32mg22_pac::CmuNs) {
                #[no_mangle]
                #[allow(non_snake_case)]
                unsafe extern "C" fn LETIMER0() {
                    use $crate::silabs::letimer::TimerQueueBackend;
                    $crate::silabs::letimer::TimerBackend::timer_queue().on_monotonic_interrupt();
                }

                $crate::silabs::letimer::TimerBackend::_start(timer, cmu, $tick_rate_hz);
            }
        }

        impl $crate::TimerQueueBasedMonotonic for $name {
            type Backend = $crate::silabs::letimer::TimerBackend;
            type Instant = fugit::Instant<
                <Self::Backend as $crate::TimerQueueBackend>::Ticks,
                1,
                { $tick_rate_hz },
            >;
            type Duration = fugit::Duration<
                <Self::Backend as $crate::TimerQueueBackend>::Ticks,
                1,
                { $tick_rate_hz },
            >;
        }

        rtic_time::impl_embedded_hal_delay_fugit!($name);
        rtic_time::impl_embedded_hal_async_delay_fugit!($name);
    };
}
