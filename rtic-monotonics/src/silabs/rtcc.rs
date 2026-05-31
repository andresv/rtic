//! [`Monotonic`](rtic_time::Monotonic) implementation for EFR32's RTCC ("Real
//! Time Clock with Capture") peripheral.
//!
//! Always runs at a fixed rate of 32768 Hz, which is a resolution of 30.518 µs.
//!
//! # Example
//!
//! ```
//! use rtic_monotonics::efr32::prelude::*;
//!
//! // Create the type `Mono`. It will manage the RTCC peripheral,
//! // which is a 32768 Hz, 32-bit timer.
//! silabs_rtcc_monotonic!(Mono);
//!
//! fn init() {
//!     # // This is normally provided by the selected PAC
//!     # let peripherals = unsafe { core::mem::transmute(()) };
//!     #
//!     // Start the monotonic - passing ownership of an efr32mg22_pac object for
//!     // RTCC, and temporary access of the clock management unit.
//!     Mono::start(peripherals.rtcc_ns);
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

/// Common definitions and traits for using the silabs rtc monotonic
pub mod prelude {
    pub use crate::silabs;

    pub use crate::Monotonic;

    pub use crate::fugit::{self, ExtU32, ExtU32Ceil};
}

use crate::{rtic_time::timer_queue::TimerQueue, TimerQueueBackend};
use efr32mg22_pac::*;

/// Timer implementing [`TimerQueueBackend`].
pub struct TimerBackend;

impl TimerBackend {
    /// Starts the monotonic timer.
    ///
    /// **Do not use this function directly.**
    ///
    /// Use the prelude macros instead.
    pub fn _start(timer: RtccNs, cmu: &CmuNs) {
        // enable required bus clock
        cmu.clken0().modify(|_, w| w.rtcc().set_bit());

        // enable rtcc and interrupts
        timer.en().write(|w| w.en().set_bit());
        timer.cmd().write(|w| w.start().set_bit());
        timer.ien().write(|w| w.cc0().set_bit());

        TIMER_QUEUE.initialize(Self {});

        unsafe {
            crate::set_monotonic_prio(efr32mg22_pac::NVIC_PRIO_BITS, Interrupt::RTCC);
            NVIC::unmask(Interrupt::RTCC);
        }
    }

    fn rtcc() -> &'static rtcc_ns::RegisterBlock {
        unsafe { &*RtccNs::ptr() }
    }
}

static TIMER_QUEUE: TimerQueue<TimerBackend> = TimerQueue::new();

impl TimerQueueBackend for TimerBackend {
    type Ticks = u32;

    fn now() -> Self::Ticks {
        let timer = Self::rtcc();

        timer.cnt().read().cnt().bits()
    }

    fn set_compare(instant: Self::Ticks) {
        Self::rtcc().cc0_ctrl().write(|w| w.mode().outputcompare());

        Self::rtcc()
            .cc0_ocvalue()
            .write(|w| unsafe { w.oc().bits(instant) });
    }

    fn clear_compare_flag() {
        // unfortunately the RTCC_IF_CLR flag is not part of the SVD/PAC
        let cc0_clear_reg = (RtccNs::ptr() as u32 + 0x2014) as *mut u32;
        unsafe {
            cc0_clear_reg.write_volatile(0b10_000 /* CC0 clear is at bit 4 */)
        };

        // disable compare
        Self::rtcc().cc0_ctrl().write(|w| w.mode().off());
    }

    fn pend_interrupt() {
        NVIC::pend(Interrupt::RTCC);
    }

    fn timer_queue() -> &'static TimerQueue<Self> {
        &TIMER_QUEUE
    }
}

/// Create an EFR32 RTCC based monotonic and register the necessary interrupt for it.
///
/// See [`crate::efr32`] for more details.
///
/// # Arguments
///
/// * `name` - The name that the monotonic type will have.
#[macro_export]
macro_rules! silabs_rtc_monotonic {
    ($name:ident) => {
        use $crate::{fugit, rtic_time};

        /// A `Monotonic` based on the EFR32's RTCC peripheral.
        pub struct $name;

        impl $name {
            /// Starts the `Monotonic`.
            ///
            /// This method must be called only once.
            pub fn start(timer: efr32mg22_pac::RtccNs, cmu: &efr32mg22_pac::CmuNs) {
                #[no_mangle]
                #[allow(non_snake_case)]
                unsafe extern "C" fn RTCC() {
                    use $crate::TimerQueueBackend;
                    $crate::silabs::rtcc::TimerBackend::timer_queue().on_monotonic_interrupt();
                }

                $crate::silabs::rtcc::TimerBackend::_start(timer, cmu);
            }
        }

        impl $crate::TimerQueueBasedMonotonic for $name {
            type Backend = $crate::silabs::rtcc::TimerBackend;
            type Instant =
                fugit::Instant<<Self::Backend as $crate::TimerQueueBackend>::Ticks, 1, 32768>;
            type Duration =
                fugit::Duration<<Self::Backend as $crate::TimerQueueBackend>::Ticks, 1, 32768>;
        }

        rtic_time::impl_embedded_hal_delay_fugit!($name);
        rtic_time::impl_embedded_hal_async_delay_fugit!($name);
    };
}
