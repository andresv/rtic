//! [`Monotonic`](rtic_time::Monotonic) implementation for EFR32's RTCC ("Real Time Clock with Capture") peripheral.
//!
//! A 32-bit counter clocked at 32.768 kHz (~30.5 µs resolution) that keeps running in EM2+ deep sleep.
//!
//! RTCC is only present on the EFR32MG22.
//!
//! # Example
//!
//! ```ignore
//! use rtic_monotonics::silabs::rtcc::prelude::*;
//!
//! // Create the type `Mono`, running at 32_768 Hz.
//! silabs_rtcc_monotonic!(Mono);
//!
//! fn init() {
//!     // Start the monotonic, passing the RTCC and CMU register blocks.
//!     Mono::start(silabs_metapac::RTCC, &silabs_metapac::CMU);
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

/// Common definitions and traits for using the silabs rtcc monotonic
pub mod prelude {
    pub use crate::silabs_rtcc_monotonic;

    pub use crate::silabs;

    pub use crate::Monotonic;

    pub use crate::fugit::{self, ExtU64, ExtU64Ceil};
}

use crate::{rtic_time::timer_queue::TimerQueue, set_monotonic_prio, silabs::NVIC_PRIO_BITS};
use cortex_m::peripheral::NVIC;
use portable_atomic::{AtomicU64, Ordering};
use rtic_time::half_period_counter::calculate_now;
use silabs_metapac::{
    rtcc_v1::vals::{Cc0CtrlMode, Cc1CtrlMode},
    Interrupt, RTCC,
};

// Re-exported so the `silabs_rtcc_monotonic!` macro can name them from a user crate.
pub use crate::TimerQueueBackend;
pub use silabs_metapac::{cmu_v1::Cmu, rtcc_v1::Rtcc};

const HALF_PERIOD: u32 = 0x8000_0000;

/// Timer implementing [`TimerQueueBackend`].
pub struct TimerBackend;

static HALF_PERIODS: AtomicU64 = AtomicU64::new(0);
static TIMER_QUEUE: TimerQueue<TimerBackend> = TimerQueue::new();

/// CC1 compare value for `instant`.
/// 0 (next wrap) if it is in the past or more than one period away.
fn compute_compare_value(instant: u64, now: u64) -> u32 {
    if u32::try_from(instant.wrapping_sub(now)).is_ok() {
        instant as u32
    } else {
        0
    }
}

impl TimerBackend {
    /// Starts the monotonic timer.
    ///
    /// **Do not use this function directly.**
    ///
    /// Use the prelude macros instead.
    pub fn _start(timer: Rtcc, cmu: &Cmu) {
        cmu.clken0().modify(|w| w.set_rtcc(true));

        timer.en().write(|w| w.set_en(true));

        // CC0 marks the half-period, CC1 is the alarm; the counter free-runs
        // over the full 32-bit range and raises OF on wrap.
        timer
            .cc0_ctrl()
            .write(|w| w.set_mode(Cc0CtrlMode::Outputcompare));
        timer.cc0_ocvalue().write(|w| w.set_oc(HALF_PERIOD));
        timer
            .cc1_ctrl()
            .write(|w| w.set_mode(Cc1CtrlMode::Outputcompare));

        timer.cmd().write(|w| w.set_start(true));

        // Clear stale flags and seed the half-period parity from the count.
        timer.if_clr().write(|w| {
            w.set_of(true);
            w.set_cc0(true);
            w.set_cc1(true);
        });
        let cnt = timer.cnt().read().cnt();
        HALF_PERIODS.store(u64::from(cnt >= HALF_PERIOD), Ordering::SeqCst);

        TIMER_QUEUE.initialize(Self {});

        // OF + CC0 drive the time base; CC1 (alarm) is toggled by enable/disable_timer.
        timer.ien().write(|w| {
            w.set_of(true);
            w.set_cc0(true);
        });

        unsafe {
            set_monotonic_prio(NVIC_PRIO_BITS, Interrupt::RTCC);
            NVIC::unmask(Interrupt::RTCC);
        }
    }

    fn rtcc() -> &'static Rtcc {
        &RTCC
    }
}

impl TimerQueueBackend for TimerBackend {
    type Ticks = u64;

    fn now() -> Self::Ticks {
        calculate_now(
            || HALF_PERIODS.load(Ordering::Relaxed),
            || Self::rtcc().cnt().read().cnt(),
        )
    }

    fn set_compare(instant: Self::Ticks) {
        let now = Self::now();
        Self::rtcc()
            .cc1_ocvalue()
            .write(|w| w.set_oc(compute_compare_value(instant, now)));
    }

    fn clear_compare_flag() {
        Self::rtcc().if_clr().write(|w| w.set_cc1(true));
    }

    fn pend_interrupt() {
        NVIC::pend(Interrupt::RTCC);
    }

    fn enable_timer() {
        Self::rtcc().ien_set().write(|w| w.set_cc1(true));
    }

    fn disable_timer() {
        Self::rtcc().ien_clr().write(|w| w.set_cc1(true));
    }

    fn on_interrupt() {
        let timer = Self::rtcc();
        let flags = timer.if_().read();
        // Full period (overflow).
        if flags.of() {
            timer.if_clr().write(|w| w.set_of(true));
            let prev = HALF_PERIODS.fetch_add(1, Ordering::Relaxed);
            assert!(prev % 2 == 1, "Monotonic must have skipped an interrupt!");
        }
        // Half period (CC0).
        if flags.cc0() {
            timer.if_clr().write(|w| w.set_cc0(true));
            let prev = HALF_PERIODS.fetch_add(1, Ordering::Relaxed);
            assert!(prev % 2 == 0, "Monotonic must have skipped an interrupt!");
        }
    }

    fn timer_queue() -> &'static TimerQueue<Self> {
        &TIMER_QUEUE
    }
}

/// Create an EFR32 RTCC based monotonic and register the necessary interrupt for it.
///
/// See [`crate::silabs::rtcc`] for more details.
///
/// # Arguments
///
/// * `name` - The name that the monotonic type will have.
#[macro_export]
macro_rules! silabs_rtcc_monotonic {
    ($name:ident) => {
        use $crate::{fugit, rtic_time};

        /// A `Monotonic` based on the EFR32's RTCC peripheral.
        pub struct $name;

        impl $name {
            /// Starts the `Monotonic`.
            ///
            /// This method must be called only once.
            pub fn start(timer: $crate::silabs::rtcc::Rtcc, cmu: &$crate::silabs::rtcc::Cmu) {
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
