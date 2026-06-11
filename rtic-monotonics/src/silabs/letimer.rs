//! [`Monotonic`](rtic_time::Monotonic) implementation for EFR32's 24-bit
//! LETimer ("Low Energy Timer") peripheral.
//!
//! Runs on the 32.768 kHz low-frequency clock (~30.5 µs resolution) and keeps
//! ticking in EM2+ deep sleep.
//!
//! # Example
//!
//! ```ignore
//! use rtic_monotonics::silabs::letimer::prelude::*;
//!
//! // Create the type `Mono`, running at 32_768 Hz. Optionally pass a different
//! // prescaler rate, e.g. 8192, as the second argument.
//! silabs_letimer_monotonic!(Mono, 8192);
//!
//! fn init() {
//!     // Start the monotonic, passing the LETIMER0 and CMU register blocks.
//!     Mono::start(silabs_metapac::LETIMER0, &silabs_metapac::CMU);
//! }
//!
//! async fn usage() {
//!     loop {
//!          // You can use the monotonic to get the time...
//!          let timestamp = Mono::now();
//!          // ...and you can use it to add a delay to async function
//!          Mono::delay(100.millis()).await;
//!     }
//! }
//! ```

/// Common definitions and traits for using the silabs letimer monotonic
pub mod prelude {
    pub use crate::silabs_letimer_monotonic;

    pub use crate::silabs;

    pub use crate::Monotonic;

    pub use crate::fugit::{self, ExtU64, ExtU64Ceil};
}

use crate::{rtic_time::timer_queue::TimerQueue, set_monotonic_prio, silabs::NVIC_PRIO_BITS};
use cortex_m::peripheral::NVIC;
use portable_atomic::{AtomicU32, Ordering};
use rtic_time::half_period_counter::{calculate_now, TimerValue};
use silabs_metapac::{Interrupt, LETIMER0};

// Re-exported so the `silabs_letimer_monotonic!` macro can name them from a user crate.
pub use crate::TimerQueueBackend;

#[cfg(feature = "silabs-efr32mg22")]
pub use silabs_metapac::{
    cmu_v1::Cmu,
    letimer_v0::{vals::Cntpresc, Letimer},
};
#[cfg(feature = "silabs-efr32mg24")]
pub use silabs_metapac::{
    cmu_v3::Cmu,
    letimer_v1::{vals::Cntpresc, Letimer},
};
#[cfg(feature = "silabs-efr32fg25")]
pub use silabs_metapac::{
    cmu_v4::Cmu,
    letimer_v1::{vals::Cntpresc, Letimer},
};
#[cfg(feature = "silabs-efr32mg26")]
pub use silabs_metapac::{
    cmu_v7::Cmu,
    letimer_v1::{vals::Cntpresc, Letimer},
};

const U24_MAX: u32 = 0x00FF_FFFF;
const HALF_PERIOD_UP: u32 = 0x0080_0000;
// COMP1 match value: fires when `U24_MAX - cnt` crosses HALF_PERIOD_UP.
const HALF_PERIOD_CNT: u32 = U24_MAX - HALF_PERIOD_UP;

/// 24-bit timer value for [`calculate_now`].
struct TimerValueU24(u32);
impl TimerValue for TimerValueU24 {
    const BITS: u32 = 24;
}
impl From<TimerValueU24> for u64 {
    fn from(value: TimerValueU24) -> Self {
        Self::from(value.0)
    }
}

/// Timer implementing [`TimerQueueBackend`].
pub struct TimerBackend;

static HALF_PERIODS: AtomicU32 = AtomicU32::new(0);
static TIMER_QUEUE: TimerQueue<TimerBackend> = TimerQueue::new();

impl TimerBackend {
    /// Starts the monotonic timer.
    ///
    /// **Do not use this function directly.**
    ///
    /// Use the prelude macros instead.
    pub fn _start(timer: Letimer, cmu: &Cmu, tick_rate_hz: u32) {
        cmu.clken0().modify(|w| w.set_letimer0(true));

        // Prescaler; repmode defaults to Free (continuous wrap).
        timer.ctrl().write(|w| match tick_rate_hz {
            32_768 => w.set_cntpresc(Cntpresc::Div1),
            16_384 => w.set_cntpresc(Cntpresc::Div2),
            8_192 => w.set_cntpresc(Cntpresc::Div4),
            4_096 => w.set_cntpresc(Cntpresc::Div8),
            2_048 => w.set_cntpresc(Cntpresc::Div16),
            1_024 => w.set_cntpresc(Cntpresc::Div32),
            512 => w.set_cntpresc(Cntpresc::Div64),
            256 => w.set_cntpresc(Cntpresc::Div128),
            _ => ::core::panic!("Timer cannot run at desired tick rate!"),
        });

        timer.en().write(|w| w.set_en(true));
        timer.comp1().write(|w| w.set_comp1(HALF_PERIOD_CNT));
        timer.cmd().write(|w| w.set_start(true));

        // CNT reads 0 across the LF clock-domain sync after START.
        // Wait it out (bounded) before sampling the parity below.
        let mut spins = 0u32;
        while timer.cnt().read().cnt() == 0 && spins < 1_000_000 {
            spins += 1;
        }

        // Clear stale flags and seed the half-period parity from the count.
        timer.if_clr().write(|w| {
            w.set_uf(true);
            w.set_comp0(true);
            w.set_comp1(true);
        });
        let up = U24_MAX - timer.cnt().read().cnt();
        HALF_PERIODS.store(u32::from(up >= HALF_PERIOD_UP), Ordering::SeqCst);

        TIMER_QUEUE.initialize(Self {});

        // UF + COMP1 drive the time base; COMP0 (alarm) is toggled by enable/disable_timer.
        timer.ien().write(|w| {
            w.set_uf(true);
            w.set_comp1(true);
        });

        unsafe {
            set_monotonic_prio(NVIC_PRIO_BITS, Interrupt::LETIMER0);
            NVIC::unmask(Interrupt::LETIMER0);
        }
    }

    fn letimer() -> &'static Letimer {
        &LETIMER0
    }
}

impl TimerQueueBackend for TimerBackend {
    type Ticks = u64;

    fn now() -> Self::Ticks {
        // Counts down: present `U24_MAX - cnt` as an up-counter to the helper.
        calculate_now(
            || HALF_PERIODS.load(Ordering::Relaxed),
            || TimerValueU24(U24_MAX - Self::letimer().cnt().read().cnt()),
        )
    }

    fn set_compare(instant: Self::Ticks) {
        let now = Self::now();
        let diff = instant.wrapping_sub(now);
        // Past or >1 period away: park at the wrap point (re-armed next half/overflow).
        let up = if diff <= u64::from(U24_MAX) {
            (instant & u64::from(U24_MAX)) as u32
        } else {
            0
        };
        // Up-counting target -> down-counter compare value.
        Self::letimer().comp0().write(|w| w.set_comp0(U24_MAX - up));
    }

    fn clear_compare_flag() {
        Self::letimer().if_clr().write(|w| w.set_comp0(true));
    }

    fn pend_interrupt() {
        NVIC::pend(Interrupt::LETIMER0);
    }

    fn enable_timer() {
        Self::letimer().ien_set().write(|w| w.set_comp0(true));
    }

    fn disable_timer() {
        Self::letimer().ien_clr().write(|w| w.set_comp0(true));
    }

    fn on_interrupt() {
        let timer = Self::letimer();
        let flags = timer.if_().read();
        // Full period (underflow).
        if flags.uf() {
            timer.if_clr().write(|w| w.set_uf(true));
            let prev = HALF_PERIODS.fetch_add(1, Ordering::Relaxed);
            assert!(prev % 2 == 1, "Monotonic must have skipped an interrupt!");
        }
        // Half period (COMP1).
        if flags.comp1() {
            timer.if_clr().write(|w| w.set_comp1(true));
            let prev = HALF_PERIODS.fetch_add(1, Ordering::Relaxed);
            assert!(prev % 2 == 0, "Monotonic must have skipped an interrupt!");
        }
    }

    fn timer_queue() -> &'static TimerQueue<Self> {
        &TIMER_QUEUE
    }
}

/// Create an EFR32 LETIMER based monotonic and register the necessary interrupt for it.
///
/// See [`crate::silabs::letimer`] for more details.
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
            pub fn start(
                timer: $crate::silabs::letimer::Letimer,
                cmu: &$crate::silabs::letimer::Cmu,
            ) {
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
