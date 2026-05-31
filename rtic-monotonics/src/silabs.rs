//! [`Monotonic`](super::Monotonic) implementations for the SiLabs series of
//! MCUs (EFR32, EFM32, ...).
//!
//! There are two monotonic implementations:
//! * [rtcc]: uses the RTCC peripheral (32-bit)
//! * [letimer]: uses the LETimer peripheral (24-bit)

pub mod letimer;
pub mod rtcc;
