#![cfg_attr(not(feature = "pc"), no_std)]

#[cfg(target_os = "none")]
extern crate alloc;

pub mod shared;
