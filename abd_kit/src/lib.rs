//! This is not a physics engine.
//!
//! It does, however, contain all the pieces needed to put together one yourself.
//!
//! The collision detection is based off affine additive collision detection, as detailed in
//! [Codimensional Incremental Potential Contact](https://arxiv.org/abs/2012.04457), and is
//! contact free.
//!
//! The actual physics is implemented using
//! [Affine Body Dynamics](https://arxiv.org/abs/2201.10022),
//! which is an intersection free method (when used with contact free collision detection).

pub mod collision_detection;
mod configuration;
mod moments;
pub mod spatial;
pub mod util;
pub mod kinematics;

pub use configuration::*;
pub use moments::*;
