pub mod accd;
pub mod closest_point;
pub mod distance_squared;
pub mod intersecting;
mod segment;
pub mod smallest_offset;
mod triangle_face;
mod vertex;
mod collider;
/*
#[cfg(test)]
mod tests;
*/
use nalgebra::RealField;
pub use segment::Segment;
pub use triangle_face::TriangleFace;
pub use vertex::Vertex;
use crate::spatial::SpatialDB;

pub fn minimum_time_step<RF: RealField, const D: usize, Ctx: Clone, DB: SpatialDB<D, Ctx>>(time_step: RF, world: &DB, context: Ctx) -> RF {
    for (a, b) in world.self_close_pairs(context.clone()) {

    }
    todo!()
}
