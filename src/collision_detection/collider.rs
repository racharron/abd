use crate::spatial::Object;

mod triangle_mesh;

pub use triangle_mesh::*;

pub trait Collider<const D: usize, Ctx>: Object<D, Ctx> {
    type Scalar: RealField;
    /// `barrier_thickness` is the distance that the barrier function extends away from the object.
    fn collide_in(&self, other: &Self, time_step: Self::Scalar, barrier_thickness: Self::Scalar, context: Ctx) -> Option<Self::Scalar>;
}

pub trait SubColliderPair {
    type Scalar: RealField;
    fn advance(&mut self,advance: Self::Scalar);
}