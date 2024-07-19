use crate::collision_detection::accd::ColliderPart;
use nalgebra::{Const, OVector, Point, RealField};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Vertex<N: RealField, const D: usize> {
    /// The current position.
    pub x: Point<N, D>,
    /// The velocity.
    pub v: OVector<N, Const<D>>,
}

impl<N: RealField, const D: usize> ColliderPart<D> for Vertex<N, D> {
    type Scalar = N;

    fn advance(&mut self, step: Self::Scalar) {
        self.x += self.v.scale(step);
    }

    fn max_speed(&self) -> Self::Scalar {
        self.v.magnitude()
    }

    fn center_position(&mut self, p: &Point<Self::Scalar, D>) {
        self.x -= &p.coords;
    }

    fn center_velocity(&mut self, v: &OVector<Self::Scalar, Const<D>>) {
        self.v -= v;
    }

    fn current_aabb(&self) -> (Point<Self::Scalar, D>, Point<Self::Scalar, D>) {
        (self.x.clone(), self.x.clone())
    }

    fn velocity_aabb(&self) -> (OVector<Self::Scalar, Const<D>>, OVector<Self::Scalar, Const<D>>) {
        (self.v.clone(), self.v.clone())
    }
}
