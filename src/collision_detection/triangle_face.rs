use crate::collision_detection::accd::ColliderPart;
use crate::collision_detection::vertex::Vertex;
use nalgebra::{Const, OVector, Point, RealField};

/// A triangle, where each vertex has its own individual velocity.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TriangleFace<N: RealField, const D: usize> {
    pub a: Vertex<N, D>,
    pub b: Vertex<N, D>,
    pub c: Vertex<N, D>,
}

impl<N: RealField, const D: usize> ColliderPart<D> for TriangleFace<N, D> {
    type Scalar = N;

    fn advance(&mut self, step: Self::Scalar) {
        self.a.advance(step.clone());
        self.b.advance(step.clone());
        self.c.advance(step);
    }

    fn max_speed(&self) -> Self::Scalar {
        self.a
            .max_speed()
            .max(self.b.max_speed())
            .max(self.c.max_speed())
    }

    fn center_position(&mut self, p: &Point<Self::Scalar, D>) {
        self.a.x -= &p.coords;
        self.b.x -= &p.coords;
        self.c.x -= &p.coords;
    }

    fn center_velocity(&mut self, v: &OVector<Self::Scalar, Const<D>>) {
        self.a.v -= v;
        self.b.v -= v;
        self.c.v -= v;
    }

    fn current_aabb(&self) -> (Point<Self::Scalar, D>, Point<Self::Scalar, D>) {
        (self.a.x.inf(&self.b.x).inf(&self.c.x), self.a.x.sup(&self.b.x).sup(&self.c.x))
    }

    fn velocity_aabb(&self) -> (OVector<Self::Scalar, Const<D>>, OVector<Self::Scalar, Const<D>>) {
        (self.a.v.inf(&self.b.v).inf(&self.c.v), self.a.v.sup(&self.b.v).sup(&self.c.v))
    }
}
