use crate::collision_detection::accd::ColliderPart;
use crate::collision_detection::vertex::Vertex;

/// A line segment defined by its two end points.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Segment<N: RealField, const D: usize>(pub Vertex<N, D>, pub Vertex<N, D>);

impl<N: RealField, const D: usize> ColliderPart<D> for Segment<N, D> {
    type Scalar = N;

    fn advance(&mut self, step: Self::Scalar) {
        self.0.advance(step.clone());
        self.1.advance(step);
    }

    fn max_speed(&self) -> Self::Scalar {
        self.0.max_speed().max(self.1.max_speed())
    }

    fn center_position(&mut self, p: &Point<Self::Scalar, D>) {
        self.0.x -= &p.coords;
        self.1.x -= &p.coords;
    }

    fn center_velocity(&mut self, v: &OVector<Self::Scalar, Const<D>>) {
        self.0.v -= v;
        self.1.v -= v;
    }

    fn current_aabb(&self) -> (Point<Self::Scalar, D>, Point<Self::Scalar, D>) {
        (self.0.x.inf(&self.1.x), self.0.x.sup(&self.1.x))
    }

    fn velocity_aabb(&self) -> (OVector<Self::Scalar, Const<D>>, OVector<Self::Scalar, Const<D>>) {
        (self.0.v.inf(&self.1.v), self.0.v.sup(&self.1.v))
    }
}
