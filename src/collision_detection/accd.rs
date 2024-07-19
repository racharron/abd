use crate::collision_detection::distance_squared;
use crate::collision_detection::segment::Segment;
use crate::collision_detection::triangle_face::TriangleFace;
use crate::collision_detection::vertex::Vertex;
use nalgebra::{Const, OVector, Point, RealField};
use std::marker::PhantomData;

/// Implemented by types that represent a distinct part of a collider that is tested for collision
/// detection.  For example, for two colliding triangular meshes, that would be the edges, faces,
/// and vertices.
pub trait ColliderPart<const D: usize> {
    type Scalar: RealField;
    /// Advance the translation of this part (move the vertices linearly).

    fn advance(&mut self, step: Self::Scalar);
    /// Maximum speed of all the points on the convex hull.
    fn max_speed(&self) -> Self::Scalar;
    /// Centers the positions around the given point.
    fn center_position(&mut self, p: &Point<Self::Scalar, D>);
    /// Centers the velocities around the given vector.
    fn center_velocity(&mut self, v: &OVector<Self::Scalar, Const<D>>);
    /// Gets the current (before any time has passed) axis aligned bounding box.  In `(min, max)`
    /// format.
    fn current_aabb(&self) -> (Point<Self::Scalar, D>, Point<Self::Scalar, D>);
    /// Gets the axis aligned bounding box of the velocities involved.  In `(min, max)` format.
    fn velocity_aabb(&self) -> (OVector<Self::Scalar, Const<D>>, OVector<Self::Scalar, Const<D>>);
}

/// Computes the unsigned squared distance between two collider parts.
pub trait D2Metric<N: RealField, const D: usize, T: ColliderPart<D, Scalar = N>, U: ColliderPart<D, Scalar = N>> {
    fn distance_squared(a: &T, b: &U) -> N;
}

pub struct SegmentSegment3D<N: RealField>(PhantomData<fn() -> N>);
pub struct PointTriangle3D<N: RealField>(PhantomData<fn() -> N>);

pub struct IpcAccdContact<RF: RealField> {
    time: RF,
    distance: RF,
}

/// Additive Continuous Collision Detection.  Based off the paper "Codimensional Incremental
/// Potential Contact".  In the style of incremental potential contact.  Internally changes the
/// frame of reference to minimize errors.
///
/// * `a`, `b` are the colliding objects.
/// * `thickness` is the sum of the half-thicknesses of the colliders.
/// * `scale` is the desired multiple of the current distance (after the thickness is applied) that
/// the collision detection should halt at.
/// * `barrier_thickness` is the distance between which a collision is considered to have occurred.
/// * `time_step` is the maximum time step.
/// * `delta_scale` scales the time advancement.  It *must* be between 0 and 1,
/// exclusive.  The default value from the paper is 0.9.
pub fn accd<
    RF: RealField,
    const D: usize,
    T: ColliderPart<D, Scalar=RF>,
    U: ColliderPart<D, Scalar=RF>,
    D2: D2Metric<RF, D, T, U>,
>(
    mut a: T,
    mut b: U,
    thickness: RF,
    scale: RF,
    barrier_thickness: RF,
    time_step: RF,
    delta_scale: RF,
) -> Option<RF> {
    let half = (RF::one() + RF::one()).recip();
    let (a_v_min, a_v_max) = a.velocity_aabb();
    let (b_v_min, b_v_max) = b.velocity_aabb();
    let central_velocity = (a_v_max.sup(&b_v_max) + a_v_min.inf(&b_v_min)).scale(half.clone());
    a.center_velocity(&central_velocity);
    b.center_velocity(&central_velocity);
    let mut t = RF::zero();
    loop {
        let (a_p_min, a_p_max) = a.current_aabb();
        let (b_p_min, b_p_max) = b.current_aabb();
        let central_position = (a_p_max.sup(&b_p_max).coords + a_p_min.inf(&b_p_min).coords).scale(half.clone());
        a.center_position(&central_position.clone().into());
        b.center_position(&central_position.into());
        if  let Some(contact)
            = ipc_accd::<_, D, _, _, D2>(&mut a, &mut b, thickness.clone(), scale.clone(), t, time_step.clone(), delta_scale.clone())
        {
            if contact.distance < barrier_thickness {
                return Some(contact.time)
            }
            t = contact.time;
        } else {
            return None
        }
    }
}

/// The base of Additive Continuous Collision Detection.  Based off the paper "Codimensional
/// Incremental Potential Contact".  In the style of incremental potential contact.
/// Does not change the frame of reference of objects like the algorithm in the paper.
///
/// * `a`, `b` are the colliding objects.
/// * `thickness` is the sum of the half-thicknesses of the colliders.
/// * `scale` is the desired multiple of the current distance (after the thickness is applied) that
/// the collision detection should halt at.
/// * `t_i` is the starting time.
/// * `t_f` is the maximum time step.
/// * `delta_scale` scales the time advancement.  It *must* be between 0 and 1,
/// exclusive.  The default value from the paper is 0.9.
pub fn ipc_accd<
    RF: RealField,
    const D: usize,
    T: ColliderPart<D, Scalar=RF>,
    U: ColliderPart<D, Scalar=RF>,
    D2: D2Metric<RF, D, T, U>,
>(
    a: &mut T,
    b: &mut U,
    thickness: RF,
    scale: RF,
    t_i: RF,
    t_f: RF,
    delta_scale: RF,
) -> Option<IpcAccdContact<RF>> {
    debug_assert!(RF::zero() < delta_scale);
    debug_assert!(delta_scale < RF::one());
    let l_p = a.max_speed().max(b.max_speed());
    if l_p.is_zero() {
        return None;
    }
    let d_sqr = D2::distance_squared(&a, &b);
    let d = d_sqr.clone().sqrt();
    let thickness_sqr = thickness.clone().powi(2);
    let g = scale.clone() * (d_sqr.clone() - thickness_sqr.clone()) / d.clone() + thickness.clone();
    let mut t = t_i;
    let mut t_l = (RF::one() - scale) * (d_sqr - thickness_sqr.clone())
        / ((d.clone() + thickness.clone()) * l_p.clone());
    loop {
        a.advance(t_l.clone());
        b.advance(t_l.clone());
        let d_sqr = D2::distance_squared(&a, &b);
        let d = d_sqr.clone().sqrt();
        if t > RF::zero()
            && (d_sqr.clone() - thickness_sqr.clone()) / (d.clone() + thickness.clone()) < g.clone()
        {
            return Some(IpcAccdContact {
                time: t,
                distance: d,
            });
        }
        t = t + t_l;
        if t > t_f {
            return None;
        }
        t_l = delta_scale.clone() * (d_sqr - thickness_sqr.clone())
            / ((d - thickness.clone()) * l_p.clone());
    }
}

impl<N: RealField> D2Metric<N, 3, Segment<N, 3>, Segment<N, 3>> for SegmentSegment3D<N> {
    fn distance_squared(a: &Segment<N, 3>, b: &Segment<N, 3>) -> N {
        distance_squared::segment_segment_3d(&a.0.x, &a.1.x, &b.0.x, &b.1.x)
    }
}

impl<N: RealField> D2Metric<N, 3, Vertex<N, 3>, TriangleFace<N, 3>> for PointTriangle3D<N> {
    fn distance_squared(a: &Vertex<N, 3>, b: &TriangleFace<N, 3>) -> N {
        distance_squared::point_triangle_3d(&a.x, &b.a.x, &b.b.x, &b.c.x)
    }
}

impl<N: RealField> D2Metric<N, 3, TriangleFace<N, 3>, Vertex<N, 3>> for PointTriangle3D<N> {
    fn distance_squared(a: &TriangleFace<N, 3>, b: &Vertex<N, 3>) -> N {
        distance_squared::point_triangle_3d(&b.x, &a.a.x, &a.b.x, &a.c.x)
    }
}
