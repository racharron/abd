use super::{closest_point, smallest_offset};
use crate::collision_detection::segment::Segment;
use crate::collision_detection::vertex::Vertex;
use nalgebra::{Const, OVector, Point, Point3, RealField};

pub fn point_point<N: RealField, const D: usize>(o1: &Vertex<N, D>, o2: &Vertex<N, D>) -> N {
    nalgebra::distance_squared(&o1.x, &o2.x)
}

pub fn point_segment<N: RealField, const D: usize>(o1: &Vertex<N, D>, o2: &Segment<N, D>) -> N {
    closest_point::on_segment(&o1.x, &o2.0.x, &o2.1.x)
        .coords
        .magnitude_squared()
}

pub fn segment_segment_3d<N: RealField>(
    a0: &Point3<N>,
    a1: &Point3<N>,
    b0: &Point3<N>,
    b1: &Point3<N>,
) -> N {
    let t1 = closest_point::line_line_parameter_3d(a0, a1, b0, b1);
    let t2 = closest_point::line_line_parameter_3d(b0, b1, a0, a1);
    match (t1, t2) {
        (Some(t1), Some(t2)) => {
            let (t1, t2) = (t1.clamp(N::zero(), N::one()), t2.clamp(N::zero(), N::one()));
            let a = a0.coords.scale(N::one() - t1.clone()) + a1.coords.scale(t1);
            let b = b0.coords.scale(N::one() - t2.clone()) + b1.coords.scale(t2);
            (a - b).magnitude_squared()
        }
        (None, None) => {
            let v = a1 - a0;
            let (b0, b1) = if v.dot(&(b0 - b1)).is_sign_positive() {
                (b0, b1)
            } else {
                (b1, b0)
            };
            if (b0 - a1).dot(&v).is_sign_positive() {
                (b0 - a1).magnitude_squared()
            } else if (a0 - b1).dot(&v).is_sign_positive() {
                (a0 - b1).magnitude_squared()
            } else {
                smallest_offset::between_par_lines_3d(a0, a1, b0).magnitude_squared()
            }
        }
        _ => unreachable!(),
    }
}

pub fn line_line_3d<N: RealField>(
    a0: &Point3<N>,
    a1: &Point3<N>,
    b0: &Point3<N>,
    b1: Point3<N>,
) -> N {
    let [a0x, a0y, a0z] = [a0.x.clone(), a0.y.clone(), a0.z.clone()];
    let [a1x, a1y, a1z] = [a1.x.clone(), a1.y.clone(), a1.z.clone()];
    let [b0x, b0y, b0z] = [b0.x.clone(), b0.y.clone(), b0.z.clone()];
    let [b1x, b1y, b1z] = [b1.x.clone(), b1.y.clone(), b1.z.clone()];
    let two = N::one() + N::one();
    let four = two.clone() + two.clone();
    let x = b0x.clone();
    let tmp66 = x.powi(2);
    let x = b0y.clone();
    let tmp73 = x.powi(2);
    let x = a1z.clone();
    let tmp68 = x.powi(2);
    let x = a0x.clone();
    let tmp72 = x.powi(2);
    let x = b0z.clone();
    let tmp82 = x.powi(2);
    let x = a1x.clone();
    let tmp76 = x.powi(2);
    let x = a1y.clone();
    let tmp65 = x.powi(2);
    let x = b1x.clone();
    let tmp93 = x.powi(2);
    let x = b1y.clone();
    let tmp107 = x.powi(2);
    let tmp113 = -two.clone() * b0x.clone() * b1x.clone();
    let tmp114 = -two.clone() * b0y.clone() * b1y.clone();
    let tmp115 =
        tmp66.clone() + tmp73.clone() + tmp113.clone() + tmp93.clone() + tmp114 + tmp107.clone();
    let x = b1z.clone();
    let tmp141 = x.powi(2);
    let tmp120 = -b1x.clone();
    let tmp121 = b0x.clone() + tmp120;
    let tmp122 = a0x.clone() * tmp121;
    let tmp33 = -b0x.clone();
    let tmp34 = tmp33 + b1x.clone();
    let tmp126 = -b1z.clone();
    let tmp127 = b0z.clone() + tmp126;
    let tmp147 = -two.clone() * b0z.clone() * b1z.clone();
    let tmp148 = tmp66.clone() + tmp82.clone() + tmp113 + tmp93.clone() + tmp147 + tmp141.clone();
    let x = a0y.clone();
    let x1 = a0z.clone();
    let denom = tmp65.clone() * tmp66.clone()
        + tmp68.clone() * tmp66
        + two.clone() * a0x.clone() * a1y.clone() * b0x.clone() * b0y.clone()
        - two.clone() * a1x.clone() * a1y.clone() * b0x.clone() * b0y.clone()
        + tmp72.clone() * tmp73.clone()
        - two.clone() * a0x.clone() * a1x.clone() * tmp73.clone()
        + tmp76.clone() * tmp73.clone()
        + tmp68.clone() * tmp73
        + two.clone() * a0x.clone() * a1z.clone() * b0x.clone() * b0z.clone()
        - two.clone() * a1x.clone() * a1z.clone() * b0x.clone() * b0z.clone()
        - two.clone() * a1y.clone() * a1z.clone() * b0y.clone() * b0z.clone()
        + tmp72.clone() * tmp82.clone()
        - two.clone() * a0x.clone() * a1x.clone() * tmp82.clone()
        + tmp76.clone() * tmp82.clone()
        + tmp65.clone() * tmp82.clone()
        - two.clone() * tmp65.clone() * b0x.clone() * b1x.clone()
        - two.clone() * tmp68.clone() * b0x.clone() * b1x.clone()
        - two.clone() * a0x.clone() * a1y.clone() * b0y.clone() * b1x.clone()
        + two.clone() * a1x.clone() * a1y.clone() * b0y.clone() * b1x.clone()
        - two.clone() * a0x.clone() * a1z.clone() * b0z.clone() * b1x.clone()
        + two.clone() * a1x.clone() * a1z.clone() * b0z.clone() * b1x.clone()
        + tmp65.clone() * tmp93.clone()
        + tmp68.clone() * tmp93
        - two.clone() * a0x.clone() * a1y.clone() * b0x.clone() * b1y.clone()
        + two.clone() * a1x.clone() * a1y.clone() * b0x.clone() * b1y.clone()
        - two.clone() * tmp72.clone() * b0y.clone() * b1y.clone()
        + four.clone() * a0x.clone() * a1x.clone() * b0y.clone() * b1y.clone()
        - two.clone() * tmp76.clone() * b0y.clone() * b1y.clone()
        - two.clone() * tmp68.clone() * b0y.clone() * b1y.clone()
        + two.clone() * a1y.clone() * a1z.clone() * b0z.clone() * b1y.clone()
        + two.clone() * a0x.clone() * a1y.clone() * b1x.clone() * b1y.clone()
        - two.clone() * a1x.clone() * a1y.clone() * b1x.clone() * b1y.clone()
        + tmp72.clone() * tmp107.clone()
        - two.clone() * a0x.clone() * a1x.clone() * tmp107.clone()
        + tmp76.clone() * tmp107.clone()
        + tmp68 * tmp107
        + x1.powi(2) * tmp115.clone()
        - two.clone()
            * a0z.clone()
            * (a1z.clone() * tmp115
                + (-(a1x.clone() * b0x.clone()) - a1y.clone() * b0y.clone()
                    + tmp122.clone()
                    + a1x.clone() * b1x.clone()
                    + a1y.clone() * b1y.clone())
                    * tmp127.clone())
        - two.clone() * a0x.clone() * a1z.clone() * b0x.clone() * b1z.clone()
        + two.clone() * a1x.clone() * a1z.clone() * b0x.clone() * b1z.clone()
        + two.clone() * a1y.clone() * a1z.clone() * b0y.clone() * b1z.clone()
        - two.clone() * tmp72.clone() * b0z.clone() * b1z.clone()
        + four * a0x.clone() * a1x.clone() * b0z.clone() * b1z.clone()
        - two.clone() * tmp76.clone() * b0z.clone() * b1z.clone()
        - two.clone() * tmp65.clone() * b0z.clone() * b1z.clone()
        + two.clone() * a0x.clone() * a1z.clone() * b1x.clone() * b1z.clone()
        - two.clone() * a1x.clone() * a1z.clone() * b1x.clone() * b1z.clone()
        - two.clone() * a1y.clone() * a1z.clone() * b1y.clone() * b1z.clone()
        + tmp72 * tmp141.clone()
        - two.clone() * a0x.clone() * a1x.clone() * tmp141.clone()
        + tmp76 * tmp141.clone()
        + tmp65 * tmp141
        + x.powi(2) * tmp148.clone()
        - two.clone()
            * a0y.clone()
            * ((b0y.clone() - b1y.clone())
                * (tmp122 + a1x.clone() * tmp34.clone() + (a0z.clone() - a1z.clone()) * tmp127)
                + a1y.clone() * tmp148);
    if denom.is_zero() {
        smallest_offset::between_par_lines_3d(a0, a1, b0).magnitude_squared()
    } else {
        let s = {
            (-(a0x.clone() * a1z.clone() * b0y.clone())
                + a0x.clone() * a1y.clone() * b0z.clone()
                + a1z.clone() * b0y.clone() * b1x.clone()
                - a1y.clone() * b0z.clone() * b1x.clone()
                + a0x.clone() * a1z.clone() * b1y.clone()
                - a1z.clone() * b0x.clone() * b1y.clone()
                - a0x.clone() * b0z.clone() * b1y.clone()
                + a1x.clone() * b0z.clone() * b1y.clone()
                + a0z.clone()
                    * (a1x.clone() * b0y.clone() - b0y.clone() * b1x.clone()
                        + a1y.clone() * tmp34.clone()
                        - a1x.clone() * b1y.clone()
                        + b0x.clone() * b1y.clone())
                - a0x.clone() * a1y.clone() * b1z.clone()
                + a1y.clone() * b0x.clone() * b1z.clone()
                + a0x.clone() * b0y.clone() * b1z.clone()
                - a1x.clone() * b0y.clone() * b1z.clone()
                + a0y.clone()
                    * (a1z.clone() * b0x.clone()
                        - a1x.clone() * b0z.clone()
                        - a1z.clone() * b1x.clone()
                        + b0z.clone() * b1x.clone()
                        + a1x.clone() * b1z.clone()
                        - b0x.clone() * b1z.clone()))
                / denom
        };
        (a1 - a0).cross(&(b1 - b0).scale(s)).magnitude_squared()
    }
}

pub fn point_triangle_3d<N: RealField>(
    p: &Point3<N>,
    a: &Point3<N>,
    b: &Point3<N>,
    c: &Point3<N>,
) -> N {
    (p - &closest_point::on_triangle_3d(p, a, b, c)).magnitude_squared()
}

pub fn aabb_aabb<RF: RealField, const D: usize>(
    a_min: &Point<RF, D>,
    a_max: &Point<RF, D>,
    b_min: &Point<RF, D>,
    b_max: &Point<RF, D>,
) -> RF {
    let min = a_min.sup(b_min);
    let max = a_max.inf(b_max);
    (min - max)
        .sup(&OVector::<RF, Const<D>>::from_element(RF::zero()))
        .magnitude_squared()
}
