use crate::collision_detection::smallest_offset;
use nalgebra::{Point, Point3, RealField};

pub fn on_line<N: RealField, const D: usize>(
    p: &Point<N, D>,
    a: &Point<N, D>,
    b: &Point<N, D>,
) -> Point<N, D> {
    let t = point_line_parameter(p, a, b);
    (a.coords.scale(N::one() - t.clone()) + b.coords.scale(t)).into()
}

pub fn on_segment<N: RealField, const D: usize>(
    p: &Point<N, D>,
    a: &Point<N, D>,
    b: &Point<N, D>,
) -> Point<N, D> {
    let t = point_line_parameter(p, a, b).clamp(N::zero(), N::one());
    (a.coords.scale(N::one() - t.clone()) + b.coords.scale(t)).into()
}

pub fn on_triangle_3d<N: RealField>(
    p: &Point3<N>,
    a: &Point3<N>,
    b: &Point3<N>,
    c: &Point3<N>,
) -> Point3<N> {
    //  TODO:   use optimized code from
    //          https://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf
    let t_ab = point_line_parameter(p, a, b);
    let t_bc = point_line_parameter(p, b, c);
    let t_ca = point_line_parameter(p, c, a);
    if t_ca > N::one() && t_ab < N::zero() {
        a.clone()
    } else if t_ab > N::one() && t_bc < N::zero() {
        b.clone()
    } else if t_bc > N::one() && t_ca < N::zero() {
        c.clone()
    } else {
        fn on_edge<N: RealField>(
            t: N,
            p: &Point3<N>,
            a: &Point3<N>,
            b: &Point3<N>,
            c: &Point3<N>,
        ) -> Option<Point3<N>> {
            if (N::zero()..=N::one()).contains(&t) {
                let on =
                    Point3::from(a.coords.scale(N::one() - t.clone()) + b.coords.scale(t.clone()));
                if (p - &on)
                    .dot(&smallest_offset::point_to_line(c, a, b))
                    .is_sign_positive()
                {
                    // panic!("{t}");
                    Some(on)
                } else {
                    None
                }
            } else {
                None
            }
        }
        if let Some(p) = on_edge(t_ab, p, &a, &b, &c)
            .or_else(|| on_edge(t_bc, p, &b, &c, &a))
            .or_else(|| on_edge(t_ca, p, &c, &a, &b))
        {
            return p;
        } else {
            let orth = (b - a).cross(&(c - a));
            p - orth.scale((p - a).dot(&orth) / orth.magnitude_squared())
        }
    }
}

pub fn point_line_parameter<N: RealField, const D: usize>(
    p: &Point<N, D>,
    a: &Point<N, D>,
    b: &Point<N, D>,
) -> N {
    (p - a).dot(&(b - a)) / nalgebra::distance_squared(a, b)
}

/// When the lines are parallel, returns None.  Otherwise, returns the point parameter for line a.
pub fn line_line_parameter_3d<N: RealField>(
    a0: &Point3<N>,
    a1: &Point3<N>,
    b0: &Point3<N>,
    b1: &Point3<N>,
) -> Option<N> {
    //  TODO: handle nearly parallel lines.
    let [a0x, a0y, a0z] = [a0.x.clone(), a0.y.clone(), a0.z.clone()];
    let [a1x, a1y, a1z] = [a1.x.clone(), a1.y.clone(), a1.z.clone()];
    let [b0x, b0y, b0z] = [b0.x.clone(), b0.y.clone(), b0.z.clone()];
    let [b1x, b1y, b1z] = [b1.x.clone(), b1.y.clone(), b1.z.clone()];
    let two = N::one() + N::one();
    let four = two.clone() + two.clone();
    let x = b0x.clone();
    let tmp2 = x.powi(2);
    let x = b0y.clone();
    let tmp9 = x.powi(2);
    let x = a1z.clone();
    let tmp4 = x.powi(2);
    let x = a0x.clone();
    let tmp8 = x.powi(2);
    let x = b0z.clone();
    let tmp18 = x.powi(2);
    let x = a1x.clone();
    let tmp12 = x.powi(2);
    let x = a1y.clone();
    let tmp1 = x.powi(2);
    let x = b1x.clone();
    let tmp29 = x.powi(2);
    let x = b1y.clone();
    let tmp41 = x.powi(2);
    let tmp47 = -two.clone() * b0x.clone() * b1x.clone();
    let tmp48 = -two.clone() * b0y.clone() * b1y.clone();
    let tmp49 = tmp2.clone()
        + tmp9.clone()
        + tmp47.clone().clone()
        + tmp29.clone()
        + tmp48.clone()
        + tmp41.clone();
    let x = b1z.clone();
    let tmp75 = x.powi(2);
    let tmp54 = -b1x.clone();
    let tmp55 = b0x.clone() + tmp54.clone();
    let tmp56 = a0x.clone() * tmp55.clone();
    let tmp60 = -b1z.clone();
    let tmp61 = b0z.clone() + tmp60.clone();
    let tmp81 = -two.clone() * b0z.clone() * b1z.clone();
    let tmp82 = tmp2.clone()
        + tmp18.clone()
        + tmp47.clone()
        + tmp29.clone()
        + tmp81.clone()
        + tmp75.clone();
    let tmp10 = tmp8.clone() * tmp9.clone();
    let tmp19 = tmp8.clone() * tmp18.clone();
    let tmp34 = -two.clone() * tmp8.clone() * b0y.clone() * b1y.clone();
    let tmp42 = tmp8.clone() * tmp41.clone();
    let x = a0z.clone();
    let tmp46 = x.powi(2);
    let tmp50 = tmp46.clone() * tmp49.clone();
    let tmp68 = -two.clone() * tmp8.clone() * b0z.clone() * b1z.clone();
    let tmp76 = tmp8.clone() * tmp75.clone();
    let tmp51 = a1z.clone() * tmp49.clone();
    let x = a0y.clone();
    let tmp80 = x.powi(2);
    let tmp83 = tmp80.clone() * tmp82.clone();
    let tmp84 = -b1y.clone();
    let tmp85 = b0y.clone() + tmp84.clone();
    let tmp94 = a1y.clone() * tmp82.clone();
    let denom = tmp1.clone() * tmp2.clone()
        + tmp4.clone() * tmp2.clone()
        + two.clone() * a0x.clone() * a1y.clone() * b0x.clone() * b0y.clone()
        - two.clone() * a1x.clone() * a1y.clone() * b0x.clone() * b0y.clone()
        + tmp10.clone()
        - two.clone() * a0x.clone() * a1x.clone() * tmp9.clone()
        + tmp12.clone() * tmp9.clone()
        + tmp4.clone() * tmp9.clone()
        + two.clone() * a0x.clone() * a1z.clone() * b0x.clone() * b0z.clone()
        - two.clone() * a1x.clone() * a1z.clone() * b0x.clone() * b0z.clone()
        - two.clone() * a1y.clone() * a1z.clone() * b0y.clone() * b0z.clone()
        + tmp19.clone()
        - two.clone() * a0x.clone() * a1x.clone() * tmp18.clone()
        + tmp12.clone() * tmp18.clone()
        + tmp1.clone() * tmp18.clone()
        - two.clone() * tmp1.clone() * b0x.clone() * b1x.clone()
        - two.clone() * tmp4.clone() * b0x.clone() * b1x.clone()
        - two.clone() * a0x.clone() * a1y.clone() * b0y.clone() * b1x.clone()
        + two.clone() * a1x.clone() * a1y.clone() * b0y.clone() * b1x.clone()
        - two.clone() * a0x.clone() * a1z.clone() * b0z.clone() * b1x.clone()
        + two.clone() * a1x.clone() * a1z.clone() * b0z.clone() * b1x.clone()
        + tmp1.clone() * tmp29.clone()
        + tmp4.clone() * tmp29.clone()
        - two.clone() * a0x.clone() * a1y.clone() * b0x.clone() * b1y.clone()
        + two.clone() * a1x.clone() * a1y.clone() * b0x.clone() * b1y.clone()
        + tmp34.clone()
        + four.clone() * a0x.clone() * a1x.clone() * b0y.clone() * b1y.clone()
        - two.clone() * tmp12.clone() * b0y.clone() * b1y.clone()
        - two.clone() * tmp4.clone() * b0y.clone() * b1y.clone()
        + two.clone() * a1y.clone() * a1z.clone() * b0z.clone() * b1y.clone()
        + two.clone() * a0x.clone() * a1y.clone() * b1x.clone() * b1y.clone()
        - two.clone() * a1x.clone() * a1y.clone() * b1x.clone() * b1y.clone()
        + tmp42.clone()
        - two.clone() * a0x.clone() * a1x.clone() * tmp41.clone()
        + tmp12.clone() * tmp41.clone()
        + tmp4.clone() * tmp41.clone()
        + tmp50.clone()
        - two.clone()
            * a0z.clone()
            * (tmp51.clone()
                + (-(a1x.clone() * b0x.clone()) - a1y.clone() * b0y.clone()
                    + tmp56.clone()
                    + a1x.clone() * b1x.clone()
                    + a1y.clone() * b1y.clone())
                    * tmp61.clone())
        - two.clone() * a0x.clone() * a1z.clone() * b0x.clone() * b1z.clone()
        + two.clone() * a1x.clone() * a1z.clone() * b0x.clone() * b1z.clone()
        + two.clone() * a1y.clone() * a1z.clone() * b0y.clone() * b1z.clone()
        + tmp68.clone()
        + four.clone() * a0x.clone() * a1x.clone() * b0z.clone() * b1z.clone()
        - two.clone() * tmp12.clone() * b0z.clone() * b1z.clone()
        - two.clone() * tmp1.clone() * b0z.clone() * b1z.clone()
        + two.clone() * a0x.clone() * a1z.clone() * b1x.clone() * b1z.clone()
        - two.clone() * a1x.clone() * a1z.clone() * b1x.clone() * b1z.clone()
        - two.clone() * a1y.clone() * a1z.clone() * b1y.clone() * b1z.clone()
        + tmp76.clone()
        - two.clone() * a0x.clone() * a1x.clone() * tmp75.clone()
        + tmp12.clone() * tmp75.clone()
        + tmp1.clone() * tmp75.clone()
        + tmp83.clone()
        - two.clone()
            * a0y.clone()
            * (tmp85.clone()
                * (tmp56.clone()
                    + a1x.clone() * (-b0x.clone() + b1x.clone())
                    + (a0z.clone() - a1z.clone()) * tmp61.clone())
                + tmp94.clone());
    if denom.is_zero() {
        None
    } else {
        Some(
            (a0x.clone() * a1y.clone() * b0x.clone() * b0y.clone() + tmp10.clone()
                - a0x.clone() * a1x.clone() * tmp9.clone()
                + a0x.clone() * a1z.clone() * b0x.clone() * b0z.clone()
                + tmp19.clone()
                - a0x.clone() * a1x.clone() * tmp18.clone()
                - a0x.clone() * a1y.clone() * b0y.clone() * b1x.clone()
                - a1y.clone() * b0x.clone() * b0y.clone() * b1x.clone()
                - a0x.clone() * tmp9.clone() * b1x.clone()
                + a1x.clone() * tmp9.clone() * b1x.clone()
                - a0x.clone() * a1z.clone() * b0z.clone() * b1x.clone()
                - a1z.clone() * b0x.clone() * b0z.clone() * b1x.clone()
                - a0x.clone() * tmp18.clone() * b1x.clone()
                + a1x.clone() * tmp18.clone() * b1x.clone()
                + a1y.clone() * b0y.clone() * tmp29.clone()
                + a1z.clone() * b0z.clone() * tmp29.clone()
                - a0x.clone() * a1y.clone() * b0x.clone() * b1y.clone()
                + a1y.clone() * tmp2.clone() * b1y.clone()
                + tmp34.clone()
                + two.clone() * a0x.clone() * a1x.clone() * b0y.clone() * b1y.clone()
                + a0x.clone() * b0x.clone() * b0y.clone() * b1y.clone()
                - a1x.clone() * b0x.clone() * b0y.clone() * b1y.clone()
                - a1z.clone() * b0y.clone() * b0z.clone() * b1y.clone()
                + a1y.clone() * tmp18.clone() * b1y.clone()
                + a0x.clone() * a1y.clone() * b1x.clone() * b1y.clone()
                - a1y.clone() * b0x.clone() * b1x.clone() * b1y.clone()
                + a0x.clone() * b0y.clone() * b1x.clone() * b1y.clone()
                - a1x.clone() * b0y.clone() * b1x.clone() * b1y.clone()
                + tmp42.clone()
                - a0x.clone() * a1x.clone() * tmp41.clone()
                - a0x.clone() * b0x.clone() * tmp41.clone()
                + a1x.clone() * b0x.clone() * tmp41.clone()
                + a1z.clone() * b0z.clone() * tmp41.clone()
                + tmp50.clone()
                - a0x.clone() * a1z.clone() * b0x.clone() * b1z.clone()
                + a1z.clone() * tmp2.clone() * b1z.clone()
                + a1z.clone() * tmp9.clone() * b1z.clone()
                + tmp68.clone()
                + two.clone() * a0x.clone() * a1x.clone() * b0z.clone() * b1z.clone()
                + a0x.clone() * b0x.clone() * b0z.clone() * b1z.clone()
                - a1x.clone() * b0x.clone() * b0z.clone() * b1z.clone()
                - a1y.clone() * b0y.clone() * b0z.clone() * b1z.clone()
                + a0x.clone() * a1z.clone() * b1x.clone() * b1z.clone()
                - a1z.clone() * b0x.clone() * b1x.clone() * b1z.clone()
                + a0x.clone() * b0z.clone() * b1x.clone() * b1z.clone()
                - a1x.clone() * b0z.clone() * b1x.clone() * b1z.clone()
                - a1z.clone() * b0y.clone() * b1y.clone() * b1z.clone()
                - a1y.clone() * b0z.clone() * b1y.clone() * b1z.clone()
                + tmp76.clone()
                - a0x.clone() * a1x.clone() * tmp75.clone()
                - a0x.clone() * b0x.clone() * tmp75.clone()
                + a1x.clone() * b0x.clone() * tmp75.clone()
                + a1y.clone() * b0y.clone() * tmp75.clone()
                - a0z.clone()
                    * (-(a1x.clone() * b0x.clone() * b0z.clone())
                        - a1y.clone() * b0y.clone() * b0z.clone()
                        + a1x.clone() * b0z.clone() * b1x.clone()
                        - b0x.clone() * b0z.clone() * b1x.clone()
                        + b0z.clone() * tmp29.clone()
                        + a1y.clone() * b0z.clone() * b1y.clone()
                        - b0y.clone() * b0z.clone() * b1y.clone()
                        + b0z.clone() * tmp41.clone()
                        + tmp51.clone()
                        + two.clone() * a0x.clone() * tmp55.clone() * tmp61.clone()
                        + a1x.clone() * b0x.clone() * b1z.clone()
                        + tmp2.clone() * b1z.clone()
                        + a1y.clone() * b0y.clone() * b1z.clone()
                        + tmp9.clone() * b1z.clone()
                        - a1x.clone() * b1x.clone() * b1z.clone()
                        - b0x.clone() * b1x.clone() * b1z.clone()
                        - a1y.clone() * b1y.clone() * b1z.clone()
                        - b0y.clone() * b1y.clone() * b1z.clone())
                + tmp83.clone()
                - a0y.clone()
                    * (-(a1x.clone() * b0x.clone() * b0y.clone())
                        + two.clone() * a0z.clone() * b0y.clone() * b0z.clone()
                        - a1z.clone() * b0y.clone() * b0z.clone()
                        + a1x.clone() * b0y.clone() * b1x.clone()
                        - b0x.clone() * b0y.clone() * b1x.clone()
                        + b0y.clone() * tmp29.clone()
                        + two.clone() * a0x.clone() * tmp55.clone() * tmp85.clone()
                        + a1x.clone() * b0x.clone() * b1y.clone()
                        + tmp2.clone() * b1y.clone()
                        - two.clone() * a0z.clone() * b0z.clone() * b1y.clone()
                        + a1z.clone() * b0z.clone() * b1y.clone()
                        + tmp18.clone() * b1y.clone()
                        - a1x.clone() * b1x.clone() * b1y.clone()
                        - b0x.clone() * b1x.clone() * b1y.clone()
                        - two.clone() * a0z.clone() * b0y.clone() * b1z.clone()
                        + a1z.clone() * b0y.clone() * b1z.clone()
                        - b0y.clone() * b0z.clone() * b1z.clone()
                        + two.clone() * a0z.clone() * b1y.clone() * b1z.clone()
                        - a1z.clone() * b1y.clone() * b1z.clone()
                        - b0z.clone() * b1y.clone() * b1z.clone()
                        + b0y.clone() * tmp75.clone()
                        + tmp94.clone()))
                / denom,
        )
    }
}

#[cfg(test)]
mod tests {
    use crate::collision_detection::closest_point::{on_triangle_3d, point_line_parameter};
    use crate::collision_detection::smallest_offset;
    use nalgebra::{vector, Point3, Vector3};
    use rand::distributions::Uniform;
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};

    #[test]

    fn check_point_line_parameter() {
        let mut rng = StdRng::seed_from_u64(1234);
        let dist = Uniform::new(-2., 2.);
        let mut points = vec![Point3::<f32>::origin()];
        points.extend(
            (&mut rng)
                .sample_iter::<Point3<f32>, _>(rand::distributions::Standard)
                .take(64),
        );
        for p in &points {
            for closest in &points {
                if p == closest {
                    continue;
                }
                for other in &points {
                    let direction = (closest - p).cross(&other.coords);
                    if direction.magnitude_squared() < 1e-3 {
                        continue;
                    }
                    let t = rng.sample(dist);
                    let a = closest - &direction.scale(t);
                    let b = closest + &direction.scale(1. - t);
                    if a != b {
                        let diff = point_line_parameter(p, &a, &b) - t;
                        assert!(
                            diff.abs() < 1e-4,
                            "diff = {:?}\na = {:?}, b = {:?}",
                            diff,
                            a,
                            b
                        );
                    }
                }
            }
        }
    }
    #[test]
    fn check_tri_point_corners() {
        let mut rng = StdRng::seed_from_u64(1234);
        for _ in 0..100 {
            let a = Point3::from(rng.gen::<Vector3<f32>>().scale(2.) - vector![1., 1., 1.]);
            let b = Point3::from(rng.gen::<Vector3<f32>>().scale(2.) - vector![1., 1., 1.]);
            let c = Point3::from(rng.gen::<Vector3<f32>>().scale(2.) - vector![1., 1., 1.]);
            for _ in 0..100 {
                let x = rng.gen::<f32>();
                let y = rng.gen::<f32>();
                let z = rng.sample(Uniform::new(-1., 1.));
                {
                    let ex: Vector3<f32> = smallest_offset::point_to_line(&c, &a, &b);
                    let ey: Vector3<f32> = smallest_offset::point_to_line(&b, &a, &c);
                    let ez: Vector3<f32> = ex.cross(&ey);
                    let p = a + ex.scale(x) + ey.scale(y) + ez.scale(z);
                    assert_eq!(
                        a,
                        on_triangle_3d(&p, &a, &b, &c),
                        "p = {:?}, b = {:?}, c = {:?}",
                        p,
                        b,
                        c
                    );
                }
                {
                    let ex = smallest_offset::point_to_line(&a, &b, &c);
                    let ey = smallest_offset::point_to_line(&c, &a, &b);
                    let ez = ex.cross(&ey);
                    let p = b + ex.scale(x) + ey.scale(y) + ez.scale(z);
                    assert_eq!(
                        b,
                        on_triangle_3d(&p, &a, &b, &c),
                        "p = {:?}, a = {:?}, c = {:?}",
                        p,
                        a,
                        c
                    );
                }
                {
                    let ex = smallest_offset::point_to_line(&b, &c, &a);
                    let ey = smallest_offset::point_to_line(&a, &c, &b);
                    let ez = ex.cross(&ey);
                    let p = c + ex.scale(x) + ey.scale(y) + ez.scale(z);
                    assert_eq!(
                        c,
                        on_triangle_3d(&p, &a, &b, &c),
                        "p = {:?}, a = {:?}, b= {:?}",
                        p,
                        a,
                        b
                    );
                }
            }
        }
    }
    #[test]
    fn check_tri_point_edges() {
        let mut rng = StdRng::seed_from_u64(1234);
        for i in 0..100 {
            let a = Point3::from(rng.gen::<Vector3<f32>>().scale(2.) - vector![1., 1., 1.]);
            let b = Point3::from(rng.gen::<Vector3<f32>>().scale(2.) - vector![1., 1., 1.]);
            let c = Point3::from(rng.gen::<Vector3<f32>>().scale(2.) - vector![1., 1., 1.]);
            for j in 0..100 {
                let x = rng.gen::<f32>();
                let y = rng.gen::<f32>();
                let z = rng.sample(Uniform::new(-1., 1.));
                {
                    let ex: Vector3<f32> = smallest_offset::point_to_line(&c, &a, &b);
                    let ey: Vector3<f32> = b - a;
                    let ez: Vector3<f32> = ex.cross(&ey);
                    let p = a + ex.scale(x) + ey.scale(y) + ez.scale(z);
                    let result = on_triangle_3d(&p, &a, &b, &c);
                    let diff = (a + ey.scale(y)) - result;
                    assert!(
                        diff.abs().max() < 1.0e-6,
                        "({i}, {j}): diff = {:?}\nresult = {:?}\np = {:?}\na = {:?}, b = {:?}, c = {:?}",
                        diff, result, p, a, b, c
                    );
                }
                {
                    let ex: Vector3<f32> = smallest_offset::point_to_line(&a, &b, &c);
                    let ey: Vector3<f32> = c - b;
                    let ez: Vector3<f32> = ex.cross(&ey);
                    let p = b + ex.scale(x) + ey.scale(y) + ez.scale(z);
                    let result = on_triangle_3d(&p, &a, &b, &c);
                    let diff = (b + ey.scale(y)) - result;
                    assert!(
                        diff.abs().max() < 1.0e-6,
                        "({i}, {j}): diff = {:?}\nresult = {:?}\np = {:?}\na = {:?}, b = {:?}, c = {:?}",
                        diff, result, p, a, b, c
                    );
                }
                {
                    let ex: Vector3<f32> = smallest_offset::point_to_line(&b, &c, &a);
                    let ey: Vector3<f32> = a - c;
                    let ez: Vector3<f32> = ex.cross(&ey);
                    let p = c + ex.scale(x) + ey.scale(y) + ez.scale(z);
                    let result = on_triangle_3d(&p, &a, &b, &c);
                    let diff = (c + ey.scale(y)) - result;
                    assert!(
                        diff.abs().max() < 1.0e-6,
                        "({i}, {j}): diff = {:?}\nresult = {:?}\np = {:?}\na = {:?}, b = {:?}, c = {:?}",
                        diff, result, p, a, b, c
                    );
                }
            }
        }
    }
    #[test]
    fn check_tri_point_inside() {
        let mut rng = StdRng::seed_from_u64(1234);
        for i in 0..100 {
            let a = Point3::from(rng.gen::<Vector3<f32>>().scale(2.) - vector![1., 1., 1.]);
            let b = Point3::from(rng.gen::<Vector3<f32>>().scale(2.) - vector![1., 1., 1.]);
            let c = Point3::from(rng.gen::<Vector3<f32>>().scale(2.) - vector![1., 1., 1.]);
            for j in 0..100 {
                let on_tri = loop {
                    let u = rng.gen::<f32>();
                    let v = rng.gen::<f32>();
                    let alpha = 1. - u.sqrt();
                    let beta = u.sqrt() * (1. - v);
                    let gamma = u.sqrt() * v;
                    let on_tri = Point3::from(
                        a.coords.scale(alpha) + b.coords.scale(beta) + c.coords.scale(gamma),
                    );
                    if (0.0..=1.0).contains(&point_line_parameter(&on_tri, &a, &b))
                        && (0.0..=1.0).contains(&point_line_parameter(&on_tri, &b, &c))
                        && (0.0..=1.0).contains(&point_line_parameter(&on_tri, &c, &a))
                    {
                        break on_tri;
                    }
                };
                let z = rng.sample(Uniform::new(-1., 1.));
                let p = on_tri + (b - a).cross(&(c - a)).scale(z);
                let result = on_triangle_3d(&p, &a, &b, &c);
                assert!((0.0..=1.0).contains(&point_line_parameter(&result, &a, &b)));
                assert!((0.0..=1.0).contains(&point_line_parameter(&result, &b, &c)));
                assert!((0.0..=1.0).contains(&point_line_parameter(&result, &c, &a)));
                let diff = on_tri - result;
                assert!(
                    diff.abs().max() < 1.0e-6,
                    "({i}, {j}): diff = {:?}\nresult = {:?}\np = {:?}\na = {:?}, b = {:?}, c = {:?}",
                    diff, result, p, a, b, c
                );
            }
        }
    }
}
