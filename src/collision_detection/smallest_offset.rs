use crate::collision_detection::closest_point;

/// From the vertex to the line.
pub fn point_to_line<N: RealField, const D: usize>(
    p: &Point<N, D>,
    a: &Point<N, D>,
    b: &Point<N, D>,
) -> OVector<N, Const<D>> {
    closest_point::on_line(p, a, b) - p
}

/// Will return NaN if lines A and B are the same line.
pub fn between_par_lines_3d<N: RealField>(
    a0: &Point3<N>,
    a1: &Point3<N>,
    b0: &Point3<N>,
) -> Vector3<N> {
    let v = a1 - a0;
    let ab = b0 - a0;
    let p = ab.cross(&v);
    let n = p.cross(&v);
    n.scale(ab.dot(&n) / n.magnitude_squared())
    /*let [ax, ay, az] = [a0.x.clone(), a0.y.clone(), a0.z.clone()];
    let [bx, by, bz] = [b0.x.clone(), b0.y.clone(), b0.z.clone()];
    let [vx, vy, vz] = [v.x.clone(), v.y.clone(), v.z.clone()];
    let x = vy.clone();
    let tmp3 = x.powi(2);
    let x = vz.clone();
    let tmp8 = x.powi(2);
    let x = vx.clone();
    let tmp12 = x.powi(2);
    Vector3::new(
        ay.clone() * vx.clone() * vy.clone()
            - by.clone() * vx.clone() * vy.clone()
            - ax.clone() * tmp3.clone()
            + bx.clone() * tmp3.clone()
            + az.clone() * vx.clone() * vz.clone()
            - bz.clone() * vx.clone() * vz.clone()
            - ax.clone() * tmp8.clone()
            + bx.clone() * tmp8.clone(),
        -(ay.clone() * tmp12.clone())
            + by.clone() * tmp12.clone()
            + ax.clone() * vx.clone() * vy.clone()
            - bx.clone() * vx.clone() * vy.clone()
            + az.clone() * vy.clone() * vz.clone()
            - bz.clone() * vy.clone() * vz.clone()
            - ay.clone() * tmp8.clone()
            + by.clone() * tmp8.clone(),
        -(az.clone() * tmp12.clone()) + bz.clone() * tmp12.clone() - az.clone() * tmp3.clone()
            + bz.clone() * tmp3.clone()
            + ax.clone() * vx.clone() * vz.clone()
            - bx.clone() * vx.clone() * vz.clone()
            + ay.clone() * vy.clone() * vz.clone()
            - by.clone() * vy.clone() * vz.clone(),
    ) / v.magnitude_squared()*/
}

#[cfg(test)]
mod tests {
    use crate::collision_detection::closest_point::line_line_parameter_3d;
    use crate::collision_detection::smallest_offset::{between_par_lines_3d, point_to_line};
    use nalgebra::{point, vector, Point3, Vector3};
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};

    #[test]
    fn check_between_par_lines_3d() {
        let mut rng = StdRng::seed_from_u64(1234);
        let mut points = vec![Point3::<f32>::origin()];
        points.extend(
            (&mut rng)
                .sample_iter::<Point3<f32>, _>(rand::distributions::Standard)
                .take(49),
        );
        let vectors = Vec::from_iter(
            (&mut rng)
                .sample_iter::<Vector3<f32>, _>(rand::distributions::Standard)
                .filter(|v| v != &vector![0., 0., 0.])
                .take(50),
        );
        for a in &points {
            for b in &points {
                if a != b {
                    for v in &vectors {
                        let ab = b - a;
                        let v = ab.cross(v);
                        if v != vector![0., 0., 0.] {
                            let diff = between_par_lines_3d(
                                a,
                                &(a + v.scale(2. * rng.gen::<f32>() - 1.)),
                                &(b + v.scale(2. * rng.gen::<f32>() - 1.)),
                            ) - ab;
                            assert!(diff.abs().max() < 1.1e3, "diff = {:?}", diff);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn check_line_to_line_parameter_3d() {
        assert_eq!(
            line_line_parameter_3d(
                &point![-1., 1., 0.],
                &point![1., 1., 0.],
                &point![0., -1., -1.],
                &point![0., -1., 1.]
            ),
            Some(0.5)
        );
        assert_eq!(
            line_line_parameter_3d(
                &point![0., 1., 0.],
                &point![1., 1., 0.],
                &point![0., -1., 0.],
                &point![0., -1., 1.]
            ),
            Some(0.0)
        );
        assert_eq!(
            line_line_parameter_3d(
                &point![0., 1., -1.],
                &point![0., 1., 1.],
                &point![0., -1., -1.],
                &point![0., -1., 1.]
            ),
            None
        );
    }

    #[test]
    fn check_point_to_line_3d() {
        let mut rng = StdRng::seed_from_u64(1234);
        let mut points = vec![Point3::<f32>::origin()];
        points.extend(
            (&mut rng)
                .sample_iter::<Point3<f32>, _>(rand::distributions::Standard)
                .take(49),
        );
        for p in &points {
            for closest in &points {
                for other in &points {
                    let direction = (&closest.coords - &p.coords).cross(&other.coords);
                    let a = closest + &direction.scale(rng.gen());
                    let b = closest - &direction.scale(rng.gen());
                    if a != b {
                        let diff = point_to_line(p, &a, &b) - (closest - p);
                        assert!(diff.abs().max() < 2.5e-5, "diff = {:?}", diff);
                    }
                }
            }
        }
    }
}
