use nalgebra::{Const, RealField, Storage, Vector};

pub fn aabb_segment<RF: RealField, const D: usize, S: Storage<RF, Const<D>>>(
    min: &Vector<RF, Const<D>, S>,
    max: &Vector<RF, Const<D>, S>,
    a: &Vector<RF, Const<D>, S>,
    b: &Vector<RF, Const<D>, S>,
) -> bool {
    (min.le(a) && a.le(max) && min.le(b) && b.le(max)) || {
        let direction = b - a;
        let t_min = &(min - a).component_div(&direction);
        let t_max = &(max - a).component_div(&direction);
        let (t_near, t_far) = t_min.inf_sup(&t_max);
        let range = RF::zero()..=RF::one();
        range.contains(&t_near.max()) || range.contains(&t_far.min())
    }
}

#[cfg(test)]
mod tests {
    use crate::collision_detection::intersecting::aabb_segment;
    use nalgebra::vector;
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};

    #[test]
    fn aab_segment_3d() {
        let mut rng = StdRng::seed_from_u64(100);
        for _ in 0..=100 {
            let min = vector![
                rng.gen_range(-4f32..=2.),
                rng.gen_range(-4.0..=2.),
                rng.gen_range(-4.0..=2.)
            ];
            let max = min
                + vector![
                    rng.gen_range(0.0..=2.),
                    rng.gen_range(0.0..=2.),
                    rng.gen_range(0.0..=2.)
                ];
            for _ in 0..=100 {
                let a = vector![
                    rng.gen_range(min.x..=max.x),
                    rng.gen_range(min.y..=max.y),
                    rng.gen_range(min.z..=max.z)
                ];
                let b = vector![
                    rng.gen_range(-4.0..=4.),
                    rng.gen_range(-4.0..=4.),
                    rng.gen_range(-4.0..=4.)
                ];
                assert!(aabb_segment(&min, &max, &a, &b));
            }
        }
    }
}
