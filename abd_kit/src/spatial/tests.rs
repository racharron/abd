use crate::collision_detection::{distance_squared, Vertex};
use crate::spatial::sweep_prune::SweepPrune;
use crate::spatial::{IndexedAccdContext, Object, SpatialDB, UniformAccdContext};
use nalgebra::{Point3, Vector3};
use rand::prelude::StdRng;
use rand::{Rng, SeedableRng};
use std::array;

#[test]
fn aabb_aabb_vertex_3d() {
    let mut rng = StdRng::seed_from_u64(342);
    for _ in 0..10 {
        let mut vertices = Vec::from_iter(
            std::iter::repeat_with(|| Vertex {
                x: Point3::from(array::from_fn(|_| rng.gen_range(-4f32..=4.0))),
                v: Vector3::from(array::from_fn(|_| {
                    let v = rng.gen_range(-1f32..=1.0);
                    v * v.abs()
                })),
            })
            .take(100),
        );
        let uniform_context = UniformAccdContext {
            step_size: 10f32.powf(rng.gen_range(-3.0..=-1.0)),
            offset: 10f32.powf(rng.gen_range(-3.0..=-1.0)),
        };
        let mut broad = SweepPrune::from_iter(
            IndexedAccdContext {
                context: uniform_context,
                collection: &vertices,
            },
            0..vertices.len(),
        );
        broad.check_overlap_count();
        for _ in 0..10 {
            let context = IndexedAccdContext {
                context: uniform_context,
                collection: &vertices,
            };
            let pairs: hashbrown::HashSet<(usize, usize)> = hashbrown::HashSet::from_iter(
                broad
                    .self_close_pairs(context)
                    .map(|(a, b)| (a.min(b), a.max(b))),
            );
            for i in 0..vertices.len() {
                for j in (i + 1)..vertices.len() {
                    assert_eq!(
                        distance_squared::aabb_aabb(
                            &i.aabb_min(context),
                            &i.aabb_max(context),
                            &j.aabb_min(context),
                            &j.aabb_max(context)
                        ) == 0.,
                        pairs.contains(&(i, j))
                    );
                }
            }
            for vertex in &mut vertices {
                vertex.x += vertex.v.scale(uniform_context.step_size);
            }
            broad.update(IndexedAccdContext {
                context: uniform_context,
                collection: &vertices,
            });
            broad.check_overlap_count();
        }
    }
}
