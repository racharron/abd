use crate::collision_detection::{distance_squared, Segment, Triangle, Vertex};
use nalgebra::{distance_squared, point, vector, Const, OVector, Point, Point3, Vector, Vector3};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::array;
use std::ops::RangeInclusive;
// use crate::spatial::hash_grid::{InGrid, TransientHashGrid};
use crate::spatial::SpatialDB;
/*
const RANGE: RangeInclusive<f32> = -4.0..=4.0;

fn gen_vertex<const D: usize>(rng: &mut StdRng) -> Vertex<f32, { D }> {
    Vertex::<f32, D> {
        x: Point::from(array::from_fn::<f32, D, _>(|_| rng.gen_range(RANGE))),
        v: OVector::from(array::from_fn::<f32, D, _>(|_| rng.gen_range(RANGE))),
    }
}

fn gen_vertex_in_grid<const D: usize>(rng: &mut StdRng, step: f32, scale: f32) -> Vertex<f32, { D }> {
    Vertex {
        x: Point::from(OVector::<f32, Const<D>>::from(array::from_fn(|_| rng.gen_range(RANGE))).scale(scale / step)),
        v: OVector::<f32, Const<D>>::from(array::from_fn(|_| rng.gen_range(-(4.0 * scale / step)..=(4.0 * scale / step)))),
    }
}

fn vertex_in_grid<const D: usize>() {
    let mut rng = StdRng::seed_from_u64(768);
    for _ in 0..=10_000 {
        let step = 10f32.powf(rng.gen_range(-3.0..=-1.));
        let scale = 2f32.powf(rng.gen_range(-5.0..=5.0));
        let vertex = gen_vertex_in_grid(&mut rng, step, scale);
        let offset = scale * 10f32.powf(rng.gen_range(-3.0..=-1.));
        let mut set = hashbrown::HashSet::new();
        for cell in <Vertex<f32, D> as InGrid<i32, D>>::occupied_cells(&vertex, &step, &offset, &scale) {
            assert!(set.insert(cell));
        }
    }
}

#[test]
fn vertex_in_grid_2d() {
    vertex_in_grid::<2>()
}

#[test]
fn vertex_in_grid_3d() {
    vertex_in_grid::<3>()
}

fn segment_in_grid<const D: usize>() {
    let mut rng = StdRng::seed_from_u64(768);
    for _ in 0..=1_000 {
        let step = 10f32.powf(rng.gen_range(-3.0..=-1.));
        let scale = 2f32.powf(rng.gen_range(-5.0..=5.0));
        let a = gen_vertex_in_grid(&mut rng, step, scale);
        let b = Vertex {
            x: a.x + OVector::from(array::from_fn(|_| rng.gen_range(-scale..=scale))),
            v: OVector::<f32, Const<D>>::from(array::from_fn(|_| rng.gen_range(-(4.0 * scale / step)..=(4.0 * scale / step)))),
        };
        let segment = Segment(a, b);
        let offset = scale * 10f32.powf(rng.gen_range(-3.0..=-1.));
        let mut set = hashbrown::HashSet::new();
        for cell in <Segment<f32, D> as InGrid<i32, D>>::occupied_cells(&segment, &step, &offset, &scale) {
            assert!(set.insert(cell));
        }
    }
}

#[test]
fn segment_in_grid_2d() {
    segment_in_grid::<2>()
}

#[test]
fn segment_in_grid_3d() {
    segment_in_grid::<3>()
}

fn triangle_in_grid<const D: usize>() {
    let mut rng = StdRng::seed_from_u64(768);
    for _ in 0..=1_000 {
        let step = 10f32.powf(rng.gen_range(-3.0..=-1.));
        let scale = 2f32.powf(rng.gen_range(-5.0..=5.0));
        let a = gen_vertex_in_grid(&mut rng, step, scale);
        let b = Vertex {
            x: a.x + OVector::from(array::from_fn(|_| rng.gen_range(-scale..=scale))),
            v: OVector::<f32, Const<D>>::from(array::from_fn(|_| rng.gen_range(-(4.0 * scale / step)..=(4.0 * scale / step)))),
        };
        let c = Vertex {
            x: a.x + OVector::from(array::from_fn(|_| rng.gen_range(-scale..=scale))),
            v: OVector::<f32, Const<D>>::from(array::from_fn(|_| rng.gen_range(-(4.0 * scale / step)..=(4.0 * scale / step)))),
        };
        let triangle = Triangle { a, b, c };
        let offset = scale * 10f32.powf(rng.gen_range(-3.0..=-1.));
        let mut set = hashbrown::HashSet::new();
        for cell in <Triangle<f32, D> as InGrid<i32, D>>::occupied_cells(&triangle, &step, &offset, &scale) {
            assert!(set.insert(cell));
        }
    }
}

#[test]
fn triangle_in_grid_2d() {
    triangle_in_grid::<2>()
}

#[test]
fn triangle_in_grid_3d() {
    triangle_in_grid::<3>()
}

#[test]
fn hash_grid_3d() {
    let mut rng = StdRng::seed_from_u64(8723);
    for _ in 0..=100 {
        let scale = 2f32.powf(rng.gen_range(-5.0..=5.0));
        let step = 10f32.powf(rng.gen_range(-3.0..=-1.));
        let mut hash_grid = TransientHashGrid::<i32, _, 3>::new(scale);
        let vertices = Vec::from_iter(
            std::iter::repeat_with(||
                Vertex {
                    x: point![
                        4. * scale * rng.gen::<f32>().powi(2),
                        4. * scale * rng.gen::<f32>().powi(2),
                        4. * scale * rng.gen::<f32>().powi(2)],
                    v: vector![
                        {let x = 4. * scale * rng.gen::<f32>().powi(2); if rng.gen() { -x } else { x }},
                        {let y = 4. * scale * rng.gen::<f32>().powi(2); if rng.gen() { -y } else { y }},
                        {let z = 4. * scale * rng.gen::<f32>().powi(2); if rng.gen() { -z } else { z }},
                    ],
                })
                .take(25)
        );
        let half_thicknesses = Vec::from_iter(
            std::iter::repeat_with(|| 10f32.powf(rng.gen_range(-3.0..=-1.)))
                .take(25)
        );
        for (vertex, ht) in vertices.iter().zip(&half_thicknesses) {
            hash_grid.add(&step, ht, vertex);
        }

        let mut all_pairs = hashbrown::HashSet::new();
        for (a, b) in hash_grid.self_close_pairs() {
            let a = a as *const _ as usize;
            let b = b as *const _ as usize;
            assert_ne!(a, b);
            assert!(all_pairs.insert((a.min(b), a.max(b))));
        }
        for i in 0..vertices.len() {
            for j in (i+1)..vertices.len() {
                let a = &vertices[i];
                let b = &vertices[j];
                let distance = distance_squared::segment_segment_3d(&a.x, &(a.x + a.v.scale(step)), &b.x, &(b.x + b.v.scale(step))).sqrt();
                let a = a as *const _ as usize;
                let b = b as *const _ as usize;
                let pair = (a.min(b), a.max(b));
                if distance <= half_thicknesses[i] + half_thicknesses[j] {
                    assert!(all_pairs.contains(&pair), "d = {distance}, ht[i] = {}, ht[j] = {}", half_thicknesses[i], half_thicknesses[j]);
                } else if distance >= 3f32.sqrt() * (scale + half_thicknesses[i] + half_thicknesses[j]) {
                    assert!(!all_pairs.contains(&pair));
                }
            }
        }
    }
}
*/
