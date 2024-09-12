use std::hash::Hash;
use hashbrown::HashMap;
use crate::{AffineTransform, Moments};
use crate::spatial::SpatialDB;

struct ObjectRef<'s, 'v, 'm, 'f, RF: RealField> {
    state: &'s AffineTransform<RF>,
    velocity: &'v AffineTransform<RF>,
    moments: &'m Moments<RF>,
    forces: &'f AffineTransform<RF>,
}

/// A set of interacting objects whose next state will rely upon each other.
pub struct InteractingObjects<'s, 'v, 'm, 'f, 'c, RF: RealField, C> {
    objects: Vec<ObjectRef<'s, 'v, 'm, 'f, RF>>,
    close: HashMap<(usize, usize), &'c C>,
}

impl<'s, 'v, 'm, 'f, 'c, 'd, RF: RealField, C> InteractingObjects<'s, 'v, 'm, 'f, 'c, RF, C> {
    pub fn create_isles<Ctx, DB: SpatialDB<3, Ctx>>(&self, db: DB, ctx: Ctx) -> Vec<Self> {
        let mut uf = partitions::PartitionVec::with_capacity(db.len());
        uf.extend(db.all_items());
        for (a, b) in db.self_close_indices(ctx) {
            uf.union(a, b);
        }
        uf.all_sets()
            .map(|set| {
                InteractingObjects { objects: todo!(), close: todo!() }
            })
            .collect()
    }
    pub fn potential(&self, param: &[AffineTransform<RF>], time_step: RF) -> RF {
        todo!()
    }
    pub fn gradient(&self, param: &[AffineTransform<RF>], time_step: RF) -> RF {
        todo!()
    }
}
