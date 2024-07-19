use crate::collision_detection::{Segment, TriangleFace, Vertex};
use nalgebra::{Point, RealField};

// pub mod hash_grid;
// pub mod hierarchical_hash_grid;
pub mod octree;
pub mod sweep_prune;
#[cfg(test)]
mod tests;

/// A spatial datastructure.  Capable of doing close distance querying.
pub trait SpatialDB<const D: usize, Ctx> {
    /// The type that a spatial database keeps track of.  Could be a reference, a geometric
    /// object, a reference, or anything really.
    type Item: Object<D, Ctx>;

    /// Look for close pairs of objects within this datastructure.
    fn self_close_pairs<'a>(
        &'a self,
        context: Ctx,
    ) -> impl Iterator<Item = (Self::Item, Self::Item)> + 'a
    where
        Ctx: 'a;
    /// Retrieve all items in some order consistent with [`Self::all_close_indices`].
    //  TODO: make exact sized iterator?
    fn all_items<'a>(&'a self) -> impl Iterator<Item=Self::Item> + 'a;
    /// The same as [`Self::self_close_pairs`], but returns indices consistent with
    /// [`Self::all_items`] *and* [`Self::self_close_pairs`] (in the same order).
    fn self_close_indices<'a>(&'a self, context: Ctx) -> impl Iterator<Item=(usize, usize)> + 'a where Ctx: 'a;
    fn len(&self) -> usize;
}

/// Describes a pair of spatial datastructures that can do closeness checking against each other.
pub trait CompatibleWith<const D: usize, Ctx1, S: SpatialDB<D, Ctx2>, Ctx2>:
    SpatialDB<D, Ctx1>
{
    fn other_close_pairs<'a, 'b>(
        &'a self,
        context_1: Ctx1,
        other: &'b S,
        context_2: Ctx2,
    ) -> impl Iterator<Item = (Self::Item, S::Item)> + 'a + 'b;
}

/// An object with spatial extent.
pub trait Object<const D: usize, Ctx>: Clone + PartialEq {
    type RF: RealField;
    /// Returns the minimum corner of this objects axis aligned bounding box.
    fn aabb_min(&self, context: Ctx) -> Point<Self::RF, D>;
    /// Returns the maximum corner of this objects axis aligned bounding box.
    fn aabb_max(&self, context: Ctx) -> Point<Self::RF, D>;
    /// Get the axis aligned bounding box minimum along one axis.
    fn aabb_min_index(&self, context: Ctx, i: usize) -> Self::RF;
    /// Get the axis aligned bounding box maximum along one axis.
    fn aabb_max_index(&self, context: Ctx, i: usize) -> Self::RF;
    /// Indicates if this object interacts with another.  Objects that are just colliders of the
    /// same object should return `false`.
    fn interacts_with(&self, other: &Self, context: Ctx) -> bool;
}

#[derive(Clone, Copy, Debug)]
pub struct UniformAccdContext<RF: RealField> {
    step_size: RF,
    offset: RF,
}

#[derive(Debug)]
pub struct IndexedAccdContext<'a, Ctx: Clone, Col> {
    context: Ctx,
    collection: &'a Col,
}

/// The type of a geometric element that is part of a collider.  It implements the `interacts_with`
/// method so that items part of the same collider (`G`) do not interact, but others do.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SubCollider<T, G=u32> {
    object: T,
    collider: G,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Index<T>(T);

impl<const D: usize, RF: RealField> Object<D, UniformAccdContext<RF>> for Vertex<RF, D> {
    type RF = RF;

    fn aabb_min(&self, context: UniformAccdContext<RF>) -> Point<Self::RF, D> {
        self.x
            .inf(&(&self.x + &self.v.scale(context.step_size)))
            .coords
            .add_scalar(-context.offset)
            .into()
    }

    fn aabb_max(&self, context: UniformAccdContext<RF>) -> Point<Self::RF, D> {
        self.x
            .sup(&(&self.x + &self.v.scale(context.step_size)))
            .coords
            .add_scalar(context.offset)
            .into()
    }

    fn aabb_min_index(&self, context: UniformAccdContext<RF>, i: usize) -> Self::RF {
        self.x[i]
            .clone()
            .min(self.x[i].clone() + self.v[i].clone().scale(context.step_size))
            - context.offset
    }

    fn aabb_max_index(&self, context: UniformAccdContext<RF>, i: usize) -> Self::RF {
        self.x[i]
            .clone()
            .max(self.x[i].clone() + self.v[i].clone().scale(context.step_size))
            + context.offset
    }

    fn interacts_with(&self, _: &Self, _: UniformAccdContext<RF>) -> bool {
        true
    }
}

impl<const D: usize, RF: RealField> Object<D, UniformAccdContext<RF>> for Segment<RF, D> {
    type RF = RF;

    fn aabb_min(&self, context: UniformAccdContext<RF>) -> Point<Self::RF, D> {
        self.0
            .x
            .inf(&(&self.0.x + &self.0.v.scale(context.step_size.clone())))
            .inf(&self.1.x)
            .inf(&(&self.1.x + &self.1.v.scale(context.step_size)))
            .coords
            .add_scalar(-context.offset)
            .into()
    }

    fn aabb_max(&self, context: UniformAccdContext<RF>) -> Point<Self::RF, D> {
        self.0
            .x
            .sup(&(&self.0.x + &self.0.v.scale(context.step_size.clone())))
            .sup(&self.1.x)
            .sup(&(&self.1.x + &self.1.v.scale(context.step_size)))
            .coords
            .add_scalar(context.offset)
            .into()
    }

    fn aabb_min_index(&self, context: UniformAccdContext<RF>, i: usize) -> Self::RF {
        self.0.x[i]
            .clone()
            .min(self.0.x[i].clone() + self.0.v[i].clone() * context.step_size.clone())
            .min(self.1.x[i].clone())
            .min(self.1.x[i].clone() + self.1.v[i].clone() * context.step_size)
            - context.offset
    }

    fn aabb_max_index(&self, context: UniformAccdContext<RF>, i: usize) -> Self::RF {
        self.0.x[i]
            .clone()
            .max(self.0.x[i].clone() + self.0.v[i].clone() * context.step_size.clone())
            .max(self.1.x[i].clone())
            .max(self.1.x[i].clone() + self.1.v[i].clone() * context.step_size)
            + context.offset
    }

    fn interacts_with(&self, _: &Self, _: UniformAccdContext<RF>) -> bool {
        true
    }
}

impl<const D: usize, RF: RealField> Object<D, UniformAccdContext<RF>> for TriangleFace<RF, D> {
    type RF = RF;

    fn aabb_min(&self, context: UniformAccdContext<RF>) -> Point<Self::RF, D> {
        self.a
            .x
            .inf(&(&self.a.x + &self.a.v.scale(context.step_size.clone())))
            .inf(&self.b.x)
            .inf(&(&self.b.x + &self.b.v.scale(context.step_size.clone())))
            .inf(&self.c.x)
            .inf(&(&self.c.x + &self.c.v.scale(context.step_size)))
            .coords
            .add_scalar(-context.offset)
            .into()
    }

    fn aabb_max(&self, context: UniformAccdContext<RF>) -> Point<Self::RF, D> {
        self.a
            .x
            .sup(&(&self.a.x + &self.a.v.scale(context.step_size.clone())))
            .sup(&self.b.x)
            .sup(&(&self.b.x + &self.b.v.scale(context.step_size.clone())))
            .sup(&self.c.x)
            .sup(&(&self.c.x + &self.c.v.scale(context.step_size)))
            .coords
            .add_scalar(context.offset)
            .into()
    }

    fn aabb_min_index(&self, context: UniformAccdContext<RF>, i: usize) -> Self::RF {
        self.a.x[i]
            .clone()
            .min(self.a.x[i].clone() + self.a.v[i].clone() * context.step_size.clone())
            .min(self.b.x[i].clone())
            .min(self.b.x[i].clone() + self.b.v[i].clone() * context.step_size.clone())
            .min(self.c.x[i].clone())
            .min(self.c.x[i].clone() + self.c.v[i].clone() * context.step_size)
            - context.offset
    }

    fn aabb_max_index(&self, context: UniformAccdContext<RF>, i: usize) -> Self::RF {
        self.a.x[i]
            .clone()
            .max(self.a.x[i].clone() + self.a.v[i].clone() * context.step_size.clone())
            .max(self.b.x[i].clone())
            .max(self.b.x[i].clone() + self.b.v[i].clone() * context.step_size.clone())
            .max(self.c.x[i].clone())
            .max(self.c.x[i].clone() + self.c.v[i].clone() * context.step_size)
            + context.offset
    }

    fn interacts_with(&self, _: &Self, _: UniformAccdContext<RF>) -> bool {
        true
    }
}

impl<'a, C, I, const D: usize, Ctx: Clone> Object<D, IndexedAccdContext<'a, Ctx, C>> for Index<I>
where
    C: std::ops::Index<I>,
    I: Clone + PartialEq,
    C::Output: Object<D, Ctx>,
{
    type RF = <C::Output as Object<D, Ctx>>::RF;

    fn aabb_min(&self, context: IndexedAccdContext<Ctx, C>) -> Point<Self::RF, D> {
        context.collection[self.0.clone()].aabb_min(context.context)
    }

    fn aabb_max(&self, context: IndexedAccdContext<Ctx, C>) -> Point<Self::RF, D> {
        context.collection[self.0.clone()].aabb_max(context.context)
    }

    fn aabb_min_index(&self, context: IndexedAccdContext<Ctx, C>, i: usize) -> Self::RF {
        context.collection[self.0.clone()].aabb_min_index(context.context, i)
    }

    fn aabb_max_index(&self, context: IndexedAccdContext<Ctx, C>, i: usize) -> Self::RF {
        context.collection[self.0.clone()].aabb_max_index(context.context, i)
    }

    fn interacts_with(&self, other: &Self, context: IndexedAccdContext<Ctx, C>) -> bool {
        context.collection[self.0.clone()]
            .interacts_with(&context.collection[other.0.clone()], context.context)
    }
}

impl<const D: usize, T: Object<D, Ctx>, Ctx, G: Clone + Eq> Object<D, Ctx> for SubCollider<T, G> {
    type RF = T::RF;

    fn aabb_min(&self, context: Ctx) -> Point<Self::RF, D> {
        T::aabb_min(&self.object, context)
    }

    fn aabb_max(&self, context: Ctx) -> Point<Self::RF, D> {
        T::aabb_max(&self.object, context)
    }

    fn aabb_min_index(&self, context: Ctx, i: usize) -> Self::RF {
        T::aabb_min_index(&self.object, context, i)
    }

    fn aabb_max_index(&self, context: Ctx, i: usize) -> Self::RF {
        T::aabb_max_index(&self.object, context, i)
    }

    fn interacts_with(&self, other: &Self, context: Ctx) -> bool {
        self.collider != other.collider && T::interacts_with(&self.object, &other.object, context)
    }
}

impl<'a, Ctx: Copy, Col> Clone for IndexedAccdContext<'a, Ctx, Col> {
    fn clone(&self) -> Self {
        Self {
            context: self.context.clone(),
            collection: self.collection,
        }
    }
    fn clone_from(&mut self, source: &Self) {
        self.context = source.context.clone();
        self.collection = source.collection;
    }
}

impl<'a, Ctx: Copy, Col> Copy for IndexedAccdContext<'a, Ctx, Col> {}
