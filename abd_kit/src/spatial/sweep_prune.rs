use crate::spatial::{Object, SpatialDB};
use std::array;
use std::fmt::Debug;

pub struct SweepPrune<const D: usize, T> {
    axes: [Axis<T>; D],
}

#[derive(Debug)]
struct Axis<T> {
    sorted: Vec<AxisElement<T>>,
    collisions: usize,
}

#[derive(Debug)]
struct AxisElement<T> {
    item: T,
    kind: Kind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Min,
    Max,
}

impl<const D: usize, T> SweepPrune<D, T> {
    /// Create a new sweep and prune container.
    pub fn from_iter<I: IntoIterator<Item = T>, Ctx: Clone>(context: Ctx, iter: I) -> Self
    where
        T: Object<D, Ctx>,
    {
        let mut vecs = array::from_fn(|_| Vec::new());
        for item in iter {
            for v in &mut vecs {
                v.extend([
                    AxisElement {
                        item: item.clone(),
                        kind: Kind::Min,
                    },
                    AxisElement {
                        item: item.clone(),
                        kind: Kind::Max,
                    },
                ]);
            }
        }
        let mut i = 0;
        Self {
            axes: vecs.map(|mut v| {
                v.sort_by(|a, b| {
                    let a = match a.kind {
                        Kind::Min => a.item.aabb_min_index(context.clone(), i),
                        Kind::Max => a.item.aabb_max_index(context.clone(), i),
                    };
                    let b = match b.kind {
                        Kind::Min => b.item.aabb_max_index(context.clone(), i),
                        Kind::Max => b.item.aabb_max_index(context.clone(), i),
                    };
                    a.partial_cmp(&b).unwrap()
                });
                let mut collisions = 0;
                for j in 0..v.len() {
                    if v[j].kind == Kind::Min {
                        for k in (j + 1).. {
                            if v[k].kind == Kind::Min {
                                collisions += 1;
                            } else if v[j].item == v[k].item {
                                break;
                            }
                        }
                    }
                }
                i += 1;
                Axis {
                    sorted: v,
                    collisions,
                }
            }),
        }
    }
    /// Update the internal state of this item with the changed state of the `Item`s.
    pub fn update<Ctx: Clone>(&mut self, context: Ctx)
    where
        T: Object<D, Ctx>,
    {
        fn position<const D: usize, T: Object<D, Ctx>, Ctx: Clone>(
            ae: &AxisElement<T>,
            context: Ctx,
            i: usize,
        ) -> T::RF {
            match ae.kind {
                Kind::Min => ae.item.aabb_min_index(context, i),
                Kind::Max => ae.item.aabb_max_index(context, i),
            }
        }
        for (v, axis) in self.axes.iter_mut().enumerate() {
            for i in 1..axis.sorted.len() {
                let mut j = i;
                while position(&axis.sorted[j - 1], context.clone(), v)
                    > position(&axis.sorted[j], context.clone(), v)
                {
                    match (axis.sorted[j].kind, axis.sorted[j - 1].kind) {
                        (Kind::Min, Kind::Max) => {
                            axis.collisions += 1;
                            axis.sorted.swap(j, j - 1);
                        }
                        (Kind::Max, Kind::Min) => {
                            axis.collisions -= 1;
                            axis.sorted.swap(j, j - 1);
                        }
                        _ => {}
                    }
                    j -= 1;
                    if j == 0 {
                        break;
                    }
                }
            }
        }
    }
    #[cfg(test)]
    pub fn check_overlap_count(&self) where T: PartialEq {
        for axis in &self.axes {
            let mut count = 0;
            for i in 0..(axis.sorted.len()-1) {
                if axis.sorted[i].kind == Kind::Max {
                    continue
                }
                for j in (i+1)..axis.sorted.len() {
                    if axis.sorted[i].item == axis.sorted[j].item {
                        break
                    } else if axis.sorted[j].kind == Kind::Min {
                        count += 1;
                    }
                }
            }
            assert_eq!(axis.collisions, count);
        }
    }
}

pub struct OverlappingItemsIter<'a, const D: usize, T: Object<D, Ctx>, Ctx> {
    axis: &'a [AxisElement<T>],
    other_index: usize,
    context: Ctx,
}

impl<'a, const D: usize, T: Object<D, Ctx>, Ctx: Clone> Iterator
    for OverlappingItemsIter<'a, D, T, Ctx>
{
    type Item = (T, T);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if let (Some(AxisElement { item, .. }), Some(AxisElement { item: other, kind })) =
                (&self.axis.get(0), &self.axis.get(self.other_index))
            {
                if item == other {
                    self.axis = &self.axis[1..];
                    self.other_index = 1;
                } else {
                    if *kind == Kind::Min
                        && item.interacts_with(other, self.context.clone())
                        && item
                            .aabb_min(self.context.clone())
                            .le(&other.aabb_max(self.context.clone()))
                        && item
                            .aabb_max(self.context.clone())
                            .ge(&other.aabb_max(self.context.clone()))
                    {
                        self.other_index += 1;
                        return Some((item.clone(), other.clone()));
                    } else {
                        self.other_index += 1;
                    }
                }
            } else {
                return None;
            }
        }
    }
}

pub struct OverlappingIndicesIter<'a, const D: usize, T: Object<D, Ctx>, Ctx> {
    axis: &'a [AxisElement<T>],
    a_idx: usize,
    b_idx: usize,
    other_index: usize,
    context: Ctx,
}

impl<'a, const D: usize, T: Object<D, Ctx>, Ctx: Clone> Iterator
    for OverlappingIndicesIter<'a, D, T, Ctx>
{
    type Item = (usize, usize);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if let (Some(AxisElement { item, .. }), Some(AxisElement { item: other, kind })) =
                (&self.axis.get(0), &self.axis.get(self.other_index))
            {
                if item == other {
                    self.axis = &self.axis[1..];
                    self.other_index = 1;
                    self.a_idx += 1;
                    self.b_idx = self.a_idx;
                } else {
                    self.other_index += 1;
                    if *kind == Kind::Min {
                        self.b_idx += 1;
                        if item.interacts_with(other, self.context.clone())
                            && item
                            .aabb_min(self.context.clone())
                            .le(&other.aabb_max(self.context.clone()))
                            && item
                            .aabb_max(self.context.clone())
                            .ge(&other.aabb_max(self.context.clone()))
                        {
                            return Some((self.a_idx, self.b_idx));
                        }
                    }
                }
            } else {
                return None;
            }
        }
    }
}

impl<const D: usize, T> SweepPrune<D, T> {
    fn best_axis(&self) -> &Axis<T> {
        let mut argmax = 0;
        let mut max = self.axes[0].collisions;
        for i in 1..D {
            if self.axes[i].collisions > max {
                argmax = i;
                max = self.axes[i].collisions;
            }
        }
        &self.axes[argmax]
    }
}

impl<const D: usize, T: Object<D, Ctx>, Ctx: Clone> SpatialDB<D, Ctx> for SweepPrune<D, T> {
    type Item = T;

    #[allow(refining_impl_trait)]
    fn self_close_pairs<'a>(&'a self, context: Ctx) -> OverlappingItemsIter<'a, D, T, Ctx>
    where
        Ctx: 'a,
    {
        OverlappingItemsIter {
            axis: &self.best_axis().sorted,
            other_index: 1,
            context,
        }
    }

    fn all_items<'a>(&'a self) -> impl Iterator<Item=Self::Item> + 'a {
        self.best_axis().sorted.iter().filter_map(|AxisElement { item, kind }| (*kind == Kind::Min).then(|| item.clone()) )
    }

    fn self_close_indices<'a>(&'a self, context: Ctx) -> impl Iterator<Item=(usize, usize)> + 'a where Ctx: 'a {
        OverlappingIndicesIter {
            axis: &self.best_axis().sorted,
            a_idx: 0,
            b_idx: 0,
            other_index: 1,
            context,
        }
    }

    fn len(&self) -> usize {
        self.axes[0].sorted.len() / 2
    }
}

