use crate::collision_detection::{intersecting, Segment, Triangle, Vertex};
use nalgebra::{Const, OVector, RealField};
use num::cast::AsPrimitive;
use num::PrimInt;
use std::collections::HashMap;
use std::fmt::Debug;
use std::hash::Hash;
use std::marker::PhantomData;
use crate::spatial::SpatialDB;

/// A hash grid with fixed size cells.  All items entered into it *must* be unique (and have unique
/// addresses).
#[derive(Debug)]
pub struct TransientHashGrid<'a, G: PrimInt + Hash, T: InGrid<G, D>, const D: usize> {
    scale: T::Real,
    /// Each item is placed into this map exactly once.
    items: Vec<(&'a T, Vec<[G; D]>)>,
    /// Stores all mappings from grid cells to items.
    grid_map: HashMap<[G; D], Vec<&'a T>>,
}

pub trait InGrid<G: PrimInt, const DIM: usize> {
    /// The type of the grid scale.
    type Real;
    /// The dimensionality of the grid.
    /// This should produce a set of cells that the object intersects with, and ideally no more,
    /// although returning additional ones will merely increase overhead.
    fn occupied_cells<'a>(
        &'a self,
        step: &Self::Real,
        offset: &Self::Real,
        scale: &Self::Real,
    ) -> impl Iterator<Item = [G; DIM]> + 'a;
}

impl<'a, const D: usize, G: PrimInt + Hash, T: InGrid<G, D>> TransientHashGrid<'a, G, T, D> {
    pub fn new(scale: T::Real) -> Self {
        Self {
            scale,
            items: Default::default(),
            grid_map: Default::default(),
        }
    }
    /// Adds an item to a grid cell as it moves linearly for the geven `step`, with a minimum
    /// detection range of `offset`, at the specified `scale`.
    pub fn add<'b>(&'b mut self, step: &'_ T::Real, offset: &'_ T::Real, item: &'a T) where 'a: 'b {
        let mut cells = item.occupied_cells(step, offset, &self.scale).collect();
        for &cell in &cells {
            self.grid_map.entry(cell).and_modify(|v| {
                if v.iter().all(|prev| !std::ptr::addr_eq::<T, T>(*prev, item)) {
                    v.push(item);
                }
            }).or_insert_with(|| vec![item]);
        }
        self.items.push((item, cells));
    }
}

impl<G: PrimInt + Hash + Debug, T: Debug + InGrid<G, D>, const D: usize> SpatialDB for TransientHashGrid<G, T, D> {
    type Item = T;

    fn self_close_pairs<'a>(&'a self) -> impl std::fmt::Debug + Iterator<Item=(Self::Item, Self::Item)> + 'a {
        #[derive(Debug)]
        struct Iter<'a: 'a, 'b, G: PrimInt + Hash, T: InGrid<G, D>, const D: usize> {
            grid_map: &'a HashMap<[G; D], Vec<&'a T>>,
            items: <&'a [(&'a T, Vec<[G; D]>)] as IntoIterator>::IntoIter,
            item: Option<&'a T>,
            returned: Vec<&'a T>,
            cells: <&'a [[G; D]] as IntoIterator>::IntoIter,
            others: <&'a [&'a T] as IntoIterator>::IntoIter,
        }
        impl<'a: 'a, 'b, G: PrimInt + Hash + Debug, T: Debug + InGrid<G, D>, const D: usize> Iterator for Iter<'a, 'a, G, T, D> {
            type Item = (&'a T, &'a T);

            fn next(&mut self) -> Option<Self::Item> {
                if let Some(mut item) = self.item {
                    loop {
                        if let Some(other) = self.others.next() {
                            if !self.returned.iter().any(|item| std::ptr::addr_eq::<T, T>(*item, *other))
                                && ((item as *const T as usize) < (*other as *const T as usize))
                            {
                                self.returned.push(*other);
                                return Some((item, *other))
                            }
                        } else if let Some(cell) = self.cells.next() {
                            self.others = self.grid_map.get(cell).unwrap().iter();
                        } else if let Some((new_item, cells)) = self.items.next() {
                            self.item = Some(*new_item);
                            item = new_item;
                            self.cells = cells.iter();
                            self.returned.clear();
                        } else {
                            self.item = None;
                            return None
                        }
                    }
                } else {
                    None
                }
            }
        }
        let mut items = self.items.iter();
        if let Some((item, cells)) = items.next() {
            let mut cells = cells.iter();
            if let Some(cell) = cells.next() {
                let others = self.grid_map.get(cell).unwrap().iter();
                return Iter {
                    grid_map: &self.grid_map,
                    items,
                    item: Some(item),
                    returned: Vec::with_capacity(8),
                    cells,
                    others,
                }
            }
        }
        Iter {
            grid_map: &self.grid_map,
            items: [].iter(),
            item: None,
            returned: Vec::new(),
            cells: [].iter(),
            others: [].iter(),
        }
    }
}

fn advance_in_volume<RF: RealField, const D: usize>(v: &mut OVector<RF, Const<D>>, min: &OVector<RF, Const<D>>, max: &OVector<RF, Const<D>>) {
    for i in 0..D {
        #[cfg(debug_assertions)]
            let prev = v[i].clone();
        v[i] += RF::one();
        debug_assert_ne!(prev, v[i]);
        if v[i] <= max[i] {
            break;
        }
        if i != D - 1 {
            v[i] = min[i].clone();
        }
    }
}

impl<RF: RealField + AsPrimitive<G>, G: PrimInt + 'static, const D: usize> InGrid<G, D>
    for Vertex<RF, D>
{
    type Real = RF;

    fn occupied_cells<'a>(
        &'a self,
        step: &Self::Real,
        offset: &Self::Real,
        scale: &Self::Real,
    ) -> impl Iterator<Item = [G; D]> + 'a {
        struct Iter<RF: RealField, const D: usize, G: PrimInt> {
            pub min: OVector<RF, Const<D>>,
            pub max: OVector<RF, Const<D>>,
            a: OVector<RF, Const<D>>,
            b: OVector<RF, Const<D>>,
            current: OVector<RF, Const<D>>,
            offset: RF,
            _phantom: PhantomData<fn() -> G>,
        }
        impl<RF: RealField + AsPrimitive<G>, G: PrimInt + 'static, const D: usize> Iterator
            for Iter<RF, D, G>
        {
            type Item = [G; D];

            fn next(&mut self) -> Option<Self::Item> {
                while self.current[D - 1] <= self.max[D - 1] {
                    //  NB: if the coordinates are two large, this will cause bugs due to p+splat(1) == p
                    let old = self.current.clone();
                    let v = &mut self.current;
                    let max = &self.max;
                    let min = &self.min;
                    advance_in_volume(v, min, max);
                    if intersecting::aabb_segment(
                        &old.add_scalar(-self.offset.clone()),
                        &old.add_scalar(RF::one() + self.offset.clone()),
                        &self.a,
                        &self.b
                    ) {
                        return Some(old.data.0[0].clone().map(AsPrimitive::as_));
                    }
                }
                None
            }
        }
        let min = self
            .x
            .inf(&(self.x + self.v.scale(step.clone()).add_scalar(-offset.clone())))
            .coords
            .unscale(scale.clone())
            .map(RF::floor);
        let max = self
            .x
            .sup(&(self.x + self.v.scale(step.clone()).add_scalar(offset.clone())))
            .coords
            .unscale(scale.clone())
            .map(RF::floor);
        let a = self.x.coords.unscale(scale.clone());
        let b = &a + self.v.unscale(scale.clone());
        Iter {
            current: min.clone(),
            min,
            max,
            a,
            b,
            offset: offset.clone(),
            _phantom: Default::default(),
        }
    }
}

/// Iterate through the 1x...x1 cubes between min and max.
struct IterCuboid<RF: RealField, const D: usize, G: PrimInt> {
    min: OVector<RF, Const<D>>,
    max: OVector<RF, Const<D>>,
    current: OVector<RF, Const<D>>,
    _phantom: PhantomData<fn() -> G>,
}

impl<RF: RealField + AsPrimitive<G>, G: PrimInt + 'static, const D: usize> Iterator
for IterCuboid<RF, D, G>
{
    type Item = [G; D];

    fn next(&mut self) -> Option<Self::Item> {
        if self.current[D - 1] <= self.max[D - 1] {
            //  NB: if the coordinates are two large, this will cause bugs due to p+splat(1) == p
            let old = self.current.clone();
            advance_in_volume(&mut self.current, &self.min, &self.max);
            Some(old.data.0[0].map(AsPrimitive::as_))
        } else {
            None
        }
    }
}

impl<RF: RealField + AsPrimitive<G>, G: PrimInt + 'static, const D: usize> InGrid<G, D>
    for Segment<RF, D>
{
    type Real = RF;

    fn occupied_cells<'a>(
        &'a self,
        step: &Self::Real,
        offset: &Self::Real,
        scale: &Self::Real,
    ) -> impl Iterator<Item = [G; D]> + 'a {
        let min =
            self.0
                .x
                .inf(&(self.0.x + self.0.v.scale(step.clone()).add_scalar(-offset.clone())))
                .inf(
                    &self.1.x.inf(
                        &(self.1.x + self.1.v.scale(step.clone()).add_scalar(-offset.clone())),
                    ),
                )
                .coords
                .unscale(scale.clone())
                .map(RF::floor);
        let max =
            self.0
                .x
                .sup(&(self.0.x + self.0.v.scale(step.clone()).add_scalar(offset.clone())))
                .sup(
                    &self.1.x.sup(
                        &(self.1.x + self.1.v.scale(step.clone()).add_scalar(offset.clone())),
                    ),
                )
                .coords
                .unscale(scale.clone())
                .map(RF::floor);
        IterCuboid {
            current: min.clone(),
            min,
            max,
            _phantom: Default::default(),
        }
    }
}

impl<RF: RealField + AsPrimitive<G>, G: PrimInt + 'static, const D: usize> InGrid<G, D>
    for Triangle<RF, D>
{
    type Real = RF;

    fn occupied_cells<'a>(
        &'a self,
        step: &Self::Real,
        offset: &Self::Real,
        scale: &Self::Real,
    ) -> impl Iterator<Item = [G; D]> + 'a {
        let min =
            self.a
                .x
                .inf(&(self.a.x + self.a.v.scale(step.clone()).add_scalar(-offset.clone())))
                .inf(
                    &self.b.x.inf(
                        &(self.b.x + self.b.v.scale(step.clone()).add_scalar(-offset.clone())),
                    ),
                )
                .inf(
                    &self.c.x.inf(
                        &(self.c.x + self.c.v.scale(step.clone()).add_scalar(-offset.clone())),
                    ),
                )
                .coords
                .unscale(scale.clone())
                .map(RF::floor);
        let max =
            self.a
                .x
                .sup(&(self.a.x + self.a.v.scale(step.clone()).add_scalar(offset.clone())))
                .sup(
                    &self.b.x.sup(
                        &(self.b.x + self.b.v.scale(step.clone()).add_scalar(offset.clone())),
                    ),
                )
                .sup(
                    &self.c.x.sup(
                        &(self.c.x + self.c.v.scale(step.clone()).add_scalar(offset.clone())),
                    ),
                )
                .coords
                .unscale(scale.clone())
                .map(RF::floor);
        IterCuboid {
            current: min.clone(),
            min,
            max,
            _phantom: Default::default(),
        }
    }
}
