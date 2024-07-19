use num::PrimInt;
use std::hash::Hash;
use std::collections::HashMap;

///! A spatial datastructure that stores items in buckets based off an infinite 3D grid.

/// A hierarchical hash-grid.  Rather than requiring a specific grid size, it has power of two
/// grid sizes
pub struct HierarchicalHashGrid<S: PrimInt, G: PrimInt, T: InHGrid<S, G, D>, const D: usize> {
    table: HashMap<([G; D], S), HGridEntry<S, T, D>>,
}

/// The type of items that can be inserted into a hierarchical grid.
///
/// Do note that all the occupied cells are at the same scale.
pub trait InHGrid<S: PrimInt, G: PrimInt, const DIM: usize> {
    /// The scale of the occupied cells.
    fn scale(&self) -> S;
    /// The occupied grid cells at scale [`self.scale`].  All cells returned *must* be unique.
    fn occupied_cells<'a>(&'a self) -> impl Iterator<Item = [G; DIM]> + 'a;
}

struct HGridEntry<S, T, const D: usize> {
    next: S,
    items: Vec<T>,
}

impl<const DIM: usize, S: PrimInt + Hash, G: PrimInt + Hash, T: InHGrid<S, G, DIM> + Clone>
    HierarchicalHashGrid<S, G, T, DIM>
{
    /// Adds an item to a grid cell at the specified scale.  Assumes that the highest scale items
    /// come first.
    pub fn from_sorted(mut items: impl Iterator<Item = T>) -> Self {
        if let Some(first) = items.next() {
            let top = first.scale();
            let mut scales = vec![top.clone()];
            let mut table = HashMap::new();
            table.extend(first.occupied_cells().map(|cell| {
                (
                    (cell, top.clone()),
                    HGridEntry {
                        next: top.clone(),
                        items: vec![first.clone()],
                    },
                )
            }));
            for item in items {
                let scale = item.scale();
                debug_assert!(&scale <= scales.last().unwrap());
                if &scale < scales.last().unwrap() {
                    scales.push(scale.clone());
                }
                'i: for cell in item.occupied_cells() {
                    if let Some(entry) = table.get_mut(&(cell.clone(), scale.clone())) {
                        entry.items.push(item.clone());
                    } else {
                        for upper in &scales {
                            let shift = upper.clone() - scale.clone();
                            if table.contains_key(&(
                                cell.clone().map(|x| x >> shift.to_usize().unwrap()),
                                upper.clone(),
                            )) {
                                table.insert(
                                    (cell.clone(), scale.clone()),
                                    HGridEntry {
                                        next: upper.clone(),
                                        items: vec![item.clone()],
                                    },
                                );
                                continue 'i;
                            }
                        }
                        table.insert(
                            (cell.clone(), scale.clone()),
                            HGridEntry {
                                next: scale.clone(),
                                items: vec![item.clone()],
                            },
                        );
                    }
                }
            }
            Self { table }
        } else {
            Self {
                table: HashMap::new(),
            }
        }
    }
}

impl<S: PrimInt, T, const D: usize> HGridEntry<S, T, D> {
    pub fn next<'a, 'b>(&'a self, current: &'b S) -> Option<&'a S> {
        if self.next.eq(current) {
            None
        } else {
            Some(&self.next)
        }
    }
}
