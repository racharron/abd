use bevy::asset::Handle;
use bevy::prelude::{Component, Entity, Mesh, Resource};
use bevy::utils::HashMap;
use bevy_app::{App, Plugin};
use abd_kit::spatial::{Object, SpatialDB};
use abd_kit::spatial::sweep_prune::SweepPrune;

pub struct AbdTriMeshOnlyPlugin {

}

#[derive(Clone, Debug, Component)]
pub struct RenderMeshCollider;

#[derive(Clone, Debug, Component)]
pub struct MeshCollider(Handle<Mesh>);

#[derive(Default, Resource)]
pub struct CollisionCache<C> {
    cache: HashMap<(Entity, Entity), C>
}

#[derive(Default, Resource)]
pub struct Broadphase<C>(C);

impl Plugin for AbdTriMeshOnlyPlugin {
    fn build(&self, app: &mut App) {
        todo!()
    }
}




