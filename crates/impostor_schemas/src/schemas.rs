use bevy::{
    ecs::{
        entity::{EntityMap, MapEntities, MapEntitiesError},
        reflect::{ReflectComponent, ReflectMapEntities},
    },
    prelude::*,
};
#[derive(Component, Reflect, Default)]
#[reflect(Component)]
pub struct Primitive {
    pub shape: String,
}

#[derive(Component, Reflect)]
#[reflect(Component)]
pub struct RigidBody(pub String);
impl Default for RigidBody {
    fn default() -> Self {
        Self("Dynamic".into())
    }
}

#[derive(Component, Reflect, Default)]
#[reflect(Component)]
pub struct ColliderCuboid {
    pub hx: f32,
    pub hy: f32,
    pub hz: f32,
}
#[derive(Component, Reflect, Default)]
#[reflect(Component)]
pub struct ColliderCylinder {
    pub half_height: f32,
    pub radius: f32,
}


#[derive(Component, Reflect)]
#[reflect(Component, MapEntities)]
pub struct ImpulseJoint {
    pub parent: Entity,
    pub joint: RevolutJoint,
}
impl FromWorld for ImpulseJoint {
    fn from_world(_world: &mut World) -> Self {
        Self {
            parent: Entity::from_raw(u32::MAX),
            joint: Default::default(),
        }
    }
}
impl MapEntities for ImpulseJoint {
    fn map_entities(&mut self, entity_map: &EntityMap) -> Result<(), MapEntitiesError> {
        if let Ok(mapped_entity) = entity_map.get(self.parent) {
            self.parent = mapped_entity;
        }
        Ok(())
    }
}

#[derive(Component, Reflect, Default)]
#[reflect(Component)]
pub struct RevolutJoint {
    pub axis: Vec3,
    pub local_anchor1: Vec3,
    pub local_anchor2: Vec3,
}

#[derive(Component, Reflect, Default)]
#[reflect(Component)]
pub struct Restitution {
    pub coefficient: f32,
}

#[derive(Component, Reflect, Default)]
#[reflect(Component)]
pub struct Tag(pub String);

#[derive(Component, Reflect, Default)]
#[reflect(Component)]
pub struct Name(pub String);

pub struct SchemasPlugin;

impl Plugin for SchemasPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<ColliderCuboid>()
            .register_type::<ColliderCylinder>()
            .register_type::<ImpulseJoint>()
            .register_type::<Name>()
            .register_type::<Primitive>()
            .register_type::<RevolutJoint>()
            .register_type::<Restitution>()
            .register_type::<RigidBody>()
            .register_type::<Tag>();
    }
}
