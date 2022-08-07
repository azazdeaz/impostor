use bevy::prelude::*;

#[derive(Component, Reflect, Default)]
#[reflect(Component)]
pub struct Primitive {
    pub shape: String,
}

#[derive(Component, Reflect, Default)]
#[reflect(Component)]
pub struct ColliderCuboid {
    hx: f32,
    hy: f32,
    hz: f32,
}
#[derive(Component, Reflect, Default)]
#[reflect(Component)]
pub struct ColliderCylinder {
    half_height: f32,
    radius: f32,
}

#[derive(Component, Reflect, Default)]
#[reflect(Component)]
pub struct ImpulseJoint {
    parent: Option<Entity>,
    joint: RevolutJoint,
}

#[derive(Component, Reflect, Default)]
#[reflect(Component)]
pub struct RevolutJoint {
    axis: Vec3,
    local_anchor1: Vec3,
    local_anchor2: Vec3,
}

#[derive(Component, Reflect, Default)]
#[reflect(Component)]
pub struct Tags(Vec<String>);

pub struct SchemasPlugin;

impl Plugin for SchemasPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<Primitive>()
            .register_type::<ColliderCuboid>()
            .register_type::<ColliderCylinder>()
            .register_type::<ImpulseJoint>()
            .register_type::<RevolutJoint>()
            .register_type::<Tags>();
    }
}
