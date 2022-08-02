use bevy::{prelude::*, reflect::TypeRegistry};
use bevy_rapier3d::prelude::{
    Collider, ImpulseJoint, NoUserData, RapierPhysicsPlugin, Restitution, RigidBody,
    SphericalJointBuilder,
};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(bevy_rapier3d::render::RapierDebugRenderPlugin::default())
        .add_startup_system(save_scene_system.exclusive_system())
        .run();
}

fn save_scene_system(world: &mut World) {
    let mut scene_world = World::new();

    scene_world
        .spawn()
        .insert(RigidBody::Dynamic)
        // .insert(Collider::ball(0.5))
        // .insert(Restitution::coefficient(0.7))
        .insert(Transform::default());

    let type_registry = world.resource::<TypeRegistry>();
    let scene = DynamicScene::from_world(&scene_world, &type_registry);

    info!("{}", scene.serialize_ron(&type_registry).unwrap());
}
