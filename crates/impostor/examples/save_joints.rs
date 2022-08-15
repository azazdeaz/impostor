use bevy::{prelude::*, reflect::TypeRegistry};
use bevy_rapier3d::prelude::{
    Collider, ImpulseJoint, NoUserData, RapierPhysicsPlugin, Restitution, RigidBody,
    SphericalJointBuilder,
};
use impostor_schemas::schemas;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(bevy_rapier3d::render::RapierDebugRenderPlugin::default())
        .add_startup_system(save_scene_system.exclusive_system().at_end())
        .run();
}

#[derive(Component, Reflect, Clone)]
#[reflect(Component)]
enum SeedRigidBody {
    Dynamic,
    Fixed,
}
impl Default for SeedRigidBody {
    fn default() -> Self {
        SeedRigidBody::Dynamic
    }
}

fn save_scene_system(world: &mut World) {
    let mut scene_world = World::new();

    scene_world
        .spawn()
        .insert(schemas::Primitive {
            shape: "cube".into(),
        })
        // .insert(Collider::ball(1.0))
        // .insert(Restitution::coefficient(0.7))
        // .insert(schemas::Transform::default())
        .with_children(|parent| {
            parent
                .spawn()
                .insert(schemas::Primitive {
                    shape: "cube".into(),
                });
                // .insert(Restitution::coefficient(0.7))
                // .insert(schemas::Transform::default());
        });

    let type_registry = world.resource::<TypeRegistry>();
    let scene = DynamicScene::from_world(&scene_world, &type_registry);

    info!("{}", scene.serialize_ron(&type_registry).unwrap());
}
