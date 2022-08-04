use std::io::Write;

use bevy::{prelude::*, reflect::TypeRegistry};
use bevy_rapier3d::prelude::{
    Collider, ImpulseJoint, NoUserData, RapierPhysicsPlugin, Restitution, RigidBody,
    SphericalJointBuilder,
};
use impostor_schemas::schemas;

fn main() {
    let mut type_registry = App::new()
        .register_type::<schemas::Primitive>()
        .register_type::<schemas::Transform>()
        .register_type::<String>()
        .add_plugins(DefaultPlugins)
        .world
        .resource::<TypeRegistry>().clone();

    let mut scene_world = World::default();
    scene_world
        .spawn()
        .insert(schemas::Primitive {
            shape: "cube".into(),
        })
        .insert(schemas::Transform::default())
        .with_children(|parent| {
            parent
                .spawn()
                .insert(schemas::Primitive {
                    shape: "uvsphere".into(),
                })
                // .insert(Restitution::coefficient(0.7))
                .insert(schemas::Transform::default());
        });
    let scene = DynamicScene::from_world(&scene_world, &type_registry);

    info!("{}", scene.serialize_ron(&type_registry).unwrap());

    let mut file = std::fs::File::create("crates/impostor/assets/scenes/start_scene.scn.ron").unwrap();
    file.write_all(scene.serialize_ron(&type_registry).unwrap().as_bytes())
        .unwrap();
}