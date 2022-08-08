use std::io::Write;

use bevy::{prelude::*, reflect::TypeRegistry};
use bevy_rapier3d::prelude::{
    Collider, ImpulseJoint, NoUserData, RapierPhysicsPlugin, Restitution, RigidBody,
    SphericalJointBuilder,
};
use impostor_schemas::schemas::{self, SchemasPlugin};

fn main() {
    let mut type_registry = App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(SchemasPlugin)
        .world
        .resource::<TypeRegistry>()
        .clone();

    let mut scene_world = World::default();
    let ent1 = scene_world
        .spawn()
        .insert(schemas::Name("Link 1".into()))
        .insert(schemas::Primitive {
            shape: "cube".into(),
        })
        .insert(schemas::ColliderCuboid {
            hx: 0.5,
            hy: 0.5,
            hz: 0.5,
        })
        .insert(schemas::RigidBody("Dynamic".into()))
        .insert(Transform::from_xyz(0., 5., 0.))
        .insert(schemas::Restitution { coefficient: 0.7 })
        .id();

    scene_world
        .spawn()
        .insert(schemas::Name("Link 2".into()))
        .insert(schemas::Primitive {
            shape: "cube".into(),
        })
        .insert(schemas::ImpulseJoint {
            parent: ent1,
            joint: schemas::RevolutJoint {
                axis: Vec3::X,
                local_anchor1: Vec3::X,
                local_anchor2: Vec3::Y,
            },
        })
        .insert(schemas::Restitution { coefficient: 0.7 })
        .insert(schemas::ColliderCuboid {
            hx: 0.5,
            hy: 0.5,
            hz: 0.5,
        })
        .insert(schemas::RigidBody("Dynamic".into()));
    let scene = DynamicScene::from_world(&scene_world, &type_registry);

    info!("{}", scene.serialize_ron(&type_registry).unwrap());

    let mut file =
        std::fs::File::create("crates/impostor/assets/scenes/start_scene.scn.ron").unwrap();
    file.write_all(scene.serialize_ron(&type_registry).unwrap().as_bytes())
        .unwrap();
}
