use std::{f32::consts::PI, io::Write};

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
    // let ent1 = scene_world
    //     .spawn()
    //     .insert(schemas::Name("Link 1".into()))
    //     .insert(schemas::Editable::default())
    //     .insert(schemas::Primitive {
    //         shape: "cube".into(),
    //     })
    //     .insert(schemas::ColliderCuboid {
    //         hx: 0.5,
    //         hy: 0.5,
    //         hz: 0.5,
    //     })
    //     .insert(schemas::RigidBody("Dynamic".into()))
    //     .insert(schemas::Transform(Transform::from_xyz(0., 50., 0.)))
    //     .insert(schemas::Restitution { coefficient: 0.7 })
    //     .id();

    // scene_world
    //     .spawn()
    //     .insert(schemas::Editable::default())
    //     .insert(schemas::Name("Link 2".into()))
    //     .insert(schemas::Primitive {
    //         shape: "cube".into(),
    //     })
    //     .insert(schemas::ImpulseJoint {
    //         parent: ent1,
    //         joint: schemas::RevolutJoint {
    //             axis: Vec3::X,
    //             local_anchor1: Vec3::X,
    //             local_anchor2: Vec3::Y,
    //         },
    //     })
    //     .insert(schemas::Restitution { coefficient: 0.7 })
    //     .insert(schemas::ColliderCuboid {
    //         hx: 0.5,
    //         hy: 0.5,
    //         hz: 0.5,
    //     })
    //     .insert(schemas::RigidBody("Dynamic".into()));

    create_car(&mut scene_world);

    let scene = DynamicScene::from_world(&scene_world, &type_registry);

    info!("{}", scene.serialize_ron(&type_registry).unwrap());

    let mut file =
        std::fs::File::create("crates/impostor/assets/scenes/start_scene.scn.ron").unwrap();
    file.write_all(scene.serialize_ron(&type_registry).unwrap().as_bytes())
        .unwrap();
}

fn create_car(commands: &mut World) {
    let width = 2.0;
    let length = 4.0;
    let height = 0.4;
    let wheel_width = 1.2;
    let wheel_radius = 1.0;

    let chasis = commands
        .spawn()
        .insert(schemas::Name("Chasis".into()))
        .insert(schemas::RigidBody("Dynamic".into()))
        .insert(schemas::ColliderCuboid::new(
            width / 2.,
            height / 2.,
            length / 2.,
        ))
        // .insert(Restitution::coefficient(0.7))
        .insert(schemas::Transform(Transform::from_xyz(0.0, 4.0, 0.0)))
        .insert(schemas::CollisionGroups::new(0b1111, 0b0111))
        .id();

    let mut add_wheel = |front: f32, left: f32| {
        let mut wheel = commands.spawn();
        wheel
            .insert(schemas::Name(format!("Wheel {} {}", front, left)))
            .insert(schemas::RigidBody("Dynamic".into()))
            .insert(schemas::ImpulseJoint {
                parent: chasis,
                joint: schemas::RevolutJoint {
                    axis: Vec3::X,
                    local_anchor1: Vec3::new(
                        (width / 2. + wheel_width * 0.6) * left,
                        0.0,
                        length / 2. * front,
                    ),
                    local_anchor2: Vec3::new(0.0, 0.0, 0.0),
                },
            })
            .insert(schemas::Transform(Transform::from_xyz(0., 6., 0.)))
            .with_children(|parent| {
                parent
                    .spawn()
                    .insert(schemas::ColliderCylinder::new(
                        wheel_width / 2.0,
                        wheel_radius,
                    ))
                    .insert(schemas::Restitution { coefficient: 0.7 })
                    .insert(schemas::Transform(Transform::from_rotation(
                        Quat::from_axis_angle(Vec3::Z, PI / 2.),
                    )))
                    .insert(schemas::CollisionGroups::new(0b1000, 0b1111));
            });
        if left > 0. {
            wheel.insert(schemas::Tag("LeftWheel".into()));
        } else {
            wheel.insert(schemas::Tag("RightWheel".into()));
        }
    };

    add_wheel(1., 1.);
    add_wheel(1., -1.);
    add_wheel(-1., 1.);
    add_wheel(-1., -1.);
}
