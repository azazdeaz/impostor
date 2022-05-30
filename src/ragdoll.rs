use bevy::prelude::*;
use bevy_rapier3d::{prelude::*, rapier::prelude::JointAxis};

fn main() {
    App::new()
        .insert_resource(Msaa { samples: 4 })
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_scene)
        .run();
}

fn setup_scene(mut commands: Commands) {
    let mut prev_section = None;
    let joint_height = 4.0;
    let radius = 1.0;
    let joint_count = 4;

    /* Create the joint chain */
    for i in 0..joint_count {
        let section = commands
            .spawn()
            .insert(RigidBody::Dynamic)
            .insert(Transform::from_xyz(0.0, joint_height * (i as f32), 0.0))
            .with_children(|children| {
                children
                    .spawn()
                    .insert(Collider::capsule_y(joint_height / 2.0, radius * 1.0))
                    .insert(CollisionGroups::new(0b1000, 0b0100))
                    .insert(Transform::from_xyz(0.0, joint_height / 2.0, 0.0));
            })
            .id();

        if let Some(prev_section) = prev_section {
            let rapier_joint = SphericalJointBuilder::new()
                .local_anchor1(Vec3::new(0.0, joint_height, 0.0))
                .local_anchor2(Vec3::new(0.0, 0.0, 0.0))
                .motor_position(JointAxis::AngX, 0.0, 100.0, 1.0)
                .motor_position(JointAxis::AngY, 0.0, 100.0, 1.0)
                .motor_position(JointAxis::AngZ, 0.0, 100.0, 1.0);

            commands
                .entity(section)
                .insert(ImpulseJoint::new(prev_section, rapier_joint));
            // commands.entity(prev_section).push_children(&[section]); // Is this needed?
        }

        prev_section = Some(section);
    }

    /* Create the ground. */
    commands
        .spawn()
        .insert(Collider::cuboid(10.0, 0.1, 10.0))
        .insert(Transform::from_xyz(0.0, -2.0, 0.0));

    /* Create the bouncing ball. */
    commands
        .spawn()
        .insert(RigidBody::Dynamic)
        .insert(Collider::ball(1.0))
        .insert(Restitution::coefficient(0.7))
        .insert(Transform::from_xyz(
            0.1,
            (joint_count as f32) * joint_height + 4.0,
            0.1,
        ));

    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_xyz(-13.0, 24.0, 17.0)
            .looking_at(Vec3::new(0.0, 12.0, 0.0), Vec3::Y),
        ..default()
    });
}
