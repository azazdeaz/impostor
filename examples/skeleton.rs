use bevy::prelude::*;
use bevy_rapier3d::{
    prelude::*,
    rapier::prelude::{JointAxis, MotorModel},
};

fn main() {
    App::new()
        .insert_resource(Msaa { samples: 4 })
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_scene)
        .run();
}

fn create_stem_skeleton(
    commands: &mut Commands,
    joint_count: i32,
    prev_section: Entity,
    prev_section_height: f32,
    rotation: Quat,
    radius: (f32, f32),
) -> Vec<Entity> {
    let joint_height = 4.0;
    let mut prev_section = prev_section;
    let mut prev_section_height = prev_section_height;
    let mut sections = Vec::with_capacity(joint_count as usize);

    for i in 0..joint_count {
        let transform =
            Transform::from_xyz(0.0, joint_height * (i as f32), 0.0).with_rotation(rotation);
        let (rot_y, rot_z, rot_x) = if i == 0 {
            rotation.to_euler(EulerRot::YZX)
        } else {
            (0.0, 0.0, 0.0)
        };
        let r = radius.0 + (radius.1 - radius.0) * (i as f32 / (joint_count - 1) as f32  );
        println!("x: {} y: {} z: {}", rot_x, rot_y, rot_z);
        let section = commands
            .spawn()
            .insert(RigidBody::Dynamic)
            .insert(transform)
            .with_children(|children| {
                children
                    .spawn()
                    .insert(Collider::capsule_y(joint_height / 2.0, r))
                    .insert(CollisionGroups::new(0b1000, 0b0100))
                    .insert(Transform::from_xyz(0.0, joint_height / 2.0, 0.0));
            })
            .id();

        let rapier_joint = SphericalJointBuilder::new()
            .local_anchor1(Vec3::new(0.0, prev_section_height, 0.0))
            .local_anchor2(Vec3::new(0.0, 0.0, 0.0))
            .motor_position(JointAxis::AngX, rot_x, 100000.0, 10000.0)
            .motor_position(JointAxis::AngY, rot_y, 100000.0, 10000.0)
            .motor_position(JointAxis::AngZ, rot_z, 100000.0, 10000.0)
            .motor_model(JointAxis::AngX, MotorModel::ForceBased)
            .motor_model(JointAxis::AngY, MotorModel::ForceBased)
            .motor_model(JointAxis::AngZ, MotorModel::ForceBased);
        commands
            .entity(section)
            .insert(ImpulseJoint::new(prev_section, rapier_joint));
        // commands.entity(prev_section).push_children(&[section]); // Is this needed?

        prev_section = section;
        sections.push(section);
        prev_section_height = joint_height;
    }
    return sections;
}

fn setup_scene(mut commands: Commands) {
    let root = commands
        .spawn()
        .insert(RigidBody::Fixed)
        // .insert(Transform::from_xyz(i as f32 * 1.5, 0.0, 0.0))
        .id();

    let joint_count = 4;

    let joints = create_stem_skeleton(
        &mut commands,
        joint_count,
        root,
        0.0,
        Quat::from_euler(EulerRot::YZX, 0.0, 0.0, 0.0),
        (0.6, 0.4)
    );

    create_stem_skeleton(
        &mut commands,
        3,
        joints[1],
        0.0,
        Quat::from_euler(EulerRot::YZX, 0.0, 0.5, 0.0),
    (0.5,0.2)
    );
    create_stem_skeleton(
        &mut commands,
        3,
        joints[2],
        0.0,
        Quat::from_euler(EulerRot::YZX, 0.0, -0.2, 0.0),
        (0.42,0.14)
    );

    /* Create the joint chain */
    // for i in 0..joint_count {
    //     let section = commands
    //         .spawn()
    //         .insert(RigidBody::Dynamic)
    //         .insert(Transform::from_xyz(0.0, joint_height * (i as f32), 0.0))
    //         .with_children(|children| {
    //             children
    //                 .spawn()
    //                 .insert(Collider::capsule_y(joint_height / 2.0, radius * 1.0))
    //                 .insert(CollisionGroups::new(0b1000, 0b0100))
    //                 .insert(Transform::from_xyz(0.0, joint_height / 2.0, 0.0));
    //         })
    //         .id();

    //     let local_anchor1_y = if prev_section == root {
    //         0.0
    //     } else {
    //         joint_height
    //     };
    //     let rapier_joint = SphericalJointBuilder::new()
    //         .local_anchor1(Vec3::new(0.0, local_anchor1_y, 0.0))
    //         .local_anchor2(Vec3::new(0.0, 0.0, 0.0))
    //         .motor_position(JointAxis::AngX, 0.0, 500.0, 500.0)
    //         .motor_position(JointAxis::AngY, 0.0, 500.0, 500.0)
    //         .motor_position(JointAxis::AngZ, 0.0, 500.0, 500.0);
    //     commands
    //         .entity(section)
    //         .insert(ImpulseJoint::new(prev_section, rapier_joint));
    //     // commands.entity(prev_section).push_children(&[section]); // Is this needed?

    //     prev_section = section;
    // }

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
            (joint_count as f32) * 4.0 + 4.0,
            0.1,
        ));

    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_xyz(-13.0, 24.0, 17.0)
            .looking_at(Vec3::new(0.0, 12.0, 0.0), Vec3::Y),
        ..default()
    });
}
