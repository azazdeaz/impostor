use bevy::{pbr::AmbientLight, prelude::*};
use bevy_rapier3d::{prelude::*, rapier::prelude::JointAxis};

fn main() {
    App::new()
        .insert_resource(Msaa { samples: 4 })
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugRenderPlugin::default())
        // .add_plugin(ConfigCam)
        .insert_resource(AmbientLight {
            brightness: 0.3,
            ..Default::default()
        })
        .add_startup_system(setup_scene)
        .add_system(handle_physics_commands)
        .run();
}

fn handle_physics_commands(
    keys: Res<Input<KeyCode>>,
    mut rapier_config: ResMut<RapierConfiguration>,
) {
    let go = keys.just_pressed(KeyCode::Space);
    let go = true;
    rapier_config.physics_pipeline_active = go;
    rapier_config.query_pipeline_active = go;
}

fn setup_scene(mut commands: Commands) {
    let mut prev_joint = None;
    let mut joint_entities = Vec::new();
    let root = commands.spawn().insert(RigidBody::Fixed).id();
    let joint_height = 4.0;
    let radius = 1.0;

    for i in 0..4 {
        let mut joint = commands.spawn();
        let joint = if prev_joint.is_none() {
            let rapier_joint = SphericalJointBuilder::new()
                .local_anchor1(Vec3::new(0.0, 0.0, 0.0))
                .local_anchor2(Vec3::new(0.0, 0.0, 0.0))
                .motor_position(JointAxis::X, 0.0, 0.05, 0.2)
                .motor_position(JointAxis::Y, 0.0, 0.05, 0.2)
                .motor_position(JointAxis::Z, 0.0, 0.05, 0.2);
            joint
                .insert(Transform::from_xyz(0.0, 0.0, 0.0))
                .insert(ImpulseJoint::new(root, rapier_joint))
        } else {
            let rapier_joint = SphericalJointBuilder::new()
                .local_anchor1(Vec3::new(0.0, joint_height, 0.0))
                .local_anchor2(Vec3::new(0.0, 0.0, 0.0))
                .motor_position(JointAxis::X, 0.0, 0.05, 0.2)
                .motor_position(JointAxis::Y, 0.0, 0.05, 0.2)
                .motor_position(JointAxis::Z, 0.0, 0.05, 0.2);
            joint
                .insert(Transform::from_xyz(0.0, joint_height, 0.0))
                .insert(ImpulseJoint::new(prev_joint.unwrap(), rapier_joint))
        };
        let joint_id = joint
            .insert(GlobalTransform::identity())
            .insert(RigidBody::Dynamic)
            .insert(Transform::from_xyz(0.0, joint_height * (i as f32), 0.0))
            // .insert(GravityScale(0.1))
            .with_children(|children| {
                children
                    .spawn()
                    .insert(Collider::capsule_y(joint_height / 2.0, radius * 1.0))
                    .insert(CollisionGroups::new(0b1000, 0b0100))
                    .insert(Transform::from_xyz(0.0, joint_height / 2.0, 0.0));
                // .insert(GlobalTransform::identity());
                // children.spawn_bundle(PbrBundle {
                //     mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
                //     material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
                //     transform: Transform::from_xyz(0.0, 0.5, 0.0),
                //     ..default()
                // });
            })
            // .insert(ExternalForce {
            //     force: Vec3::new(10.0, 20.0, 30.0),
            //     torque: Vec3::new(1.0, 2.0, 3.0),
            // })
            // .insert(ExternalImpulse {
            //     impulse: Vec3::new(1.0, 2.0, 3.0),
            //     torque_impulse: Vec3::new(0.1, 0.2, 0.3),
            // })
            .id();
        if let Some(prev_joint) = prev_joint {
            commands.entity(prev_joint).push_children(&[joint_id]);
        }
        joint_entities.push(joint_id);
        prev_joint = Some(joint_id);
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
        .insert(Transform::from_xyz(0.1, 20.0, 0.0))
        // .insert(ExternalImpulse {
        //         impulse: Vec3::new(10.0, 0.0, 0.0),
        //         torque_impulse: Vec3::new(10.0, 0.0, 0.0),
        //     })
        //     .insert(ExternalForce {
        //             force: Vec3::new(10.0, 20.0, 30.0),
        //             torque: Vec3::new(1.0, 2.0, 3.0),
        //         })
                ;

    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_xyz(-13.0, 24.0, 17.0).looking_at(Vec3::new(0.0, 12.0, 0.0), Vec3::Y),
        ..default()
    });
}
