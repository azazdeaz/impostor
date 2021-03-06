use bevy::{
    ecs::schedule::ShouldRun,
    prelude::*,
    render::mesh::{
        skinning::{SkinnedMesh, SkinnedMeshInverseBindposes},
        Indices, PrimitiveTopology,
    },
};
// use bevy_inspector_egui::{Inspectable, InspectorPlugin, WorldInspectorPlugin};
use bevy_rapier3d::{
    prelude::*,
    rapier::prelude::{JointAxis, MotorModel},
};
use itertools::Itertools;
use std::rc::Rc;
use std::{cell::RefCell, fmt};

fn main() {
    App::new()
        .insert_resource(Msaa { samples: 4 })
        .add_plugins(DefaultPlugins)
        .insert_resource(GrowSteps::from_steps(16))
        // .add_plugin(InspectorPlugin::<GrowSteps>::new_insert_manually())
        // .add_plugin(WorldInspectorPlugin::new())
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_plant)
        .add_startup_system(setup_scene)
        .add_system_set(
            SystemSet::new()
                .with_run_criteria(run_if_growing)
                .with_system(count_grow_steps)
                .with_system(grow)
                .with_system(extend),
        )
        .add_system(update_mesh)
        .add_system(add_bodies)
        .register_type::<Stem>()
        .run();
}

// #[derive(Inspectable, Default)]
struct GrowSteps {
    current: u32,
    // #[inspectable(min = 1, max = 30)]
    max: u32,
    mesh_updated: bool,
}
impl GrowSteps {
    fn from_steps(max: u32) -> Self {
        Self {
            current: 0,
            max: max,
            mesh_updated: false,
        }
    }
    fn step(&mut self) {
        self.current += 1;
        self.mesh_updated = false;
    }
    fn is_done(&self) -> bool {
        self.max <= self.current
    }
    fn set_mesh_updated(&mut self) {
        self.mesh_updated = true;
    }
}

fn run_if_growing(grow_steps: Res<GrowSteps>) -> ShouldRun {
    if grow_steps.is_done() {
        ShouldRun::No
    } else {
        ShouldRun::Yes
    }
}

fn count_grow_steps(mut grow_steps: ResMut<GrowSteps>) {
    println!("STEP: {}", grow_steps.current);
    grow_steps.step();
}

#[derive(Component)]
struct AxisRoot {}
#[derive(Component)]
struct NextAxe(Entity);
#[derive(Component, Debug)]
struct Branches(Vec<Entity>);

#[derive(Component, Debug)]
struct Length(f32);
#[derive(Component, Debug)]
struct Radius(f32);

#[derive(Reflect, Default, Component, Debug, Clone)]
#[reflect(Component)]
struct Stem {
    order: u32,
    max_length: f32,
    direction: Quat,
}

#[derive(Bundle)]
struct StemBundle {
    stem: Stem,
    length: Length,
    radius: Radius,
}

impl Default for StemBundle {
    fn default() -> Self {
        Self {
            stem: Stem {
                order: 0,
                max_length: 4.0,
                direction: Quat::default(),
            },
            length: Length(1.0),
            radius: Radius(1.0),
        }
    }
}

fn grow(mut commands: Commands, query: Query<(Entity, Option<&Parent>, &Radius, &Length, &Stem)>) {
    for (stem_id, parent, Radius(radius), Length(length), stem) in query.iter() {
        let prev_radius = parent
            .and_then(|parent| query.get(parent.get()).ok())
            .and_then(|q| Some((*q.2).0))
            .unwrap_or(1.0);
        commands
            .entity(stem_id)
            .insert(Radius(radius + (prev_radius - radius) / 12.0))
            .insert(Length(length + (stem.max_length - length) / 3.0));
    }
}

fn extend(mut commands: Commands, query: Query<(Entity, &Stem, &Length), (Without<NextAxe>)>) {
    for (id, stem, Length(length)) in query.iter() {
        if *length > stem.max_length * 0.8 {
            let next = commands
                .spawn()
                .insert(Name::new("Stem Node"))
                .insert_bundle(StemBundle {
                    stem: Stem {
                        order: stem.order,
                        max_length: 4.0,
                        direction: Quat::default(),
                    },
                    length: Length(0.2),
                    radius: Radius(0.001),
                    ..Default::default()
                })
                .id();
            commands
                .entity(id)
                .push_children(&[next])
                .insert(NextAxe(next));
        }
    }
}

fn branch_out(mut commands: Commands, query: Query<(Entity, &Stem, &Length, Option<&mut Branches>)>) {
    for (id, stem, Length(length), branches) in query.iter() {
        if *length > stem.max_length * 0.8 {
            let next = commands
                .spawn()
                .insert(Name::new("Stem Node"))
                .insert_bundle(StemBundle {
                    stem: Stem {
                        order: stem.order,
                        max_length: 4.0,
                        direction: Quat::default(),
                    },
                    length: Length(0.2),
                    radius: Radius(0.001),
                    ..Default::default()
                })
                .id();
            commands
                .entity(id)
                .push_children(&[next])
                .insert(NextAxe(next));
        }
    }
}

// fn branch_out(q_root: Query<(Entity, &NextAxe), (With<AxisRoot>, With)>, q_stem: Query<(&Stem, &NextAxe)>) {
//     for root in q_root.iter() {
//         let mut no_branch_length = 0.0;
//         let mut prev = root;
//         while len(next) = q_stem()
//         if let Ok(mut stem) = q_stem.get_mut(stem_id) {
//             stem.length += (stem.max_length - stem.length) / 3.0;
//             stem.radius += (prev_radius - stem.radius) / 12.0;
//         }
//     }
// }
// fn create_plant_joint(stiffness:f32, damping: f32) -> SphericalJointBuilder {
//     let rapier_joint = SphericalJointBuilder::new()
//         .local_anchor1(Vec3::new(0.0, 0.0, 0.0))
//         .local_anchor2(Vec3::new(0.0, -ln.data.length, 0.0))
//         .motor_position(JointAxis::AngX, rot_x, stiffness, damping)
//         .motor_position(JointAxis::AngY, rot_y, stiffness, damping)
//         .motor_position(JointAxis::AngZ, rot_z, stiffness, damping)
//         .motor_model(JointAxis::AngX, MotorModel::ForceBased)
//         .motor_model(JointAxis::AngY, MotorModel::ForceBased)
//         .motor_model(JointAxis::AngZ, MotorModel::ForceBased)
// }

fn add_bodies(
    mut commands: Commands,
    mut grow_steps: ResMut<GrowSteps>,
    q_root: Query<Entity, (With<AxisRoot>, Without<RigidBody>)>,
    q_stems: Query<(&Stem, &Length, &Radius, Option<&Children>)>,
) {
    if !grow_steps.is_done() {
        return;
    }

    for root in q_root.iter() {
        let base = commands
            .spawn()
            .insert(RigidBody::Fixed)
            .insert(GlobalTransform::identity())
            .insert(Transform::from_xyz(0.0, 0.0, 0.0))
            .id();
        let mut next_step = Some((base, root));
        while let Some((prev_node, node)) = next_step {
            next_step = None;

            if let Some((stem, Length(length), Radius(radius), children)) = q_stems.get(node).ok() {
                let transform = Transform::from_xyz(0.0, *length, 0.0);
                let (rot_y, rot_z, rot_x) = (0.0, 0.0, 0.0); //transform.rotation.to_euler(EulerRot::YZX);
                println!("rot_x {} rot_y {} rot_z {}", rot_x, rot_y, rot_z);

                let rapier_joint = SphericalJointBuilder::new()
                    .local_anchor1(Vec3::new(0.0, 0.0, 0.0))
                    .local_anchor2(Vec3::new(0.0, -length, 0.0))
                    .motor_position(JointAxis::AngX, rot_x, 2000.0 * radius, 10000.0)
                    .motor_position(JointAxis::AngY, rot_y, 2000.0 * radius, 10000.0)
                    .motor_position(JointAxis::AngZ, rot_z, 2000.0 * radius, 10000.0)
                    .motor_model(JointAxis::AngX, MotorModel::ForceBased)
                    .motor_model(JointAxis::AngY, MotorModel::ForceBased)
                    .motor_model(JointAxis::AngZ, MotorModel::ForceBased);

                commands
                    .entity(node)
                    .insert(RigidBody::Dynamic)
                    .insert(transform)
                    .insert(GlobalTransform::identity())
                    .with_children(|children| {
                        children
                            .spawn()
                            .insert(Collider::capsule_y(length / 2.0, *radius))
                            .insert(CollisionGroups::new(0b1000, 0b0100))
                            .insert(Transform::from_xyz(0.0, -length / 2.0, 0.0));
                    })
                    .insert(ImpulseJoint::new(prev_node, rapier_joint));

                next_step = children
                    .and_then(|children| children.iter().find(|c| q_stems.get(**c).is_ok()))
                    .and_then(|next| Some((node, *next)));
            }
        }
    }
}

fn update_mesh(
    mut grow_steps: ResMut<GrowSteps>,
    q_root: Query<Entity, With<AxisRoot>>,
    q_stems: Query<(&Stem, &Children)>,
) {
    if !grow_steps.is_done() || grow_steps.mesh_updated {
        return;
    }
    grow_steps.set_mesh_updated();

    for root in q_root.iter() {
        let mut prev = Some(&root);
        while let Some(stem_id) = prev {
            prev = if let Ok((stem, children)) = q_stems.get(*stem_id) {
                println!("STEM {:?}", stem);
                children.first()
            } else {
                None
            }
        }
    }
}

fn setup_plant(
    mut commands: Commands,
    // mut meshes: ResMut<Assets<Mesh>>,
    // mut materials: ResMut<Assets<StandardMaterial>>,
    // mut skinned_mesh_inverse_bindposes_assets: ResMut<Assets<SkinnedMeshInverseBindposes>>,
    // asset_server: Res<AssetServer>,
) {
    commands
        .spawn()
        .insert(Name::new("Root Node"))
        .insert_bundle(StemBundle {
            stem: Stem {
                order: 0,
                max_length: 4.0,
                direction: Quat::default(),
            },
            length: Length(0.2),
            radius: Radius(0.001),
            ..Default::default()
        })
        .insert(AxisRoot {});

    let root_joint = commands
        .spawn()
        .insert(RigidBody::Fixed)
        .insert(GlobalTransform::identity())
        .insert(Transform::from_xyz(0.0, 0.0, 0.0))
        .id();
}

fn setup_scene(mut commands: Commands) {
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
        .insert(Transform::from_xyz(0.1, (6.0 as f32) * 4.0 + 4.0, 0.1));

    commands.spawn_bundle(Camera3dBundle {
        transform: Transform::from_xyz(-13.0 / 1.0, 24.0 / 1.0, 17.0 / 1.0)
            .looking_at(Vec3::new(0.0, 8.0, 0.0), Vec3::Y),
        ..default()
    });
}
