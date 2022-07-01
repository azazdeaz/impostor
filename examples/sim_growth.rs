use bevy::{
    app::ScheduleRunnerSettings,
    ecs::schedule::ShouldRun,
    prelude::*,
    render::mesh::{
        skinning::{SkinnedMesh, SkinnedMeshInverseBindposes},
        Indices, PrimitiveTopology,
    },
};
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
        .insert_resource(ScheduleRunnerSettings {
            run_mode: bevy::app::RunMode::Loop { wait: None },
        })
        .insert_resource(GrowSteps::from_steps(6))
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_plant)
        .add_startup_system(setup_scene)
        .add_system_set(
            SystemSet::new()
                .with_run_criteria(run_if_growing)
                .with_system(count_grow_steps)
                // .with_system(grow)
                // .with_system(extend),
        )
        .run();
}

struct GrowSteps {
    current: u32,
    max: u32,
}
impl GrowSteps {
    fn from_steps(max: u32) -> Self {
        Self {
            current: 0,
            max: max,
        }
    }
    fn step(&mut self) {
        self.current += 1;
    }
    fn is_done(&self) -> bool {
        self.max <= self.current
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

#[derive(Component, Debug, Clone)]
struct Stem {
    order: u32,
    length: f32,
    max_length: f32,
    radius: f32,
    direction: Quat,
}

impl Default for Stem {
    fn default() -> Self {
        Self {
            order: 0,
            length: 1.0,
            max_length: 4.0,
            radius: 1.0,
            direction: Quat::default(),
        }
    }
}

fn grow(mut query: Query<(&Parent, &mut Stem)>, parent_stem: Query<&Stem>) {
    for (parent, mut stem) in query.iter_mut() {
        stem.length += (stem.max_length - stem.length) / 3.0;
        let parent = parent_stem.get(parent.0);
        let prev_radius = if let Ok(parent) = parent {
            parent.radius
        } else {
            1.0
        };
        stem.radius += (prev_radius - stem.radius) / 12.0;
    }
}

fn extend(
    mut commands: Commands,
    mut query: Query<(Entity, &mut Stem, &Children)>,
    child_stem: Query<&Stem>,
) {
    for (stem_entity, mut stem, children) in query.iter_mut() {
        let has_following = children.iter().any(|c| child_stem.get(*c).is_ok());
        if !has_following {}
        if stem.length > stem.max_length * 0.8 && !has_following {
            let next = commands
                .spawn()
                .insert(Stem {
                    order: stem.order,
                    length: 0.2,
                    max_length: 4.0,
                    radius: 0.001,
                    direction: Quat::default(),
                })
                .id();
            commands.entity(stem_entity).push_children(&[next]);
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
    commands.spawn().insert(Stem {
        order: 0,
        length: 0.2,
        max_length: 4.0,
        radius: 0.001,
        direction: Quat::default(),
    });
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

    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_xyz(-13.0 / 1.0, 24.0 / 1.0, 17.0 / 1.0)
            .looking_at(Vec3::new(0.0, 8.0, 0.0), Vec3::Y),
        ..default()
    });
}
