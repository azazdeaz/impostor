use bevy::{
    ecs::{entity, schedule::ShouldRun},
    prelude::*,
    render::mesh::{
        skinning::{SkinnedMesh, SkinnedMeshInverseBindposes},
        Indices, PrimitiveTopology,
    },
};
use bevy_inspector_egui::{Inspectable, InspectorPlugin, WorldInspectorPlugin};
use bevy_rapier3d::{
    prelude::*,
    rapier::prelude::{JointAxis, MotorModel},
};
use itertools::Itertools;
use ptree::{print_tree, TreeBuilder};
use rand::Rng;
use std::{cell::RefCell, collections::VecDeque, fmt};
use std::{f32::consts::PI, rc::Rc};

fn main() {
    App::new()
        .insert_resource(Msaa { samples: 4 })
        .add_plugins(DefaultPlugins)
        .insert_resource(GrowSteps::from_steps(40))
        .add_plugin(InspectorPlugin::<GrowSteps>::new_insert_manually())
        .insert_resource(PlantConfig::default())
        .add_plugin(InspectorPlugin::<PlantConfig>::new_insert_manually())
        .add_plugin(WorldInspectorPlugin::new())
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugRenderPlugin::default())
        // .add_startup_system(setup_plant)
        .add_startup_system(setup_scene)
        .add_system_set(
            SystemSet::new()
                .label("grow")
                .with_run_criteria(run_if_growing)
                .with_system(count_grow_steps)
                .with_system(grow)
                .with_system(branch_out)
                .with_system(extend)
                .with_system(update_strength)
                .with_system(print_structure),
        )
        .add_stage_after(CoreStage::Update, "prune", SystemStage::single_threaded())
        .add_system_to_stage("prune", resetPlant)
        .add_system(update_mesh)
        .add_system(add_skeleton)
        .register_type::<Stem>()
        .register_type::<Radius>()
        .register_type::<Length>()
        .register_type::<Strength>()
        .run();
}

#[derive(Inspectable, Default)]
struct GrowSteps {
    current: u32,
    #[inspectable(min = 1, max = 30)]
    max: u32,
    mesh_updated: bool,
    skeleton_updated: bool,
}

impl GrowSteps {
    fn from_steps(max: u32) -> Self {
        Self {
            current: 0,
            max: max,
            mesh_updated: false,
            skeleton_updated: false,
        }
    }
    fn reset(&mut self) {
        self.current = 0;
        self.mesh_updated = false;
        self.skeleton_updated = false;
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
    fn set_skeleton_updated(&mut self) {
        self.skeleton_updated = true;
    }
}

#[derive(Inspectable)]
struct PlantConfig {
    stiffness: f32,
    damping: f32,
    max_node_length: f32,
}
impl Default for PlantConfig {
    fn default() -> Self {
        Self {
            stiffness: 2000.0,
            damping: 1500.0,
            max_node_length: 2.0,
        }
    }
}

fn resetPlant(
    mut commands: Commands,
    plant_config: Res<PlantConfig>,
    mut grow_steps: ResMut<GrowSteps>,
    stems: Query<Entity, (With<Stem>)>,
) {
    if plant_config.is_changed() {
        println!("RESET PLANT");
        grow_steps.reset();
        stems
            .iter()
            .for_each(|entity| commands.entity(entity).despawn_recursive());

        // Start the new plant
        commands
            .spawn()
            .insert(Name::new("Root Node"))
            .insert_bundle(StemBundle {
                stem: Stem {
                    order: 0,
                    max_length: plant_config.max_node_length,
                    direction: Quat::default(),
                },
                length: Length(0.2),
                radius: Radius(0.001),
                ..Default::default()
            })
            .insert(AxisRoot {});
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
struct PrevAxe(Entity);
#[derive(Component, Debug)]
struct Branches(Vec<Entity>);


#[derive(Reflect, Default, Component, Debug)]
#[reflect(Component)]
struct Length(f32);
#[derive(Reflect, Default, Component, Debug)]
#[reflect(Component)]
struct Radius(f32);

#[derive(Reflect, Default, Component, Debug)]
#[reflect(Component)]
struct Strength(f32);

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

fn extend(
    mut commands: Commands,
    plant_config: Res<PlantConfig>,
    query: Query<(Entity, &Stem, &Length)>,
    prev_axes: Query<&PrevAxe, Without<AxisRoot>>,
) {
    let end_stems = query
        .iter()
        .filter(|(entity, _, _)| prev_axes.iter().all(|prev_axe| prev_axe.0 != *entity));

    for (id, stem, Length(length)) in end_stems {
        if length.clone() > stem.max_length * 0.8 {
            commands
                .spawn()
                .insert(Name::new("Stem Node"))
                .insert_bundle(StemBundle {
                    stem: Stem {
                        order: stem.order,
                        max_length: plant_config.max_node_length,
                        direction: Quat::default(),
                    },
                    length: Length(0.2),
                    radius: Radius(0.001),
                    ..Default::default()
                })
                .insert(PrevAxe(id));
        }
    }
}

fn update_strength(
    mut commands: Commands,
    roots: Query<Entity, With<Stem>>,
    stems: Query<(Entity, (&Stem, &Length, &Radius), Option<&PrevAxe>)>,
) {
    let mut tree = TreeBuilder::new("tree".to_string());

    fn weight_above(
        entity: Entity,
        mut tree: &mut TreeBuilder,
        stems: &Query<(Entity, (&Stem, &Length, &Radius), Option<&PrevAxe>)>,
    ) -> f32 {
        let (_, (_, Length(length), Radius(radius)), _) = stems.get(entity).unwrap();
        let stem_weight = radius * length;
        let children_weight: f32 = stems
            .iter()
            .filter_map(|(child, _, prev)| {
                prev.and_then(|prev| if prev.0 == entity { Some(child) } else { None })
            })
            .map(|next| weight_above(next, &mut tree, &stems)).sum();
        stem_weight + children_weight
    };
    for root in roots.iter() {
        let weight = weight_above(root, &mut tree, &stems);
        commands.entity(root).insert(Strength(weight));
    }
    print_tree(&tree.build()).ok();
}

fn branch_out(
    mut commands: Commands,
    plant_config: Res<PlantConfig>,
    roots: Query<Entity, With<AxisRoot>>,
    lengths: Query<(Entity, &Length, Option<&PrevAxe>)>,
) {
    let min_length_between_branches = 5.0; // TODO move this to config resource

    for root_axe in roots.iter() {
        let mut this_axe = root_axe;
        let mut length_without_branch = 0.0;
        let mut branch_out_from = Vec::<Entity>::new();
        while let Ok((entity, Length(length), _)) = lengths.get(this_axe) {
            println!(
                "this {:?} sumlength {}, candidates {:?}",
                this_axe.id(),
                length_without_branch,
                branch_out_from
            );

            let has_branch_starting = lengths.iter().any(|(entity, _, prev)| {
                roots.contains(entity)
                    && prev
                        .and_then(|prev| Some(prev.0 == this_axe))
                        .unwrap_or(false)
            });
            let next_axe = lengths.iter().find_map(|(entity, _, prev)| {
                prev.and_then(|prev| {
                    if prev.0 == this_axe {
                        Some(entity)
                    } else {
                        None
                    }
                })
            });

            length_without_branch += length;
            if length_without_branch > min_length_between_branches {
                branch_out_from.push(entity);
            }

            if has_branch_starting {
                println!("reset branchinf");
                length_without_branch = 0.0;
                branch_out_from.clear();
            }

            if let Some(next_axe) = next_axe {
                this_axe = next_axe;
            } else {
                break;
            }
        }

        for new_root_node in branch_out_from.iter() {
            println!("Add branch to {:?}", new_root_node);
            let mut rng = rand::thread_rng();
            commands
                .spawn()
                .insert(Name::new("Stem Node"))
                .insert_bundle(StemBundle {
                    stem: Stem {
                        order: 1, //TODO
                        max_length: plant_config.max_node_length,
                        direction: Quat::from_euler(
                            EulerRot::XZY,
                            0.0,
                            PI / 2.0,
                            rng.gen::<f32>() * PI * 2.0,
                        ),
                    },
                    length: Length(0.2),
                    radius: Radius(0.001),
                    ..Default::default()
                })
                .insert(AxisRoot {})
                .insert(PrevAxe(*new_root_node));
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

fn add_skeleton(
    mut commands: Commands,
    mut grow_steps: ResMut<GrowSteps>,
    q_root: Query<Entity, With<AxisRoot>>,
    q_stems: Query<(Entity, (&Stem, &Length, &Radius, &Strength), Option<&PrevAxe>)>,
    plant_config: Res<PlantConfig>,
) {
    if !grow_steps.is_done() || grow_steps.skeleton_updated {
        return;
    }
    grow_steps.set_skeleton_updated();

    for root in q_root.iter() {
        let prev_axe = q_stems.get(root).unwrap().2;
        let base = if let Some(prev_axe) = prev_axe {
            prev_axe.0
        } else {
            commands
                .spawn()
                .insert(RigidBody::Fixed)
                .insert_bundle(TransformBundle::from_transform(Transform::from_xyz(0.0, 0.0, 0.0)))
                .id()
        };

        let mut next_step = Some((base, root));
        while let Some((prev_axe, this_axe)) = next_step {
            if let Ok((_, (stem, Length(length), Radius(radius), Strength(strength)), _)) = q_stems.get(this_axe) {
                next_step = q_stems
                    .iter()
                    .find(|(_, _, prev_axe)| {
                        prev_axe
                            .and_then(|axe| Some(axe.0 == this_axe))
                            .unwrap_or(false)
                    })
                    .and_then(|stem| Some((this_axe, stem.0)));

                let transform = Transform::from_xyz(0.0, *length, 0.0);
                let (rot_y, rot_z, rot_x) = stem.direction.to_euler(EulerRot::YZX);
                println!("rot_x {} rot_y {} rot_z {}", rot_x, rot_y, rot_z);

                // next_step =
                // next_step = children
                //     .and_then(|children| children.iter().find(|c| q_stems.get(**c).is_ok()))
                //     .and_then(|next| Some((node, *next)));

                let rapier_joint: GenericJoint = if next_step.is_some() {
                    SphericalJointBuilder::new()
                        .local_anchor1(Vec3::new(0.0, 0.0, 0.0))
                        .local_anchor2(Vec3::new(0.0, -length, 0.0))
                        .motor_position(
                            JointAxis::AngX,
                            rot_x,
                            plant_config.stiffness * strength,
                            plant_config.damping * strength,
                        )
                        .motor_position(
                            JointAxis::AngY,
                            rot_y,
                            plant_config.stiffness * strength * 10.0,
                            plant_config.damping * strength * 10.0,
                        )
                        .motor_position(
                            JointAxis::AngZ,
                            rot_z,
                            plant_config.stiffness * strength,
                            plant_config.damping * strength,
                        )
                        .motor_model(JointAxis::AngX, MotorModel::AccelerationBased)
                        .motor_model(JointAxis::AngY, MotorModel::AccelerationBased)
                        .motor_model(JointAxis::AngZ, MotorModel::AccelerationBased)
                        .limits(JointAxis::AngX, [-0.0, 0.0])
                        .limits(JointAxis::AngY, [-0.0, 0.0])
                        .limits(JointAxis::AngZ, [-0.0, 0.0])
                        .into()
                } else {
                    FixedJointBuilder::new()
                        .local_anchor1(Vec3::new(0.0, 0.0, 0.0))
                        .local_anchor2(Vec3::new(0.0, -length, 0.0))
                        .into()
                };

                commands
                    .entity(this_axe)
                    .insert(RigidBody::Dynamic)
                    .insert_bundle(TransformBundle::from_transform(transform))
                    .insert(GlobalTransform::identity())
                    .with_children(|children| {
                        children
                            .spawn()
                            .insert(Collider::capsule_y(length / 2.0, *radius* 0.2))
                            .insert(CollisionGroups::new(0b1000, 0b0100))
                            .insert(Transform::from_xyz(0.0, -length / 2.0, 0.0));
                    })
                    .insert(ImpulseJoint::new(prev_axe, rapier_joint));
            }
        }
    }
}

fn update_mesh(
    mut commands: Commands,
    mut grow_steps: ResMut<GrowSteps>,
    q_root: Query<Entity, With<AxisRoot>>,
    q_stems: Query<(Entity, (&Stem, &Length, &Radius), Option<&PrevAxe>)>,
) {
    if !grow_steps.is_done() || grow_steps.mesh_updated {
        return;
    }
    grow_steps.set_mesh_updated();

    commands
        .spawn()
        .insert(RigidBody::Dynamic)
        .insert(Collider::ball(1.0))
        .insert(Restitution::coefficient(0.7))
        .insert(Transform::from_xyz(-0.01, (16.0 as f32) * 4.0 + 4.0, -0.01));

    for root in q_root.iter() {
        let mut prev = Some(root);
        while let Some(stem_id) = prev {
            prev = if let Ok((entity, info, _)) = q_stems.get(stem_id) {
                println!(
                    "STEM {:?} length: {:?} radius: {:?}",
                    info.0, info.1, info.2
                );
                q_stems
                    .iter()
                    .find(|(_, _, prev_axe)| {
                        prev_axe
                            .and_then(|prev_axe| Some(prev_axe.0 == entity))
                            .unwrap_or(false)
                    })
                    .and_then(|(entity, _, _)| Some(entity))
            } else {
                None
            }
        }
    }
}

fn print_structure(
    roots: Query<Entity, (With<AxisRoot>, Without<PrevAxe>)>,
    stems: Query<(Entity, (&Stem, &Length, &Radius), Option<&PrevAxe>)>,
) {
    let mut tree = TreeBuilder::new("tree".to_string());

    fn add_node(
        entity: Entity,
        mut tree: &mut TreeBuilder,
        stems: &Query<(Entity, (&Stem, &Length, &Radius), Option<&PrevAxe>)>,
    ) {
        let (_, (_, Length(length), Radius(radius)), _) = stems.get(entity).unwrap();
        tree.begin_child(format!("{:?} len:{:.2} r:{:.2}", entity, length, radius));
        stems
            .iter()
            .filter_map(|(child, _, prev)| {
                prev.and_then(|prev| if prev.0 == entity { Some(child) } else { None })
            })
            .for_each(|next| add_node(next, &mut tree, &stems));
        tree.end_child();
    };
    for root in roots.iter() {
        add_node(root, &mut tree, &stems);
    }
    print_tree(&tree.build()).ok();
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

    // let root_joint = commands
    //     .spawn()
    //     .insert(RigidBody::Fixed)
    //     .insert(GlobalTransform::identity())
    //     .insert(Transform::from_xyz(0.0, 0.0, 0.0))
    //     .id();
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
        .insert(Transform::from_xyz(
            -10.01,
            (16.0 as f32) * 4.0 + 4.0,
            -10.01,
        ));

    commands.spawn_bundle(Camera3dBundle {
        transform: Transform::from_xyz(-13.0 / 1.0, 24.0 / 1.0, 17.0 / 1.0)
            .looking_at(Vec3::new(0.0, 8.0, 0.0), Vec3::Y),
        ..default()
    });
}
