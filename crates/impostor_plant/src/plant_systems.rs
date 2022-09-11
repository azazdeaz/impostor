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

pub struct PlantPlugin;

impl Plugin for PlantPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(GrowSteps::from_steps(80))
            .add_plugin(InspectorPlugin::<GrowSteps>::new_insert_manually())
            .insert_resource(PlantConfig::default())
            .add_plugin(InspectorPlugin::<PlantConfig>::new_insert_manually())
            .add_plugin(WorldInspectorPlugin::new())
            // .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
            .add_plugin(RapierDebugRenderPlugin::default())
            // .add_startup_system(setup_plant)
            // .add_startup_system(setup_scene)
            .add_system_set(
                SystemSet::new()
                    .label("grow")
                    .with_run_criteria(run_if_growing)
                    .with_system(count_grow_steps)
                    .with_system(grow)
                    .with_system(branch_out)
                    .with_system(extend)
                    .with_system(update_strength)
                    .with_system(update_joint_forces), // .with_system(print_structure),
            )
            .add_stage_after(CoreStage::Update, "prune", SystemStage::single_threaded())
            .add_system_to_stage("prune", resetPlant)
            // .add_system(update_mesh)
            // .add_system(add_skeleton)
            .register_type::<Stem>()
            .register_type::<Radius>()
            .register_type::<Length>()
            .register_type::<Strength>();
    }
}

#[derive(Inspectable, Default)]
struct GrowSteps {
    current: u32,
    #[inspectable(min = 1, max = 60)]
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
    max_radius: f32,
}
impl Default for PlantConfig {
    fn default() -> Self {
        Self {
            stiffness: 2000.0,
            damping: 1500.0,
            max_node_length: 2.0,
            max_radius: 1.2,
        }
    }
}

fn resetPlant(
    mut commands: Commands,
    plant_config: Res<PlantConfig>,
    mut grow_steps: ResMut<GrowSteps>,
    stems: Query<Entity, With<Stem>>,
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

fn run_if_growing(grow_steps: Res<GrowSteps>, joints: Query<&Velocity>) -> ShouldRun {
    let still_moving = joints.iter().any(|velocity| {
        let sq_linvel = velocity.linvel.length_squared();
        let sq_angvel = velocity.angvel.length_squared();
        // println!("LIN {:?} {:?}", sq_linvel, velocity.linvel);
        // println!("ANG {:?} {:?}", sq_angvel, velocity.angvel);
        sq_linvel > 0.8 || sq_angvel > 1.0
    });
    // println!("MOVING {}", still_moving);
    if still_moving || grow_steps.is_done() {
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
struct AxeCollider {}

#[derive(Component)]
struct PrevAxe(Entity);
#[derive(Component, Debug)]
struct Branches(Vec<Entity>);

#[derive(Reflect, Default, Component, Debug)]
#[reflect(Component)]
struct Length(f32);

#[derive(Reflect, Component, Debug, Clone, Copy)]
#[reflect(Component)]
struct BranchingPos(f32);
impl Default for BranchingPos {
    fn default() -> Self {
        Self(1.0)
    }
}

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
            .map(|next| weight_above(next, &mut tree, &stems))
            .sum();
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
            // println!(
            //     "this {:?} sumlength {}, candidates {:?}",
            //     this_axe.id(),
            //     length_without_branch,
            //     branch_out_from
            // );

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
            if next_axe.is_some() && length_without_branch > min_length_between_branches {
                branch_out_from.push(entity);
            }

            if has_branch_starting {
                println!("reset branching");
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
            let angle = rng.gen::<f32>() * PI * 2.0;
            commands
                .spawn()
                .insert(Name::new("Stem Node"))
                .insert_bundle(StemBundle {
                    stem: Stem {
                        order: 1, //TODO
                        max_length: plant_config.max_node_length,
                        direction: Quat::from_euler(EulerRot::XYZ, 0.0, angle, PI / 4.0),
                    },
                    length: Length(0.2),
                    radius: Radius(0.001),
                    ..Default::default()
                })
                .insert(BranchingPos(0.5))
                .insert(AxisRoot {})
                .insert(PrevAxe(*new_root_node));
            commands
                .spawn()
                .insert(Name::new("Stem Node"))
                .insert_bundle(StemBundle {
                    stem: Stem {
                        order: 1, //TODO
                        max_length: plant_config.max_node_length,
                        direction: Quat::from_euler(EulerRot::XYZ, 0.0, angle + PI, PI / 4.0),
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

fn update_joint_forces(
    mut commands: Commands,
    mut axes: Query<(Entity, Option<&mut ImpulseJoint>, Option<&Children>)>,
    details: Query<(&Strength, &Stem, &Length, &Radius, Option<&BranchingPos>)>,
    positions: Query<(Option<&Transform>, Option<&Length>)>,
    colliders: Query<Entity, With<AxeCollider>>,
    next_axes: Query<&PrevAxe>,
    plant_config: Res<PlantConfig>,
) {
    println!(">> Update joint forces");
    for (
        (this_axe, joint, children),
        (Strength(strength), stem, Length(length), Radius(radius), branching_pos),
    ) in axes.iter_mut().filter_map(|axe| {
        details
            .get(axe.0)
            .and_then(|details| Ok((axe, details)))
            .ok()
    }) {
        let branching_pos = branching_pos.and_then(|bp| Some(bp.0)).unwrap_or(1.0);
        println!(">> Update joint in {:?}", this_axe);
        let has_next_axe = next_axes
            .iter()
            .find(|PrevAxe(prev)| *prev == this_axe)
            .is_some();

        if let Some(mut joint) = joint {
            joint
                .data
                .as_spherical_mut()
                .expect("Found non spherical joint in the plant")
                // .set_local_anchor1(Vec3::new(0.0, -length + length * branching_pos, 0.0))
                .set_local_anchor2(Vec3::new(0.0, -length, 0.0))
                .set_motor_position(
                    JointAxis::AngX,
                    0.0,
                    plant_config.stiffness * strength,
                    plant_config.damping * strength,
                )
                .set_motor_position(
                    JointAxis::AngY,
                    0.0,
                    plant_config.stiffness * strength,
                    plant_config.damping * strength,
                )
                .set_motor_position(
                    JointAxis::AngZ,
                    0.0,
                    plant_config.stiffness * strength,
                    plant_config.damping * strength,
                );
            // if let Some(children) = children {
            //     if let Some((collider, mut transform)) =
            //         colliders.iter_mut().find(|(c, _)| children.contains(c))
            //     {
            //         transform.translation.y = -length / 2.0;
            //         commands
            //         .entity(collider)
            //         // .insert(Collider::capsule_y(length / 2.0, *radius))
            //         // .insert(Transform::from_xyz(0.0, -length , 0.0))
            //         ;
            //     }
            // }

            if let Some(children) = children {
                if let Some(collider) = colliders.iter().find(|c| children.contains(c)) {
                    commands.entity(collider).despawn();
                }
            }

            commands.entity(this_axe).with_children(|children| {
                children
                    .spawn()
                    .insert(Collider::capsule_y(length / 2.0, *radius))
                    .insert(AxeCollider {})
                    .insert(CollisionGroups::new(0b0000, 0b0000))
                    .insert_bundle(TransformBundle::from_transform(Transform::from_xyz(
                        0.0,
                        -length / 2.0,
                        0.0,
                    )));
            });
        } else {
            let rapier_joint = SphericalJointBuilder::new()
                .local_anchor1(Vec3::new(0.0, 0.0, 0.0))
                .local_anchor2(Vec3::new(0.0, -length, 0.0))
                .motor_position(
                    JointAxis::AngX,
                    0.0,
                    plant_config.stiffness * strength,
                    plant_config.damping * strength,
                )
                .motor_position(
                    JointAxis::AngY,
                    0.0,
                    plant_config.stiffness * strength,
                    plant_config.damping * strength,
                )
                .motor_position(
                    JointAxis::AngZ,
                    0.0,
                    plant_config.stiffness * strength,
                    plant_config.damping * strength,
                )
                .motor_model(JointAxis::AngX, MotorModel::ForceBased)
                .motor_model(JointAxis::AngY, MotorModel::ForceBased)
                .motor_model(JointAxis::AngZ, MotorModel::ForceBased);

            let parent_entity = next_axes
                .get(this_axe)
                .and_then(|PrevAxe(prev)| Ok(*prev))
                .unwrap_or_else(|_| {
                    println!(">> Creating Root for {:?}", this_axe);
                    commands
                        .spawn()
                        .insert(RigidBody::Fixed)
                        .insert_bundle(TransformBundle::default())
                        .id()
                });

            // let initial_transform = if let Ok((transform, _)) = positions.get(parent_entity) {
            //     println!(">> parent transform: {:?}", transform);

            //     transform
            //         .and_then(|transform| {
            //             Some(Transform::from_matrix(
            //                 transform.compute_matrix()
            //                     * Transform::from_rotation(stem.direction).compute_matrix()
            //                     * Transform::from_xyz(0.0, length * branching_pos, 0.0)
            //                         .compute_matrix(),
            //             ))
            //         })
            //         .unwrap_or_default()
            // } else {
            //     println!(
            //         ">> cant find paremt transform: this {:?} parent {:?}",
            //         this_axe, parent_entity
            //     );
            //     Transform::default()
            // };

            // println!(">> initial transform: {:?}", initial_transform);

            // commands
            //     .spawn()
            //     .insert(RigidBody::Fixed)
            //     .insert(Collider::ball(0.2))
            //     .insert(CollisionGroups::new(0b0000, 0b0000))
            //     .insert_bundle(TransformBundle::from_transform(initial_transform))
            //     // .insert_bundle(TransformBundle::from_transform(initial_transform))
            //     .with_children(|children| {
            //         children
            //             .spawn()
            //             .insert(Collider::capsule_y(length / 2.0, *radius))
            //             .insert(CollisionGroups::new(0b0000, 0b0000))
            //             .insert_bundle(TransformBundle::from_transform(Transform::from_xyz(
            //                 0.0,
            //                 -length / 2.0,
            //                 0.0,
            //             )));
            //     });
            let parent = commands
                .spawn()
                .insert(RigidBody::Fixed)
                .insert(Collider::cuboid(0.2, 0.2, 0.2))
                .insert(CollisionGroups::new(0b0000, 0b0000))
                .insert_bundle(TransformBundle::from_transform(Transform::from_rotation(
                    stem.direction,
                )))
                .id();
            commands.entity(parent_entity).add_child(parent);

            commands
                .entity(this_axe)
                .insert(Velocity::default())
                .insert(RigidBody::Dynamic)
                // .insert(Collider::ball(0.2))
                // .insert(CollisionGroups::new(0b0000, 0b0000))
                // .insert(initial_transform)
                // .insert_bundle(TransformBundle::from_transform(initial_transform))
                .insert_bundle(TransformBundle::default())
                .with_children(|children| {
                    children
                        .spawn()
                        .insert(AxeCollider {})
                        .insert(Collider::capsule_y(length / 2.0, *radius))
                        .insert(CollisionGroups::new(0b0000, 0b0000))
                        .insert_bundle(TransformBundle::from_transform(Transform::from_xyz(
                            0.0,
                            -length / 2.0,
                            0.0,
                        )));
                })
                .insert(ImpulseJoint::new(parent, rapier_joint));
        };
    }

    //     rapier_joint
    //         .set_local_anchor1(Vec3::new(0.0, 0.0, 0.0))
    //         .set_local_anchor2(Vec3::new(0.0, -length, 0.0))
    //         .set_motor_position(
    //             JointAxis::AngX,
    //             rot_x,
    //             plant_config.stiffness * strength,
    //             plant_config.damping * strength,
    //         )
    //         .set_motor_position(
    //             JointAxis::AngY,
    //             rot_y,
    //             plant_config.stiffness * strength,
    //             plant_config.damping * strength,
    //         )
    //         .set_motor_position(
    //             JointAxis::AngZ,
    //             rot_z,
    //             plant_config.stiffness * strength,
    //             plant_config.damping * strength,
    //         )
    //         .set_motor_model(JointAxis::AngX, MotorModel::ForceBased)
    //         .set_motor_model(JointAxis::AngY, MotorModel::ForceBased)
    //         .set_motor_model(JointAxis::AngZ, MotorModel::ForceBased)
    //         .set_limits(JointAxis::AngX, [rot_x, rot_x])
    //         .set_limits(JointAxis::AngY, [rot_y, rot_y])
    //         .set_limits(JointAxis::AngZ, [rot_z, rot_z]);

    // }
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
