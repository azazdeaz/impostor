use bevy::{
    ecs::schedule::ShouldRun,
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
use std::f32::consts::PI;

pub struct PlantPlugin;

impl Plugin for PlantPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(GrowSteps::from_steps(80))
            .add_plugin(InspectorPlugin::<GrowSteps>::new_insert_manually())
            .insert_resource(PlantConfig::default())
            .add_plugin(InspectorPlugin::<PlantConfig>::new_insert_manually())
            .insert_resource(StemMeshConfig::default())
            .add_plugin(InspectorPlugin::<StemMeshConfig>::new_insert_manually())
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
                    // .with_system(update_stem_mesh)
                    .with_system(build_skeleton), // .with_system(update_joint_forces),
                                                  // .with_system(print_structure),
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
    min_length_between_branches: f32,
}
impl Default for PlantConfig {
    fn default() -> Self {
        Self {
            stiffness: 4000.0,
            damping: 2000.0,
            max_node_length: 1.2,
            max_radius: 0.2,
            min_length_between_branches: 3.0,
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
                length: Length(0.2),
                radius: Radius(0.001),
                ..Default::default()
            })
            .insert(AxisRoot {})
            .insert(Seed {});
    }
}

fn run_if_growing(grow_steps: Res<GrowSteps>, joints: Query<&Velocity>) -> ShouldRun {
    let still_moving = joints.iter().any(|velocity| {
        let sq_linvel = velocity.linvel.length_squared();
        let sq_angvel = velocity.angvel.length_squared();
        // println!("LIN {:?} {:?}", sq_linvel, velocity.linvel);
        // println!("ANG {:?} {:?}", sq_angvel, velocity.angvel);
        sq_linvel > 0.8 || sq_angvel > 0.5
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
struct Seed {}

#[derive(Component)]
struct SkeletonOf(Entity);

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
struct Direction(Quat);

#[derive(Reflect, Default, Component, Debug)]
#[reflect(Component)]
struct Strength(f32);

#[derive(Reflect, Default, Component, Debug, Clone)]
#[reflect(Component)]
struct Stem {}

#[derive(Bundle)]
struct StemBundle {
    stem: Stem,
    length: Length,
    radius: Radius,
    strength: Strength,
    direction: Direction,
}

impl Default for StemBundle {
    fn default() -> Self {
        Self {
            stem: Stem {},
            length: Length(1.0),
            radius: Radius(1.0),
            strength: Strength(0.1),
            direction: Direction::default(),
        }
    }
}

fn grow(
    mut commands: Commands,
    query: Query<(Entity, Option<&Parent>, &Radius, &Length)>,
    config: Res<PlantConfig>,
) {
    for (stem_id, parent, Radius(radius), Length(length)) in query.iter() {
        let prev_radius = parent
            .and_then(|parent| query.get(parent.get()).ok())
            .and_then(|q| Some((*q.2).0))
            .unwrap_or(config.max_radius);
        commands
            .entity(stem_id)
            .insert(Radius(radius + (prev_radius - radius) / 12.0))
            .insert(Length(length + (config.max_node_length - length) / 3.0));
    }
}

fn extend(
    mut commands: Commands,
    config: Res<PlantConfig>,
    query: Query<(Entity, &Length)>,
    prev_axes: Query<&PrevAxe, Without<AxisRoot>>,
) {
    let end_stems = query
        .iter()
        .filter(|(entity, _)| prev_axes.iter().all(|prev_axe| prev_axe.0 != *entity));

    for (id, Length(length)) in end_stems {
        if length.clone() > config.max_node_length * 0.8 {
            commands
                .spawn()
                .insert(Name::new("Stem Node"))
                .insert_bundle(StemBundle {
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
    stems: Query<(Entity, (&Length, &Radius), Option<&PrevAxe>)>,
    is_root: Query<(), With<AxisRoot>>,
) {
    fn weight_above(
        entity: Entity,
        stems: &Query<(Entity, (&Length, &Radius), Option<&PrevAxe>)>,
        is_root: &Query<(), With<AxisRoot>>,
    ) -> f32 {
        let (_, (Length(length), Radius(radius)), _) = stems.get(entity).unwrap();
        let stem_weight = radius * length;
        let children_weight: f32 = stems
            .iter()
            .filter_map(|(child, _, prev)| {
                prev.and_then(|prev| if prev.0 == entity { Some(child) } else { None })
            })
            .map(|next| weight_above(next, &stems, &is_root))
            .sum();
        let multiplier = if is_root.contains(entity) { 3.0 } else { 1.0 };
        (stem_weight + children_weight) * multiplier
    }
    for stem in stems.iter() {
        let weight = weight_above(stem.0, &stems, &is_root);
        commands.entity(stem.0).insert(Strength((weight + 0.1)));
    }
}

fn branch_out(
    mut commands: Commands,
    plant_config: Res<PlantConfig>,
    roots: Query<Entity, With<AxisRoot>>,
    lengths: Query<(Entity, &Length, Option<&PrevAxe>)>,
) {
    println!("ROOT LEN {}", roots.iter().len());
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
            // println!(
            //     "length_without_branch {} {}",
            //     length_without_branch,
            //     next_axe.is_some()
            // );
            if next_axe.is_some()
                && length_without_branch > plant_config.min_length_between_branches
            {
                // println!("add node");
                length_without_branch = 0.0;
                branch_out_from.push(entity);
            }

            if has_branch_starting {
                // println!("reset branching at {}", length_without_branch);
                length_without_branch = 0.0;
                branch_out_from.clear();
            }

            if let Some(next_axe) = next_axe {
                this_axe = next_axe;
            } else {
                break;
            }
        }
        // println!("branch_out_from {:?}", branch_out_from);
        for new_root_node in branch_out_from.iter() {
            println!("Add branch to {:?}", new_root_node);
            let mut rng = rand::thread_rng();
            let angle = rng.gen::<f32>() * PI * 2.0;
            commands
                .spawn()
                .insert(Name::new("Stem Node"))
                .insert_bundle(StemBundle {
                    length: Length(0.2),
                    radius: Radius(0.01),
                    direction: Direction(Quat::from_euler(EulerRot::XYZ, 0.0, angle, PI / 4.0)),
                    ..Default::default()
                })
                .insert(BranchingPos(0.5))
                .insert(AxisRoot {})
                .insert(PrevAxe(*new_root_node));
            commands
                .spawn()
                .insert(Name::new("Stem Node"))
                .insert_bundle(StemBundle {
                    length: Length(0.2),
                    radius: Radius(0.01),
                    direction: Direction(Quat::from_euler(
                        EulerRot::XYZ,
                        0.0,
                        angle + PI,
                        PI / 4.0,
                    )),
                    ..Default::default()
                })
                .insert(AxisRoot {})
                .insert(PrevAxe(*new_root_node));
        }
    }
}

fn build_skeleton(
    mut commands: Commands,
    seeds: Query<Entity, With<Seed>>,
    stems: Query<(
        Entity,
        (&Length, &Radius, &Direction, &Strength),
        Option<&PrevAxe>,
    )>,
    skeletons: Query<(Entity, &SkeletonOf)>,
    plant_config: Res<PlantConfig>,
) {
    fn add_section(
        commands: &mut Commands,
        seed: Entity,
        entity: Entity,
        (parent_entity, parent_transform): (Entity, Transform),
        stems: &Query<(
            Entity,
            (&Length, &Radius, &Direction, &Strength),
            Option<&PrevAxe>,
        )>,
        plant_config: &PlantConfig,
    ) {
        let (_, (Length(length), Radius(radius), Direction(direction), Strength(strength)), _) =
            stems.get(entity).unwrap();
        let transform = Transform::from_matrix(
            parent_transform.compute_matrix()
                * Transform::from_rotation(*direction).compute_matrix()
                * Transform::from_xyz(0.0, *length, 0.0).compute_matrix(),
        );

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
            .motor_model(JointAxis::AngX, MotorModel::AccelerationBased)
            .motor_model(JointAxis::AngY, MotorModel::AccelerationBased)
            .motor_model(JointAxis::AngZ, MotorModel::AccelerationBased);

        let base = commands
            .spawn()
            .insert(RigidBody::Fixed)
            .insert(Collider::cuboid(0.2, 0.01, 0.2))
            .insert(CollisionGroups::new(0b0000, 0b0000))
            .insert_bundle(TransformBundle::from_transform(Transform::from_rotation(
                *direction,
            )))
            .id();
        commands.entity(parent_entity).add_child(base);
        let section = commands
            .spawn()
            .insert(SkeletonOf(seed))
            .insert(Velocity::default())
            .insert(RigidBody::Dynamic)
            // .insert(Collider::ball(0.2))
            // .insert(CollisionGroups::new(0b0000, 0b0000))
            .insert_bundle(TransformBundle::from_transform(transform))
            // .insert_bundle(TransformBundle::default())
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
            .insert(ImpulseJoint::new(base, rapier_joint))
            .id();

        commands
            .spawn()
            .insert(SkeletonOf(seed))
            .insert(Velocity::default())
            .insert(RigidBody::Fixed)
            // .insert(Collider::ball(0.2))
            // .insert(CollisionGroups::new(0b0000, 0b0000))
            .insert_bundle(TransformBundle::from_transform(transform))
            // .insert_bundle(TransformBundle::default())
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
            });

        stems
            .iter()
            .filter_map(|(child, _, prev)| {
                prev.and_then(|prev| if prev.0 == entity { Some(child) } else { None })
            })
            .for_each(|next| {
                add_section(
                    commands,
                    seed,
                    next,
                    (section, transform),
                    &stems,
                    &plant_config,
                )
            });
    }

    for seed in seeds.iter() {
        println!("build skeleton for {:?}", seed);
        if !stems.contains(seed) {
            continue;
        }
        for (entity, SkeletonOf(skeleton_seed)) in skeletons.iter() {
            if *skeleton_seed == seed {
                commands.entity(entity).despawn_recursive();
            }
        }
        let root_transform = Transform::default();
        let root_entity = commands
            .spawn()
            .insert(RigidBody::Fixed)
            .insert_bundle(TransformBundle::from_transform(root_transform))
            .id();
        add_section(
            &mut commands,
            seed,
            seed,
            (root_entity, root_transform),
            &stems,
            &plant_config,
        );
    }
}

fn print_structure(
    roots: Query<Entity, (With<AxisRoot>, Without<PrevAxe>)>,
    stems: Query<(Entity, (&Length, &Radius), Option<&PrevAxe>)>,
) {
    let mut tree = TreeBuilder::new("tree".to_string());

    fn add_node(
        entity: Entity,
        mut tree: &mut TreeBuilder,
        stems: &Query<(Entity, (&Length, &Radius), Option<&PrevAxe>)>,
    ) {
        let (_, (Length(length), Radius(radius)), _) = stems.get(entity).unwrap();
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

//////////////////////////////////////////////////////////////////////
/// Create Mesh //////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

#[derive(Inspectable)]
struct StemMeshConfig {
    vertical_resolution: f32,
    ring_resolution: usize,
    use_wireframe: bool,
}
impl Default for StemMeshConfig {
    fn default() -> Self {
        Self {
            vertical_resolution: 0.2,
            ring_resolution: 8,
            use_wireframe: false,
        }
    }
}

fn update_stem_mesh(
    mut commands: Commands,
    roots: Query<Entity, With<AxisRoot>>,
    joints: Query<(
        Entity,
        Option<&ImpulseJoint>,
        Option<&PrevAxe>,
        &Transform,
        Option<&Radius>,
    )>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut skinned_mesh_inverse_bindposes_assets: ResMut<Assets<SkinnedMeshInverseBindposes>>,
    asset_server: Res<AssetServer>,
    config: Res<StemMeshConfig>,
) {
    use bevy_easings::*;

    // let texture_handle = asset_server.load("tomato/AG15brn1.png");
    let texture_handle = asset_server.load("debug_color_01.png");

    let StemMeshConfig {
        vertical_resolution,
        ring_resolution,
        use_wireframe,
    } = *config;

    // Go trough each stem
    for root in roots.iter() {
        // Collect all the joins in sequence
        let axis_joints =
            if let Ok((axe, Some(ImpulseJoint { parent, .. }), _, transform, radius)) =
                joints.get(root)
            {
                // Get the entity the first joint connects to
                if let Ok(ground) = joints.get(*parent) {
                    let radius = radius.unwrap_or(&Radius(0.0)).0;
                    let mut axis_joints =
                        vec![(ground.0, ground.3, radius), (axe, transform, radius)];
                    // Find the next axe and add it to the list
                    while let Some((axe, _, _, transform, radius)) =
                        joints.iter().find(|(_, _, prev_axe, _, _)| {
                            // TODO filter out roots
                            prev_axe
                                .and_then(|prev_axe| {
                                    Some(prev_axe.0 == axis_joints.last().unwrap().0)
                                })
                                .unwrap_or(false)
                        })
                    {
                        let radius = radius.unwrap_or(&Radius(0.0)).0;
                        axis_joints.push((axe, transform, radius))
                    }
                    Some(axis_joints)
                } else {
                    None
                }
            } else {
                None
            };
        if axis_joints.is_none() {
            println!("no joints {:?}", root);
            continue;
        }
        let axis_joints = axis_joints.unwrap();
        println!("joints {}", axis_joints.len());

        struct Ring {
            transform: Transform,
            radius: f32,
            joint_indices: [u16; 4],
            joint_weights: [f32; 4],
            from_start: f32,
        }

        let mut rings = vec![Ring {
            transform: axis_joints[0].1.clone(),
            radius: axis_joints[0].2,
            joint_indices: [0, 0, 0, 0],
            joint_weights: [1.0, 0.0, 0.0, 0.0],
            from_start: 0.0,
        }];

        // full distances up until each joints
        let distances = axis_joints
            .windows(2)
            .map(|w| w[0].1.translation.distance(w[1].1.translation))
            .collect_vec();
        let radii = axis_joints
            .iter()
            .map(|(_, _, radius)| radius)
            .collect_vec();

        let full_distances = distances.iter().fold(Vec::<f32>::new(), |mut acc, axe| {
            let last = acc.last().unwrap_or(&0.0);
            acc.push(*last + axe);
            return acc;
        });
        let full_length = *full_distances.last().unwrap();
        println!(
            "Distances={:?}\nFull distances={:?}\nFull{:?}",
            distances, full_distances, full_length
        );

        let mut state = 0.0;
        while state < full_length {
            let i = full_distances
                .iter()
                .position(|distance| distance > &state)
                .unwrap();
            // TODO there is probably a simpler way to use lerp
            let a = EaseValue(axis_joints[i].1.clone());
            let b = EaseValue(axis_joints[i + 1].1.clone());

            // 0..1.0 place of the ring between the two joints
            let start = if i == 0 { 0.0 } else { full_distances[i - 1] };
            let joint_p = (state - start) / distances[i];
            let w0 = 1.0 - joint_p;
            let w1 = joint_p;

            rings.push(Ring {
                transform: a.lerp(&b, &joint_p).0,
                radius: radii[i].lerp(radii[i + 1], &joint_p),
                joint_indices: [i as u16, i as u16 + 1, 0, 0],
                joint_weights: [w0, w1, 0.0, 0.0],
                from_start: state,
            });

            // Increment state
            state += vertical_resolution;
        }

        rings.push(Ring {
            transform: axis_joints.last().unwrap().1.clone(),
            radius: 0.0,
            joint_indices: [axis_joints.len() as u16 - 1, 0, 0, 0],
            joint_weights: [1.0, 0.0, 0.0, 0.0],
            from_start: full_length,
        });
        println!("Rinngs {}", rings.len());

        let vertex_count = rings.len() * (ring_resolution + 1);
        let mut positions = Vec::with_capacity(vertex_count);
        let mut normals = Vec::with_capacity(vertex_count);
        let mut uvs = Vec::with_capacity(vertex_count);
        let mut joint_indices = Vec::with_capacity(vertex_count);
        let mut joint_weights = Vec::with_capacity(vertex_count);

        let inverse_bindposes = axis_joints
            .iter()
            .map(|(_, transform, _)| transform.compute_matrix().inverse())
            .collect_vec();

        let inverse_bindposes = skinned_mesh_inverse_bindposes_assets
            .add(SkinnedMeshInverseBindposes::from(inverse_bindposes));

        for ring in rings.iter() {
            for step in 0..(ring_resolution) {
                let step = step as f32;
                let theta = (step / ring_resolution as f32) * PI * 2.0;
                let mut vertex = ring.transform.clone();
                vertex.rotate_local_y(theta);
                let normal = vertex.forward();
                let vertex = vertex.translation + normal * ring.radius;
                let mut uv_x = (step / (ring_resolution as f32 / 4.0)) % 2.0;
                if uv_x > 1.0 {
                    uv_x = 2.0 - uv_x;
                }
                let mut uv_y = (ring.from_start / 0.4) % 2.0;
                if uv_y > 1.0 {
                    uv_y = 2.0 - uv_y;
                }

                positions.push([vertex.x, vertex.y, vertex.z]);
                normals.push([normal.x, normal.y, normal.z]);
                uvs.push([uv_x, uv_y]);
                joint_indices.push(ring.joint_indices);
                joint_weights.push(ring.joint_weights);
            }
        }

        let quad_count = ring_resolution * (rings.len() - 1);
        let mut indices = Vec::<u32>::with_capacity(quad_count * 6);
        let step_up = ring_resolution as u32;

        for i in 0..(rings.len() as u32 - 1) {
            for j in 0..step_up {
                let start = i * step_up;
                let step1 = j;
                let step2 = (j + 1) % step_up;

                // lower triangle
                let a = start + step1;
                let b = start + step2 + step_up;
                let c = start + step2;
                // upper triangle
                let d = start + step1;
                let e = start + step1 + step_up;
                let f = start + step2 + step_up;

                if use_wireframe {
                    indices.extend_from_slice(&[a, b, b, c, c, a, d, e, e, f, f, d])
                } else {
                    indices.extend_from_slice(&[a, c, b, d, f, e])
                }
            }
        }

        // Create a mesh
        let mut mesh = if use_wireframe {
            Mesh::new(PrimitiveTopology::LineList)
        } else {
            Mesh::new(PrimitiveTopology::TriangleList)
        };
        // Set mesh vertex positions
        // println!("\n\npositions: {} {:?}", positions.len(), positions);
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);

        // Set mesh vertex normals
        // println!("\n\nnormals: {} {:?}", normals.len(), normals);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);

        // Set mesh vertex UVs. Although the mesh doesn't have any texture applied,
        //  UVs are still required by the render pipeline. So these UVs are zeroed out.
        // println!("\n\nuvs: {} {:?}", uvs.len(), uvs);
        mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);

        // Set mesh vertex joint indices for mesh skinning.
        // Each vertex gets 4 indices used to address the `JointTransforms` array in the vertex shader
        //  as well as `SkinnedMeshJoint` array in the `SkinnedMesh` component.
        // This means that a maximum of 4 joints can affect a single vertex.

        // println!(
        //     "\n\njoint_indices: {} {:?}",
        //     joint_indices.len(),
        //     joint_indices
        // );
        mesh.insert_attribute(Mesh::ATTRIBUTE_JOINT_INDEX, joint_indices);

        // Set mesh vertex joint weights for mesh skinning.
        // Each vertex gets 4 joint weights corresponding to the 4 joint indices assigned to it.
        // The sum of these weights should equal to 1.
        // println!(
        //     "\n\njoint_weights: {} {:?}",
        //     joint_weights.len(),
        //     joint_weights
        // );
        mesh.insert_attribute(Mesh::ATTRIBUTE_JOINT_WEIGHT, joint_weights);

        // Tell bevy to construct triangles from a list of vertex indices,
        //  where each 3 vertex indices form an triangle.

        // println!("\n\nindices: {} {:?}", indices.len(), indices);
        mesh.set_indices(Some(Indices::U32(indices)));

        let mesh = meshes.add(mesh);

        // let material_handle = materials.add(StandardMaterial {
        //     // base_color: Color::rgb(
        //     //     rand::thread_rng().gen_range(0.0..1.0),
        //     //     rand::thread_rng().gen_range(0.0..1.0),
        //     //     rand::thread_rng().gen_range(0.0..1.0),
        //     // ),
        //     base_color_texture: Some(texture_handle.clone()),
        //     alpha_mode: AlphaMode::Blend,
        //     unlit: false,
        //     // double_sided: true,
        //     metallic: 0.001,
        //     reflectance: 0.01,
        //     perceptual_roughness: 0.3,
        //     // flip_normal_map_y: true,
        //     ..default()
        // });
        let material_handle = materials.add(texture_handle.clone().into());

        // Create skinned mesh renderer. Note that its transform doesn't affect the position of the mesh.
        commands
            .entity(root)
            .insert_bundle(PbrBundle {
                mesh,
                material: material_handle,
                ..Default::default()
            })
            // .insert(Wireframe)
            .insert(SkinnedMesh {
                inverse_bindposes: inverse_bindposes.clone(),
                joints: axis_joints
                    .iter()
                    .map(|(entity, _, _)| *entity)
                    .collect_vec(),
            });
    }
}
