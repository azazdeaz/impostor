use bevy::{prelude::*, utils::HashMap};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_mod_picking::Selection;
use bevy_prototype_debug_lines::*;
use bevy_rapier3d::prelude::*;
use itertools::Itertools;
use push_test::*;
use rand::thread_rng;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_plugin(DebugLinesPlugin::default())
        .add_plugin(WorldInspectorPlugin)
        .add_plugin(impostor_editor_camera::EditorCameraPlugin)
        .add_plugins(bevy_mod_picking::DefaultPickingPlugins)
        .add_plugin(bevy_transform_gizmo::TransformGizmoPlugin::default())
        .add_startup_system(setup)
        // .add_system(handle_collisions)
        .add_system(handle_segment_collisions)
        .run();
}

#[derive(Component)]
struct Pushable {
    home_position: Transform,
    prev_position: Transform,
}

#[derive(Component)]
struct TargetRotation(Quat);

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mut rng = thread_rng();
    // plane
    commands
        .spawn(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Box::new(10.0, 1.2, 10.0))),
            material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
            ..default()
        })
        .insert(RigidBody::Fixed)
        .insert(Collider::cuboid(5.0, 0.6, 5.0));

    // light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });
    // camera
    // commands.spawn(Camera3dBundle {
    //     transform: Transform::from_xyz(-2.0, 2.5, 5.0).looking_at(Vec3::Y * 2.0, Vec3::Y * 6.0),
    //     ..default()
    // });

    let segment_count = 5;
    let segment_length = 0.4;
    let segments = (0..segment_count)
        .map(|i| commands.spawn(Name::new(format!("Segment #{}", i))).id())
        .collect_vec();
    let colliders = (0..segment_count)
        .map(|_| {
            commands
                .spawn(Collider::ball(0.2))
                .insert(TransformBundle::IDENTITY)
                .insert(bevy_mod_picking::PickableBundle::default())
                .insert(bevy_transform_gizmo::GizmoTransformable)
                .insert(PbrBundle {
                    mesh: meshes.add(Mesh::from(shape::Box::new(0.1, 0.1, 0.1))),
                    material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
                    ..default()
                })
                .id()
        })
        .collect_vec();

    let base_translation = Vec3::new(0.0, 2.0, 0.0);
    let mut accumulated_transform = Transform::from_translation(base_translation);
    let mut segment_data_map = HashMap::new();
    for i in 0..segment_count {
        let segment = segments[i];
        if i == 0 {
            commands.entity(segment).insert(PlantBase {
                translation: base_translation,
            });
        }
        let rotation = Quat::from_scaled_axis(Vec3::new(3.0, 0.0, -1.0).normalize() * 0.2);
        let segment_data = SegmentData {
            length: segment_length,
            target_rotation: rotation,
            previous_rotation: rotation,
            collider: colliders[i],
            forward: if i < segments.len() - 1 {
                Some(segments[i + 1])
            } else {
                None
            },
            backward: if i > 0 { Some(segments[i - 1]) } else { None },
        };
        segment_data_map.insert(segment, segment_data);

        if let Some(backward) = segment_data.backward {
            let length = segment_data_map.get(&backward).unwrap().length;
            accumulated_transform = accumulated_transform * Transform::from_xyz(0.0, length, 0.0);
        }
        accumulated_transform =
            accumulated_transform * Transform::from_rotation(segment_data.target_rotation);

        commands
            .entity(segment)
            .insert(Name::new(format!("Segment {}", i)))
            .insert(segment_data);
        commands
            .entity(segment_data.collider)
            .insert(Name::new(format!("Segment Collider {}", i)))
            .insert(TransformBundle::from(accumulated_transform));
    }
    commands
        .spawn(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Box::new(0.5, 0.01, 0.5))),
            material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
            transform: Transform::from_translation(base_translation),
            ..default()
        })
        .insert(Name::new("Plant Base Disk"));

    let transform = Transform::from_xyz(-0.5, 1.0, 0.0);
    commands
        // .spawn(RigidBody::KinematicPositionBased)
        // .insert(Collider::ball(0.3))
        .spawn(Collider::ball(0.4))
        .insert(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Icosphere {
                radius: 0.4,
                subdivisions: 4,
            })),
            material: materials.add(Color::rgb(0.5, 0.3, 0.3).into()),
            ..default()
        })
        .insert(Restitution::coefficient(0.7))
        .insert(bevy_mod_picking::PickableBundle::default())
        .insert(bevy_transform_gizmo::GizmoTransformable)
        .insert(TransformBundle::from(transform));

    /* Create the bouncing ball. */
    commands
        .spawn(RigidBody::KinematicVelocityBased)
        // .insert(Collider::ball(0.3))
        .insert(Collider::cuboid(0.1, 0.1, 2.0))
        .insert(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Box::new(0.2, 0.2, 4.0))),
            material: materials.add(Color::rgb(0.5, 0.3, 0.3).into()),
            ..default()
        })
        .insert(Restitution::coefficient(0.7))
        // .insert(Velocity::linear(Vec3::X * 0.6))
        .insert(bevy_mod_picking::PickableBundle::default())
        .insert(bevy_transform_gizmo::GizmoTransformable)
        .insert(TransformBundle::from(Transform::from_xyz(-2.0, 2.0, 0.0)));
}

fn handle_segment_collisions(
    bases: Query<(Entity, &PlantBase)>,
    mut segments_query: Query<(&mut SegmentData)>,
    mut colliders_query: Query<(&Collider, &mut Transform, Option<&mut Velocity>, &Selection)>,
    rapier_context: Res<RapierContext>,
    time: Res<Time>,
    mut lines: ResMut<DebugLines>,
) {
    let delta_time = time.delta_seconds();

    for (base_id, base) in bases.iter() {
        let mut global_transforms = HashMap::<Entity, Transform>::new();
        let mut segments = HashMap::<Entity, SegmentData>::new();

        let mut explore = vec![base_id];
        while !explore.is_empty() {
            let segment_id = explore.pop().unwrap();
            let mut segment = segments_query.get_mut(segment_id).expect(&format!(
                "Can't find segment entity '{:?}' in segments query.",
                segment_id
            ));
            let &transform: &Transform = colliders_query.get_component(segment.collider).unwrap();
            let current_rotation = transform.rotation;
            let rotation_difference = current_rotation * segment.previous_rotation.inverse();
            // segment.previous_rotation = current_rotation;

            if let Some(forward) = segment.forward {
                explore.push(forward)
            }
        }

        // Selected with the transform gizmo (mimic grasping)
        let mut selected_segment = None;
        let mut end_segment = base_id;
        // Buffer positions and segment data
        let mut explore = vec![base_id];
        while !explore.is_empty() {
            let segment_id = explore.pop().unwrap();
            let &segment = segments_query.get(segment_id).expect(&format!(
                "Can't find segment entity '{:?}' in segments query.",
                segment_id
            ));

            end_segment = segment_id;
            segments.insert(segment_id, segment);

            if segment_id == base_id {
                global_transforms.insert(
                    base_id,
                    Transform::from_translation(base.translation)
                        * Transform::from_rotation(segment.previous_rotation),
                );
            } else {
                let backward = segment
                    .backward
                    .expect("Missig backward on non-base section");
                let &parent_transform = global_transforms.get(&backward).expect(
                    "Incorrect structure! Couldn't find the backward entity in the `global_transforms` buffer",
                );
                let &parent_segment = segments.get(&backward).expect(
                    "Incorrect structure! Couldn't find the backward entity in the `segments` buffer",
                );
                let global_transform = Transform::from_matrix(
                    Transform::from_rotation(segment.previous_rotation).compute_matrix()
                        * Transform::from_xyz(0.0, parent_segment.length, 0.0).compute_matrix()
                        * parent_transform.compute_matrix(),
                );

                // parent_transform.tra

                global_transforms.insert(segment_id, global_transform);
                // lines.line_colored(
                //     global_transform.translation,
                //     parent_transform.translation,
                //     0.0,
                //     Color::GRAY,
                // );
            }

            let selection: &Selection = colliders_query
                .get_component(segment.collider)
                .expect("Cant find segment.collider");
            if selection.selected() {
                selected_segment = Some(segment_id);
                let &transform: &Transform =
                    colliders_query.get_component(segment.collider).unwrap();
                global_transforms.insert(segment_id, transform);
            }

            if let Some(forward) = segment.forward {
                explore.push(forward)
            }
        }

        let mut fabrik_computer = FabrikComputer {
            global_xyz: global_transforms
                .iter()
                .map(|(k, v)| (*k, v.translation))
                .collect(),
            segments,
        };

        // Solve IK up until the grasped segment
        if let Some(seleted_segment) = selected_segment {
            for _ in 0..12 {
                fabrik_computer.iterate(base_id, seleted_segment);
                fabrik_computer.iterate_half(
                    seleted_segment,
                    end_segment,
                    FabrikDirection::Forward,
                );
            }
        }

        // Iterate the tree from bottom-up
        let mut explore = vec![base_id];
        // The latest segment that was updated by the IK solver (because it was pushed)
        // Next IK optimizations should only reach back until this segment.
        let mut last_fixed_segment = base_id;
        while false && !explore.is_empty() {
            let segment_id = explore.pop().unwrap();
            let segment_data = fabrik_computer.segments[&segment_id];

            let collider_offset = 0.2;
            let collider_velocity_coefficient = 1.0;
            let mut segment_was_pushed = false;
            for contact_pair in rapier_context.contacts_with(segment_data.collider) {
                let other_entity = if contact_pair.collider1() == segment_data.collider {
                    contact_pair.collider2()
                } else {
                    contact_pair.collider1()
                };
                let Ok((other_collider, other_transform, other_velocity, _)) = colliders_query.get(other_entity) else {
                    error!("Couldn't find collider on entity {:?}", other_entity);
                    continue;
                };

                let vel = other_velocity.as_ref().map_or(0.0, |v| {
                    v.linvel.length_squared()
                        * delta_time
                        * delta_time
                        * collider_velocity_coefficient
                });
                let pushed_position = {
                    let original_xyz = fabrik_computer.global_xyz[&segment_id];
                    let projected_point = other_collider.project_point(
                        other_transform.translation,
                        other_transform.rotation,
                        original_xyz,
                        false,
                    );

                    let normal: Vec3 = (projected_point.point - original_xyz)
                        .try_normalize()
                        .unwrap_or(Vec3::Y);
                    if projected_point.is_inside {
                        Some(projected_point.point + (normal * collider_offset) + (normal * vel))
                    } else if original_xyz.distance_squared(projected_point.point)
                        < collider_offset * collider_offset
                    {
                        Some(projected_point.point - (normal * collider_offset))
                    } else {
                        None
                    }
                };
                if let Some(position) = pushed_position {
                    segment_was_pushed = true;
                    for _ in 0..3 {
                        fabrik_computer.global_xyz.insert(segment_id, position);
                        fabrik_computer.iterate(last_fixed_segment, segment_id);
                    }
                }
            }

            if segment_was_pushed {
                last_fixed_segment = segment_id;
            }

            if let Some(forward) = segment_data.forward {
                explore.push(forward)
            }
        }

        // Update transforms (rotations)
        let mut explore = vec![base_id];
        while !explore.is_empty() {
            let segment_id = explore.pop().unwrap();
            let mut segment_data = fabrik_computer.segments[&segment_id];

            let (_, mut this_transform, _, selection) =
                colliders_query.get_mut(segment_data.collider).unwrap();
            let this_position = fabrik_computer.global_xyz[&segment_id];

            if let Some(forward) = segment_data.forward {
                explore.push(forward);

                // Convert translation diff between segments to rotation

                // Find the global global of the next segment
                let next_position = fabrik_computer.global_xyz[&forward];
                // Get the current direction
                let current_direction = this_transform.rotation * Vec3::Y;
                // The requested direction
                let target_direction = (next_position - this_position).normalize();
                // The rotation difference between the two directions
                let rotation = Quat::from_rotation_arc(current_direction, target_direction);
                lines.line_colored(
                    this_position,
                    this_position + current_direction * 0.2,
                    0.0,
                    Color::PINK,
                );
                lines.line_colored(
                    this_position,
                    this_position + segment_data.previous_rotation * Vec3::Y * 0.2,
                    0.0,
                    Color::LIME_GREEN,
                );
                // Avoid colliding updates with the transform gizmo
                if !selection.selected() {
                    *this_transform = Transform::from_translation(this_position)
                        * Transform::from_rotation(rotation);
                }

                // lines.line_colored(
                //     fabrik_computer.global_xyz[&forward],
                //     fabrik_computer.global_xyz[&segment_id],
                //     0.0,
                //     Color::ORANGE_RED,
                // );
                // The closing segment (forward is None)
            } else {
                if !selection.selected() {
                    *this_transform = Transform::from_translation(this_position);
                }
            }
        }
    }
}

// fn handle_collisions(
//     mut pushables: Query<(Entity, &mut Transform, &mut Pushable, &Selection)>,
//     rapier_context: Res<RapierContext>,
//     mut colliders_query: Query<
//         (&Collider, &GlobalTransform, Option<&mut Velocity>),
//         Without<Pushable>,
//     >,
//     time: Res<Time>,
//     mut lines: ResMut<DebugLines>,
// ) {
//     let delta_time = time.delta_seconds();

//     for (entity, mut transform, mut pushable, selection) in pushables.iter_mut() {
//         println!(
//             "\n\n\nentity {:?} selected={:?}",
//             entity,
//             selection.selected()
//         );
//         println!("prev={:?}", pushable.prev_position.translation);
//         let lin_vel = transform.translation - pushable.prev_position.translation;
//         pushable.prev_position = transform.clone();

//         if (selection.selected()) {
//             continue;
//         }
//         println!("new prev={:?}", pushable.prev_position.translation);
//         let lin_acc = (pushable.home_position.translation - transform.translation);
//         let friction = 0.92;
//         println!("before={:?}", transform.translation);
//         transform.translation =
//             (transform.translation + lin_vel * friction + lin_acc * delta_time.powi(2)).into();
//         println!("lin_vel={:?} lin_acc={:?}", lin_vel, lin_acc);
//         println!("after={:?}", transform.translation);

//         let collider_offset = 0.4;
//         let collider_velocity_coefficient = 1.0;
//         for contact_pair in rapier_context.contacts_with(entity) {
//             let other_entity = if contact_pair.collider1() == entity {
//                 contact_pair.collider2()
//             } else {
//                 contact_pair.collider1()
//             };
//             let Ok((other_collider, other_transform, other_velocity)) = colliders_query.get_mut(other_entity) else {
//                 // error!("Couldn't find collider on entity {:?}", entity);
//                 continue;
//             };
//             let vel = other_velocity.as_ref().map_or(0.0, |v| {
//                 v.linvel.length_squared() * delta_time * delta_time * collider_velocity_coefficient
//             });
//             let pushed_position = {
//                 let other_transform = other_transform.compute_transform();
//                 let projected_point = other_collider.project_point(
//                     other_transform.translation,
//                     other_transform.rotation,
//                     transform.translation,
//                     false,
//                 );

//                 let normal: Vec3 = (projected_point.point - transform.translation)
//                     .try_normalize()
//                     .unwrap_or(Vec3::Y);
//                 if projected_point.is_inside {
//                     Some(projected_point.point + (normal * collider_offset) + (normal * vel))
//                 } else if transform
//                     .translation
//                     .distance_squared(projected_point.point)
//                     < collider_offset * collider_offset
//                 {
//                     Some(projected_point.point - (normal * collider_offset))
//                 } else {
//                     None
//                 }
//             };
//             if let Some(position) = pushed_position {
//                 lines.line(transform.translation, position, 0.0);
//                 transform.translation = position.into();
//             }

//             // let collider_dampen_others = Some(0.05);
//             // if let Some((ref mut vel, dampen_coef)) = other_velocity.zip(collider_dampen_others) {
//             //     let damp = 1.0 - dampen_coef;
//             //     vel.linvel *= damp;
//             //     vel.angvel *= damp;
//             // }
//         }
//         // *rapier_collider = get_collider(rendering, collider, None);
//     }
// }
