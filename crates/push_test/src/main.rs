use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI};

use bevy::{prelude::*, utils::HashMap};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_mod_picking::Selection;
use bevy_prototype_debug_lines::*;
use bevy_rapier3d::prelude::*;
use itertools::Itertools;
use rand::{seq::IteratorRandom, thread_rng, Rng};

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

#[derive(Component)]
struct PlantBase {}

#[derive(Component, Clone, Copy)]
struct SegmentData {
    collider: Entity,
    next: Option<Entity>,
    length: f32,
    previous_rotation: Quat,
    target_rotation: Quat,
}

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
    let segment_length = 0.8;
    let segments = (0..segment_count)
        .map(|i| {
            commands
                .spawn(Name::new(format!("Segment #{}", i)))
                .insert(PbrBundle {
                    mesh: meshes.add(Mesh::from(shape::Box::new(0.1, 0.1, 0.1))),
                    material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
                    ..default()
                })
                .id()
        })
        .collect_vec();
    let colliders = (0..segment_count)
        .map(|_| {
            commands
                .spawn(Collider::ball(0.2))
                .insert(TransformBundle::IDENTITY)
                .id()
        })
        .collect_vec();
    for i in 0..segment_count {
        let segment = segments[i];
        if i == 0 {
            commands.entity(segment).insert(PlantBase {});
        }
        let segment_data = SegmentData {
            length: segment_length,
            previous_rotation: Quat::IDENTITY,
            target_rotation: Quat::IDENTITY,
            collider: colliders[i],
            next: if i < segments.len() - 1 {
                Some(segments[i + 1])
            } else {
                None
            },
        };

        commands
            .entity(segment)
            .insert(segment_data)
            // .insert(TransformBundle::IDENTITY)
            .with_children(|children| {
                let mut container = children.spawn(TransformBundle::from(Transform::from_xyz(
                    0.0,
                    segment_length,
                    0.0,
                )));
                container.insert(VisibilityBundle::default());
                container.add_child(segment_data.collider);
                if let Some(next) = segment_data.next {
                    container.add_child(next);
                }
            });
    }
    commands
        .spawn(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Box::new(0.5, 0.01, 0.5))),
            material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
            transform: Transform::from_xyz(0.0, 2.0, 0.0),
            ..default()
        })
        .insert(Name::new("Plant Base"))
        .add_child(segments[0]);

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
        .insert_bundle(bevy_mod_picking::PickableBundle::default())
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
        .insert_bundle(bevy_mod_picking::PickableBundle::default())
        .insert(bevy_transform_gizmo::GizmoTransformable)
        .insert(TransformBundle::from(Transform::from_xyz(-2.0, 2.0, 0.0)));
}

fn handle_segment_collisions(
    bases: Query<(Entity, &GlobalTransform), With<PlantBase>>,
    mut segments: Query<(&mut Transform, &mut SegmentData)>,
    mut colliders_query: Query<(&Collider, &GlobalTransform, Option<&mut Velocity>)>,
    rapier_context: Res<RapierContext>,
    time: Res<Time>,
    mut lines: ResMut<DebugLines>,
) {
    let delta_time = time.delta_seconds();

    for (base, transform) in bases.iter() {
        let mut next = segments.get_mut(base).ok();
        let mut prev_transform = transform.compute_transform();

        while let Some((mut transform, mut data)) = next {
            let ang_vel = transform.rotation * data.previous_rotation.inverse();
            data.previous_rotation = transform.rotation;
            let ang_acc = (data.target_rotation * transform.rotation.inverse());
            let friction = 0.89;
            let physics_rotation = Quat::IDENTITY.lerp(ang_vel, friction)
                * Quat::IDENTITY.lerp(ang_acc, delta_time.powi(2));
            // transform.rotate(physics_rotation);
            let collider_offset = 0.2;
            let collider_velocity_coefficient = 1.0;
            for contact_pair in rapier_context.contacts_with(data.collider) {
                let other_entity = if contact_pair.collider1() == data.collider {
                    contact_pair.collider2()
                } else {
                    contact_pair.collider1()
                };
                let Ok((other_collider, other_transform, other_velocity)) = colliders_query.get(other_entity) else {
                    error!("Couldn't find collider on entity {:?}", other_entity);
                    continue;
                };
                let Ok((_, self_collider_transform, __)) = colliders_query.get(data.collider) else {
                    error!("Couldn't find self collider on entity {:?}", other_entity);
                    continue;
                };

                let vel = other_velocity.as_ref().map_or(0.0, |v| {
                    v.linvel.length_squared()
                        * delta_time
                        * delta_time
                        * collider_velocity_coefficient
                });
                let self_collider_transform = self_collider_transform.compute_transform();
                let pushed_position = {
                    let other_transform = other_transform.compute_transform();

                    let projected_point = other_collider.project_point(
                        other_transform.translation,
                        other_transform.rotation,
                        self_collider_transform.translation,
                        false,
                    );
                    println!("projected_point.is_inside {:?}", projected_point.is_inside);

                    let normal: Vec3 = (projected_point.point
                        - self_collider_transform.translation)
                        .try_normalize()
                        .unwrap_or(Vec3::Y);
                    if projected_point.is_inside {
                        Some(projected_point.point + (normal * collider_offset) + (normal * vel))
                    } else if self_collider_transform
                        .translation
                        .distance_squared(projected_point.point)
                        < collider_offset * collider_offset
                    {
                        Some(projected_point.point - (normal * collider_offset))
                    } else {
                        None
                    }
                };
                if let Some(position) = pushed_position {
                    let from = self_collider_transform.translation - prev_transform.translation;
                    let to = position - prev_transform.translation;

                    let turn = Quat::from_rotation_arc(from.normalize(), to.normalize());
                    transform.rotate(turn);
                    // transform.translation = position.into();
                }
            }
            let old_pt = prev_transform.clone();
            prev_transform =
                prev_transform * transform.clone() * Transform::from_xyz(0.0, data.length, 0.0);
            lines.line_colored(
                prev_transform.translation,
                old_pt.translation,
                0.0,
                Color::ORANGE_RED,
            );
            // Continue the iteration from the next child
            next = data.next.and_then(|next| segments.get_mut(next).ok());
        }
    }
}

fn handle_collisions(
    mut pushables: Query<(Entity, &mut Transform, &mut Pushable, &Selection)>,
    rapier_context: Res<RapierContext>,
    mut colliders_query: Query<
        (&Collider, &GlobalTransform, Option<&mut Velocity>),
        Without<Pushable>,
    >,
    time: Res<Time>,
    mut lines: ResMut<DebugLines>,
) {
    let delta_time = time.delta_seconds();

    for (entity, mut transform, mut pushable, selection) in pushables.iter_mut() {
        println!(
            "\n\n\nentity {:?} selected={:?}",
            entity,
            selection.selected()
        );
        println!("prev={:?}", pushable.prev_position.translation);
        let lin_vel = transform.translation - pushable.prev_position.translation;
        pushable.prev_position = transform.clone();

        if (selection.selected()) {
            continue;
        }
        println!("new prev={:?}", pushable.prev_position.translation);
        let lin_acc = (pushable.home_position.translation - transform.translation);
        let friction = 0.92;
        println!("before={:?}", transform.translation);
        transform.translation =
            (transform.translation + lin_vel * friction + lin_acc * delta_time.powi(2)).into();
        println!("lin_vel={:?} lin_acc={:?}", lin_vel, lin_acc);
        println!("after={:?}", transform.translation);

        let collider_offset = 0.4;
        let collider_velocity_coefficient = 1.0;
        for contact_pair in rapier_context.contacts_with(entity) {
            let other_entity = if contact_pair.collider1() == entity {
                contact_pair.collider2()
            } else {
                contact_pair.collider1()
            };
            let Ok((other_collider, other_transform, other_velocity)) = colliders_query.get_mut(other_entity) else {
                // error!("Couldn't find collider on entity {:?}", entity);
                continue;
            };
            let vel = other_velocity.as_ref().map_or(0.0, |v| {
                v.linvel.length_squared() * delta_time * delta_time * collider_velocity_coefficient
            });
            let pushed_position = {
                let other_transform = other_transform.compute_transform();
                let projected_point = other_collider.project_point(
                    other_transform.translation,
                    other_transform.rotation,
                    transform.translation,
                    false,
                );

                let normal: Vec3 = (projected_point.point - transform.translation)
                    .try_normalize()
                    .unwrap_or(Vec3::Y);
                if projected_point.is_inside {
                    Some(projected_point.point + (normal * collider_offset) + (normal * vel))
                } else if transform
                    .translation
                    .distance_squared(projected_point.point)
                    < collider_offset * collider_offset
                {
                    Some(projected_point.point - (normal * collider_offset))
                } else {
                    None
                }
            };
            if let Some(position) = pushed_position {
                lines.line(transform.translation, position, 0.0);
                transform.translation = position.into();
            }

            // let collider_dampen_others = Some(0.05);
            // if let Some((ref mut vel, dampen_coef)) = other_velocity.zip(collider_dampen_others) {
            //     let damp = 1.0 - dampen_coef;
            //     vel.linvel *= damp;
            //     vel.angvel *= damp;
            // }
        }
        // *rapier_collider = get_collider(rendering, collider, None);
    }
}
