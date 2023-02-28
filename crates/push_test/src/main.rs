use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI};

use bevy::{prelude::*, utils::HashMap};
use bevy_mod_picking::Selection;
use bevy_prototype_debug_lines::*;
use bevy_rapier3d::prelude::*;
use rand::{seq::IteratorRandom, thread_rng, Rng};
fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_plugin(DebugLinesPlugin::default())
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
struct TargetLength(f32);

#[derive(Component)]
struct TargetRotation(Quat);

#[derive(Component)]
struct PlantBase {}

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

    let mut parent = commands
        .spawn(PlantBase {})
        .insert(TransformBundle::IDENTITY)
        .id();
    for _ in 0..4 {
        let transform = Transform::from_xyz(0.0, 1.0, 0.0);
        let segment = commands
            .spawn(PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Icosphere {
                    radius: 0.2,
                    subdivisions: 3,
                })),
                material: materials.add(Color::rgb(0.5, 0.3, 0.3).into()),
                ..default()
            })
            .with_children(|children| {
                children
                    .spawn(Collider::ball(0.2))
                    .insert(TransformBundle::IDENTITY);
            })
            .insert(TargetLength(1.0))
            .insert(TargetRotation(Quat::IDENTITY))
            .insert(TransformBundle::from(transform))
            .id();

        commands.entity(parent).add_child(segment);
        parent = segment;
    }

    let transform = Transform::from_xyz(-2.0, 1.0, 0.0);
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
        .insert(Pushable {
            home_position: transform * Transform::from_xyz(0.0, 10.0, 0.0),
            prev_position: transform,
        })
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
    bases: Query<(Entity, &GlobalTransform, &Children), With<PlantBase>>,
    mut segments: Query<(&mut Transform, &Children), With<TargetLength>>,
    mut colliders_query: Query<(&Collider, &GlobalTransform, Option<&mut Velocity>)>,
    rapier_context: Res<RapierContext>,
    time: Res<Time>,
    mut lines: ResMut<DebugLines>,
) {
    let delta_time = time.delta_seconds();

    for (base, transform, children) in bases.iter() {
        let mut next = children.iter().find(|&&child| segments.contains(child));
        let mut prev_transform = transform.compute_transform();

        while let Some(&segment) = next {
            let (transform, children) = segments.get(segment).expect(&format!(
                "Can't find segment({:?}) in the segments query",
                segment
            ));
            let combined_transform = (*transform * prev_transform);

            let collider_offset = 0.2;
            let collider_velocity_coefficient = 1.0;
            let contact_child = *children
                .first()
                .expect("Segments have to have one child with a Collider component");
            for contact_pair in rapier_context.contacts_with(contact_child) {
                let other_entity = if contact_pair.collider1() == contact_child {
                    contact_pair.collider2()
                } else {
                    contact_pair.collider1()
                };
                let Ok((other_collider, other_transform, other_velocity)) = colliders_query.get_mut(other_entity) else {
                    error!("Couldn't find collider on entity {:?}", other_entity);
                    continue;
                };
                println!("Got contact {:?}<>{:?}", segment, other_entity);
                let vel = other_velocity.as_ref().map_or(0.0, |v| {
                    v.linvel.length_squared()
                        * delta_time
                        * delta_time
                        * collider_velocity_coefficient
                });
                let pushed_position = {
                    let other_transform = other_transform.compute_transform();

                    let projected_point = other_collider.project_point(
                        other_transform.translation,
                        other_transform.rotation,
                        combined_transform.translation,
                        false,
                    );
                    lines.line_colored(
                        prev_transform.translation,
                        combined_transform.translation,
                        0.0,
                        Color::PINK,
                    );
                    lines.line_colored(
                        combined_transform.translation,
                        projected_point.point,
                        0.0,
                        Color::LIME_GREEN,
                    );

                    println!("projected_point.is_inside {:?}", projected_point.is_inside);

                    let normal: Vec3 = (projected_point.point - combined_transform.translation)
                        .try_normalize()
                        .unwrap_or(Vec3::Y);
                    if projected_point.is_inside {
                        Some(projected_point.point + (normal * collider_offset) + (normal * vel))
                    } else if combined_transform
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
                    // lines.line(combined_transform.translation, position, 0.0);
                    lines.line(prev_transform.translation, position, 0.0);
                    // transform.translation = position.into();
                }
            }
            // Continue the iteration from the next child
            next = children.iter().find(|&&child| segments.contains(child));
            prev_transform = combined_transform;
            println!("Next is {:?}", next);
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
