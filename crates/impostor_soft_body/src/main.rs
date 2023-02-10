use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI};

use bevy::{prelude::*, utils::HashMap};
use bevy_prototype_debug_lines::*;
use bevy_rapier3d::prelude::*;
use impostor_soft_body::{Constraint, Particle, StemStructure};
use rand::{seq::IteratorRandom, thread_rng, Rng};
fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_plugin(DebugLinesPlugin::default())
        .add_plugin(impostor_editor_camera::EditorCameraPlugin)
        .add_startup_system(setup)
        .add_system(update)
        .add_system(handle_collisions)
        .run();
}

#[derive(Component)]
struct Matreial {}

/// set up a simple 3D scene
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
        .insert(Velocity::linear(Vec3::X * 0.6))
        .insert(TransformBundle::from(Transform::from_xyz(-2.0, 2.0, 0.0)));

    /* Main stem */
    let mut stem = StemStructure {
        sides: 5,
        sections: 18,
        section_height: 0.12,
        radius: 0.1,
        particles: HashMap::new(),
        start_translation: Vec3::Y * 0.6,
        orientation: Quat::from_rotation_x(0.0),
        fix_first_ring: true,
    };
    stem.spawn(&mut commands, &mut meshes, &mut materials);

    /* Branches */
    for _ in 0..2 {
        let start = Transform::from_translation(Vec3::Y * (1.2 + rng.gen::<f32>()))
            * Transform::from_rotation(Quat::from_rotation_y(PI * 2.0 * rng.gen::<f32>()))
            * Transform::from_rotation(Quat::from_rotation_x(0.6 + 0.4 * rng.gen::<f32>()));
        let mut stem2 = StemStructure {
            sides: 5,
            sections: 12,
            section_height: 0.1,
            radius: 0.07,
            particles: HashMap::new(),
            start_translation: start.translation,
            orientation: start.rotation,
            fix_first_ring: false,
        };
        stem2.spawn(&mut commands, &mut meshes, &mut materials);
        stem2.connect_to_stem(&mut commands, &stem);
    }
}

fn handle_collisions(
    mut cloth_query: Query<
        (
            Entity,
            // &Collider,
            &mut Particle,
        ),
        With<Particle>,
    >,
    rapier_context: Res<RapierContext>,
    mut colliders_query: Query<
        (&Collider, &GlobalTransform, Option<&mut Velocity>),
        Without<Particle>,
    >,
    time: Res<Time>,
) {
    let delta_time = time.delta_seconds();
    for (entity, mut particle) in cloth_query.iter_mut() {
        let collider_offset = 0.1;
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
                    particle.position,
                    false,
                );

                let normal: Vec3 = (projected_point.point - particle.position)
                    .try_normalize()
                    .unwrap_or(Vec3::Y);
                if projected_point.is_inside {
                    Some(projected_point.point + (normal * collider_offset) + (normal * vel))
                } else if particle.position.distance_squared(projected_point.point)
                    < collider_offset * collider_offset
                {
                    Some(projected_point.point - (normal * collider_offset))
                } else {
                    None
                }
            };
            if let Some(position) = pushed_position {
                particle.position = position;
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

fn update(
    time: Res<Time>,
    mut lines: ResMut<DebugLines>,
    mut particles: Query<(&mut Particle, &mut Transform)>,
    constraints: Query<&Constraint>,
) {
    for (mut particle, mut transform) in particles.iter_mut() {
        // particle.accelerate(Vec3::Y * -1.8);
        particle.simulate(time.delta_seconds());
        // particle.restrain();
        particle.reset_forces();
        transform.translation = particle.position;
    }
    let iterations = 32;
    for _ in 0..iterations {
        for constraint in constraints.iter() {
            let particle_ab =
                particles.get_many_mut([constraint.particle_a, constraint.particle_b]);
            if let Ok([mut particle_a, mut particle_b]) = particle_ab {
                constraint.relax_weighted(&mut particle_a.0, &mut particle_b.0);
                lines.line(particle_a.0.position, particle_b.0.position, 0.0);
            }
        }
    }
}
