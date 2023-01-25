use std::ops::Mul;

use bevy::prelude::*;
use itertools_num::linspace;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_startup_system(setup)
        .add_system(update)
        .run();

}
#[derive(Component)]
struct Particle {
    previous_position: Vec3,
    position: Vec3,
    velocity: Vec3,
    acceleration: Vec3
}

#[derive(Component)]
struct Matreial {

}

/// set up a simple 3D scene
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // plane
    commands.spawn(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Plane { size: 5.0 })),
        material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
        ..default()
    });
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
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-2.0, 2.5, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    let start_y = 1.0;
    let half = 0.15;
    for x in linspace::<f32>(-half, half, 3) {
        for y in linspace::<f32>(-half, half, 3) {
            for z in linspace::<f32>(-half, half, 3) {
                let transform = Transform::from_xyz(x, start_y + y, z);
                commands.spawn(PbrBundle {
                    mesh: meshes.add(Mesh::from(shape::Icosphere { radius: 0.05, subdivisions: 3 })),
                    material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
                    transform,
                    ..default()
                }).insert(Particle {
                    previous_position: transform.translation,
                    position: transform.translation,
                    velocity: Vec3::ZERO,
                    acceleration: Vec3::Y * -0.01,
                });
            }
        }
    }
}

fn update(mut particles: Query<(&mut Particle, &mut Transform)>) {
    for (particle, mut transform) in particles.iter_mut() {
        transform.translation += particle.acceleration;
    }
}