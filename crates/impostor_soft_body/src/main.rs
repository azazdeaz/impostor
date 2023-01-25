use std::ops::Mul;

use bevy::{prelude::*, utils::HashMap};
use bevy_prototype_debug_lines::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(DebugLinesPlugin::default())
        .add_startup_system(setup)
        .add_system(update)
        .run();
}
#[derive(Component)]
struct Particle {
    previous_position: Vec3,
    position: Vec3,
    velocity: Vec3,
    acceleration: Vec3,
    mass: f32,
}

impl Particle {
    fn accelerate(&mut self, rate: Vec3) {
        self.acceleration += rate;
    }
    fn simulate(&mut self, delta: f32) {
        if self.mass == 0.0 {
            return;
        }
        self.velocity = 2.0 * self.position - self.previous_position;
        self.previous_position = self.position;
        self.position = self.velocity + self.acceleration * delta * delta;
        self.velocity = self.position - self.previous_position;
        self.acceleration = Vec3::ZERO;
    }
    fn apply_force(&mut self, force: Vec3) {
        if self.mass == 0.0 {
            return;
        }
        self.acceleration += force / self.mass;
    }
    fn apply_impulse(&mut self, impulse: Vec3) {
        if self.mass == 0.0 {
            return;
        }
        self.position += impulse / self.mass;
    }
    fn reset_forces(&mut self) {
        self.acceleration = Vec3::ZERO;
    }
    fn restrain(&mut self) {
        if self.position.y < 0.0 {
            self.position = self.position - 2.0 * self.position.dot(Vec3::Y) * Vec3::Y;
            self.velocity = self.velocity - 2.0 * self.velocity.dot(Vec3::Y) * Vec3::Y;
            self.previous_position = self.position - self.velocity;
        }
    }
}

#[derive(Component)]
struct Constraint {
    particle_a: Entity,
    particle_b: Entity,
    target: f32,
    stiffness: f32,
    damping: f32,
}

impl Constraint {
    fn new(particle_a: Entity, particle_b: Entity, target: f32) -> Self {
        Self {
            particle_a,
            particle_b,
            target,
            stiffness: 1.0,
            damping: 0.0,
        }
    }
    fn relax(&self, particle_a: &mut Particle, particle_b: &mut Particle) {
        let distance = particle_b.position - particle_a.position;
        let force = 0.5 * self.stiffness * (distance.length() - self.target) * distance.normalize();
        if particle_a.mass != 0.0 && particle_b.mass != 0.0 {
            particle_a.apply_force(-force);
            particle_b.apply_force(force);
        }
    }
}

#[derive(Component)]
struct Matreial {}

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

    
    let mut map = HashMap::<(i32, i32, i32), Entity>::new();

    let resolution = 3;
    let start_y = 1.0;
    let half = 0.15;
    let i_to_vec = |ix: i32, iy: i32, iz: i32| -> Vec3 {
        let x = ix as f32 / resolution as f32 - 0.5 * half * 2.0;
        let y = iy as f32 / resolution as f32 - 0.5 * half * 2.0 + start_y;
        let z = iz as f32 / resolution as f32 - 0.5 * half * 2.0;
        Vec3::new(x, y, z)
    };
    for ix in 0..resolution {
        for iy in 0..resolution {
            for iz in 0..resolution {
                let transform = Transform::from_translation(i_to_vec(ix, iy, iz));
                let particle = commands
                    .spawn(PbrBundle {
                        mesh: meshes.add(Mesh::from(shape::Icosphere {
                            radius: 0.05,
                            subdivisions: 3,
                        })),
                        material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
                        transform,
                        ..default()
                    })
                    .insert(Particle {
                        previous_position: transform.translation,
                        position: transform.translation,
                        velocity: Vec3::ZERO,
                        acceleration: Vec3::Y * -0.01,
                        mass: 0.1,
                    })
                    .id();

                map.insert((ix,iy,iz), particle);
                
                if ix > 0 {
                    let key = (ix-1, iy, iz);
                    let particle_b = map.get(&key).unwrap();
                    let target = (i_to_vec(key.0, key.1, key.2) - transform.translation).length();
                    commands.spawn(Constraint::new(particle, *particle_b, target));
                }
                if iy > 0 {
                    let key = (ix, iy-1, iz);
                    let particle_b = map.get(&key).unwrap();
                    let target = (i_to_vec(key.0, key.1, key.2) - transform.translation).length();
                    commands.spawn(Constraint::new(particle, *particle_b, target));
                }
                if iz > 0 {
                    let key = (ix, iy, iz-1);
                    let particle_b = map.get(&key).unwrap();
                    let target = (i_to_vec(key.0, key.1, key.2) - transform.translation).length();
                    commands.spawn(Constraint::new(particle, *particle_b, target));
                }
                
            }
        }
    }
}

fn update(time: Res<Time>, mut lines: ResMut<DebugLines>, mut particles: Query<(&mut Particle, &mut Transform)>, constraints: Query<&Constraint>) {
    for (mut particle, mut transform) in particles.iter_mut() {
        particle.accelerate(Vec3::Y * -10.0);
        particle.simulate(time.delta_seconds());
        particle.restrain();
        particle.reset_forces();
        transform.translation = particle.position;
    }
    for constraint in constraints.iter() {
        let particle_ab = particles.get_many_mut([constraint.particle_a, constraint.particle_b]);
        if let Ok([mut particle_a, mut particle_b]) = particle_ab {
            println!("relaxing {:?} {:?}", particle_a.0.position, particle_a.0.position);
            constraint.relax(&mut particle_a.0, &mut particle_b.0);
            lines.line(particle_a.0.position, particle_b.0.position, 0.0);
        }
    }
}
