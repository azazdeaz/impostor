use std::f32::consts::TAU;

use bevy::{log, prelude::*, utils::HashMap};
use bevy_prototype_debug_lines::*;
use bevy_rapier3d::prelude::*;
use random_color::RandomColor;
use rand::Rng;

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
struct Particle {
    previous_position: Vec3,
    position: Vec3,
    velocity: Vec3,
    acceleration: Vec3,
    mass: f32,
    is_fixed: bool,
}

impl Particle {
    fn accelerate(&mut self, rate: Vec3) {
        self.acceleration += rate;
    }
    fn simulate(&mut self, delta: f32) {
        if self.is_fixed || self.mass == 0.0 {
            return;
        }
        let friction = 0.95;
        self.velocity = self.position - self.previous_position;
        self.previous_position = self.position;
        self.position += self.velocity * friction + self.acceleration * delta * delta;
        // self.velocity = self.position - self.previous_position;
        // self.acceleration = Vec3::ZERO;

        // self.velocity = self.position - self.previous_position;
        // self.previous_position = self.position;
        // let friction = 0.98;
        // self.position += self.velocity * friction + self.acceleration * delta * delta;
        // self.velocity = self.position - self.previous_position;
        // self.acceleration = Vec3::ZERO;
        // self.velocity = self.position - self.previous_position;

    }
    fn apply_force(&mut self, force: Vec3) {
        if self.mass == 0.0 {
            return;
        }
        self.acceleration += force / self.mass;
        println!(
            "\tapply force: force {:?} mass {:?} new acceleration: {:?}",
            force, self.mass, self.acceleration
        );
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
            let bounce = 0.95;
            self.position = self.position - 2.0 * self.position.dot(Vec3::Y) * Vec3::Y;
            self.velocity = self.velocity - 2.0 * self.velocity.dot(Vec3::Y) * Vec3::Y;
            self.previous_position = self.position - self.velocity * bounce;
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
            stiffness: 0.5,
            damping: 0.0,
        }
    }
    fn relax_old(&self, particle_a: &mut Particle, particle_b: &mut Particle) {
        let distance = particle_b.position - particle_a.position;
        // let distance_length = dista
        if particle_a.mass != 0.0 && particle_b.mass != 0.0 && distance.length() != self.target {
            let force = 0.5 * self.stiffness * (distance.length() - self.target) / self.target
                * distance.normalize();
            println!(
                "relaxing target {} distance {} force {:?} ",
                self.target,
                distance.length(),
                force
            );
            particle_a.apply_force(-force);
            particle_b.apply_force(force);
        }
    }

    fn relax(&self, particle_a: &mut Particle, particle_b: &mut Particle) {
        if particle_a.is_fixed && particle_b.is_fixed {
            return;
        }

        let center = (particle_a.position + particle_b.position) / 2.0;
        let direction = match (particle_b.position - particle_a.position).try_normalize() {
            None => {
                log::warn!("Failed handle stick between points {} and {} which are too close to each other", particle_a.position, particle_b.position);
                return;
            }
            Some(dir) => dir * self.target / 2.0,
        };
        let slowing = 0.8;
        // println!(
        //     "\tcenter {:?} direction {:?} target {:?}",
        //     center, direction, self.target
        // );

        if particle_a.is_fixed {
            particle_b.position = particle_a.position + direction * 2.0;
        } else if particle_b.is_fixed {
            particle_a.position = particle_b.position - direction * 2.0;
        } else {
            particle_a.position = center - direction;
            particle_b.position = center + direction;
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
        .insert(Collider::cuboid(0.2,0.2,4.0))
        .insert(Restitution::coefficient(0.7))
        .insert(Velocity::linear(Vec3::X * 0.6))
        .insert(TransformBundle::from(Transform::from_xyz(-2.0, 2.0, 0.0)));

    let sides = 5;
    let sections = 18;
    let section_height = 0.2;
    let radius = 0.2;
    let start_height = 0.2;
    let mut prev_ring = Vec::new();
    let mut rng = rand::thread_rng();
    for i_section in 0..sections {
        let next_ring: Vec<_> = (0..sides)
            .map(|i_side| {
                let mut angle = TAU * i_side as f32 / sides as f32;
                if i_section % 2 > 0 {
                    angle += TAU / (sides * 2) as f32;
                }

                let (z, x) = angle.sin_cos();
                let z = z + rng.gen::<f32>() * 0.1;
                let x = x + rng.gen::<f32>() * 0.1;
                println!("{} angle {} {} {}", i_side, angle, z, x);
                let y = start_height + section_height * i_section as f32;
                let transform = Transform::from_translation(Vec3::new(x * radius, y, z * radius));
                let [r, g, b] = RandomColor::new().to_rgb_array();
                let particle = Particle {
                    previous_position: transform.translation,
                    position: transform.translation,
                    velocity: Vec3::ZERO,
                    acceleration: Vec3::Y * -0.01,
                    mass: 0.01,
                    is_fixed: i_section == 0,
                };

                let entity = commands
                    .spawn(PbrBundle {
                        mesh: meshes.add(Mesh::from(shape::Icosphere {
                            radius: 0.05,
                            subdivisions: 3,
                        })),
                        material: materials.add(
                            Color::rgb(r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0).into(),
                        ),
                        transform,
                        ..default()
                    })
                    .insert(particle)
                    .insert((
                        RigidBody::KinematicPositionBased,
                        Collider::ball(0.1),
                        SolverGroups::new(Group::NONE, Group::NONE),
                    ))
                    .id();
                (entity, transform.translation)
            })
            .collect();

        let mut add_constraint = |a: (Entity, Vec3), b: (Entity, Vec3)| {
            let target = (a.1 - b.1).length();
            println!("TargetL {} {:?} {:?}", target, a.1, b.1);
            commands.spawn(Constraint::new(a.0, b.0, target));
        };

        let create_tube = false;

        // iterate through the ring
        for i_side in 0..sides {
            // get the index of the next particle on the ring
            let i2_side = (i_side + 1) % sides;
            // connect with the neighbouring particle
            add_constraint(next_ring[i_side], next_ring[i2_side]);

            // Create tube
            if create_tube {
                if i_section > 0 {
                    if i_section % 2 > 0 {
                        add_constraint(prev_ring[i_side], next_ring[i_side]);
                        add_constraint(prev_ring[i2_side], next_ring[i_side]);
                    } else {
                        add_constraint(next_ring[i_side], prev_ring[i_side]);
                        add_constraint(next_ring[i2_side], prev_ring[i_side]);
                    }
                }
            }
            // Connect all particles between segments
            else {
                if i_section > 0 {
                    for i2_side in 0..sides {
                        add_constraint(prev_ring[i2_side], next_ring[i_side]);
                    }
                }
            }
        }

        for i_side in 0..sides {
        }

        prev_ring = next_ring;
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
                constraint.relax(&mut particle_a.0, &mut particle_b.0);
                lines.line(particle_a.0.position, particle_b.0.position, 0.0);
            }
        }
    }
}
