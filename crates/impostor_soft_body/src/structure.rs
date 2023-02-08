use bevy::{ prelude::*, utils::HashMap};
use bevy_rapier3d::prelude::*;
use random_color::RandomColor;
use rand::Rng;
use std::f32::consts::TAU;


use crate::components::{Particle, Constraint};

pub struct StemStructure {
    pub sides: usize,
    pub sections: usize,
    pub section_height: f32,
    pub radius: f32,
    pub particles: HashMap<(usize, usize), Particle>
}

impl StemStructure {
    pub fn spawn(&self, mut commands: Commands,
        mut meshes: ResMut<Assets<Mesh>>,
        mut materials: ResMut<Assets<StandardMaterial>>) {


            let start_height = 0.8;
    let mut prev_ring = Vec::new();
    let mut rng = rand::thread_rng();
    for i_section in 0..self.sections {
        let next_ring: Vec<_> = (0..self.sides)
            .map(|i_side| {
                let mut angle = TAU * i_side as f32 / self.sides as f32;
                if i_section % 2 > 0 {
                    angle += TAU / (self.sides * 2) as f32;
                }

                let (z, x) = angle.sin_cos();
                let z = z + rng.gen::<f32>() * 0.1;
                let x = x + rng.gen::<f32>() * 0.1;
                let y = start_height + self.section_height * i_section as f32;
                println!("section {}, side {}, x {}, z {}, y {}", i_section, i_side, x, z, y);
                let transform = Transform::from_translation(Vec3::new(x * self.radius, y, z * self.radius));
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
                            radius: self.section_height / 2.0,
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
                        Collider::ball(self.section_height / 2.0),
                        SolverGroups::new(Group::NONE, Group::NONE),
                    ))
                    .id();
                (entity, transform.translation)
            })
            .collect();

        let mut add_constraint = |a: (Entity, Vec3), b: (Entity, Vec3)| {
            let target = (a.1 - b.1).length();
            commands.spawn(Constraint::new(a.0, b.0, target));
        };

        let create_tube = false;

        // iterate through the ring
        for i_side in 0..self.sides {
            // get the index of the next particle on the ring
            let i2_side = (i_side + 1) % self.sides;
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
                    for i2_side in 0..self.sides {
                        add_constraint(prev_ring[i2_side], next_ring[i_side]);
                    }
                }
            }
        }

        prev_ring = next_ring;
    }
    }
}