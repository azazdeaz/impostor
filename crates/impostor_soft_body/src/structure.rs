use bevy::{prelude::*, utils::HashMap};
use bevy_rapier3d::prelude::*;
use itertools::Itertools;
use rand::{seq::IteratorRandom, thread_rng, Rng};
use random_color::RandomColor;
use std::f32::consts::TAU;

use crate::components::{Constraint, Particle};

pub struct StemStructure {
    pub sides: usize,
    pub sections: usize,
    pub section_height: f32,
    pub radius: f32,
    pub particles: HashMap<(usize, usize), (Particle, Entity)>,
    pub start_translation: Vec3,
    pub orientation: Quat,
    pub fix_first_ring: bool,
}

impl StemStructure {
    pub fn connect_to_stem(&self, commands: &mut Commands, stem: &StemStructure) {
        let mut add_constraint = |a: (Particle, Entity), b: (Particle, Entity)| {
            let target = (a.0.position - b.0.position).length();
            commands.spawn(Constraint::new(a.1, b.1, target));
        };

        let mut rng = thread_rng();
        for i_side in 0..self.sides {
            let particle = self.particles.get(&(0 as usize, i_side)).unwrap();
            let position = particle.0.position.clone();
            // find N close particle on the parent stem
            let parent_particles = stem
                .particles
                .clone()
                .into_values()
                .sorted_by(|p1, p2| {
                    let l1 = (p1.0.position - position).length();
                    let l2 = (p2.0.position - position).length();
                    l1.total_cmp(&l2)
                })
                // sample randomly from the closest particles
                .take(self.sides * 3)
                .choose_multiple(&mut rng, self.sides);
            for parent_particle in parent_particles {
                add_constraint(parent_particle, *particle)
            }
        }
    }
    pub fn spawn(
        &mut self,
        commands: &mut Commands,
        meshes: &mut ResMut<Assets<Mesh>>,
        materials: &mut ResMut<Assets<StandardMaterial>>,
    ) {
        let start =
            Transform::from_translation(self.start_translation).with_rotation(self.orientation);
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
                    let y = self.section_height * i_section as f32;
                    let transform = start
                        * Transform::from_translation(Vec3::new(
                            x * self.radius,
                            y,
                            z * self.radius,
                        ));
                    let [r, g, b] = RandomColor::new().to_rgb_array();
                    let particle = Particle {
                        previous_position: transform.translation,
                        position: transform.translation,
                        velocity: Vec3::ZERO,
                        acceleration: Vec3::Y * -0.01,
                        mass: 0.01,
                        is_fixed: self.fix_first_ring && i_section == 0,
                    };

                    let entity = commands
                        .spawn(PbrBundle {
                            mesh: meshes.add(Mesh::from(shape::Icosphere {
                                radius: self.section_height / 2.0,
                                subdivisions: 3,
                            })),
                            material: materials.add(
                                Color::rgb(r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0)
                                    .into(),
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

                    self.particles
                        .insert((i_section, i_side), (particle, entity));
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
