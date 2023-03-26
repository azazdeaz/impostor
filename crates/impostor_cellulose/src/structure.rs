use bevy::{prelude::*, utils::HashMap};
use bevy_rapier3d::prelude::*;
use itertools::Itertools;
use rand::{seq::IteratorRandom, thread_rng, Rng};
use random_color::RandomColor;
use std::f32::consts::TAU;

use crate::{PlantBody, PointId};

pub struct StemStructure {
    pub sides: usize,
    pub sections: usize,
    pub section_height: f32,
    pub radius: f32,
    pub points: HashMap<(usize, usize), PointId>,
    pub start_translation: Vec3,
    pub orientation: Quat,
    pub fix_first_ring: bool,
}

impl StemStructure {
    // Connects the stem to a parent stem
    //  Add Sticks between the first ring and some of the closest points on the parent stem
    pub fn connect_to_stem(&self, body: &mut PlantBody, parent_stem: &StemStructure) {
        let mut rng = thread_rng();
        for i_side in 0..self.sides {
            let &root_point = self.points.get(&(0 as usize, i_side)).unwrap();
            let position = { body.points.get(&root_point).unwrap().position.clone() };
            // find N close points on the parent stem
            let parent_points = parent_stem
                .points
                .values()
                .sorted_by(|p1, p2| {
                    let pos_1 = body.points.get(&p1).unwrap().position;
                    let pos_2 = body.points.get(&p2).unwrap().position;
                    let l1 = (pos_1 - position).length();
                    let l2 = (pos_2 - position).length();
                    l1.total_cmp(&l2)
                })
                // sample randomly from the closest points
                .take(self.sides * 3)
                .choose_multiple(&mut rng, self.sides)
                .clone();
            for parent_point in parent_points {
                body.add_stick(*parent_point, root_point);
            }
        }
    }
    pub fn spawn(
        &mut self,
        body: &mut PlantBody,
        commands: &mut Commands,
        meshes: &mut ResMut<Assets<Mesh>>,
        materials: &mut ResMut<Assets<StandardMaterial>>,
    ) {
        let start =
            Transform::from_translation(self.start_translation).with_rotation(self.orientation);
        // let mut prev_ring = Vec::new();
        let mut rng = rand::thread_rng();
        for i_section in 0..self.sections {
            for i_side in 0..self.sides {
                let mut angle = TAU * i_side as f32 / self.sides as f32;
                if i_section % 2 > 0 {
                    angle += TAU / (self.sides * 2) as f32;
                }

                let (z, x) = angle.sin_cos();
                let z = z + rng.gen::<f32>() * 0.1;
                let x = x + rng.gen::<f32>() * 0.1;
                let y = self.section_height * i_section as f32;
                let transform = start
                    * Transform::from_translation(Vec3::new(x * self.radius, y, z * self.radius));
                let [r, g, b] = RandomColor::new().to_rgb_array();
                let point =
                    body.add_point(transform.translation, self.fix_first_ring && i_section == 0);

                commands
                    .spawn(PbrBundle {
                        mesh: meshes.add(Mesh::from(shape::Icosphere {
                            radius: self.section_height / 20.0,
                            subdivisions: 3,
                        })),
                        material: materials.add(
                            Color::rgb(r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0).into(),
                        ),

                        transform,
                        ..default()
                    })
                    .insert(point)
                    .insert((
                        // RigidBody::KinematicPositionBased,
                        Collider::ball(self.section_height / 4.0),
                        // SolverGroups::new(Group::NONE, Group::NONE),
                    ));

                self.points.insert((i_section, i_side), point);
            }

            let create_tube = false;

            // iterate through the ring
            for i_side in 0..self.sides {
                // get the index of the next particle on the ring
                let next_side = (i_side + 1) % self.sides;
                // connect with the neighbouring particle
                let stick = body.add_stick(
                    self.points[&(i_section, i_side)],
                    self.points[&(i_section, next_side)],
                );
                body.update_stick_growth_direction(stick, 0.0);

                // Create tube
                if create_tube {
                    if i_section > 0 {
                        let prev_section = i_section - 1;
                        if i_section % 2 > 0 {
                            body.add_stick(
                                self.points[&(prev_section, i_side)],
                                self.points[&(i_section, i_side)],
                            );
                            body.add_stick(
                                self.points[&(prev_section, next_side)],
                                self.points[&(i_section, i_side)],
                            );
                        } else {
                            body.add_stick(
                                self.points[&(prev_section, i_side)],
                                self.points[&(i_section, i_side)],
                            );
                            body.add_stick(
                                self.points[&(prev_section, i_side)],
                                self.points[&(i_section, next_side)],
                            );
                        }
                    }
                }
                // Connect all particles between segments
                else {
                    for backreach in 1..=1 {
                        if i_section >= backreach {
                            for i2_side in 0..self.sides {
                                let prev_section = i_section - backreach;
                                let stick = body.add_stick(
                                    self.points[&(prev_section, i2_side)],
                                    self.points[&(i_section, i_side)],
                                );
                                body.update_stick_growth_direction(stick, 1.0);
                            }
                        }
                    }
                }
            }

            // prev_ring = next_ring;
        }
    }
}
