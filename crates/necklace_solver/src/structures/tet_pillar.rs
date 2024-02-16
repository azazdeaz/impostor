use bevy::{ecs::system::Command, prelude::*};
use std::f32::consts::PI;
use crate::{Bond, DragParticle, Point, TetFrame};

#[derive(Default)]
pub struct TetPillar {
    pub sections: usize,
    pub section_height: f32,
    pub radius: f32,
    pub points: Vec<Point>,
    pub start_translation: Vec3,
    pub orientation: Quat,
    pub fix_first_ring: bool,
}

impl TetPillar {
}

impl Command for TetPillar {
    fn apply(self, world: &mut World) {
        let mut points = Vec::new();

        for i in 0..=self.sections {
            // iterate over the angles of the triangle
            for angle in [0.0, PI * 2.0 / 3.0, PI * 4.0 / 3.0].iter() {
                let x = self.radius * angle.cos();
                let z = self.radius * angle.sin();
                let y = i as f32 * self.section_height;
                let point = Point(Vec3::new(x, y, z));
                points.push((world.spawn((point, DragParticle::enabled())).id(), point));
            }
        }
        // Spawn Bonds
        for i in 0..=self.sections {
            let offset = i * 3;
            // connect vertices on this level
            for [a, b] in [[0, 1], [1, 2], [2, 0]].iter() {
                let pa = points[offset + a];
                let pb = points[offset + b];
                world.spawn(Bond {
                    a: pa.0,
                    b: pb.0,
                    length: pa.1.distance(pb.1.0),
                    compliance: 0.1,
                });
            }
            // connect with vertices on the next level
            if i < self.sections {
                for j in 0..3 {
                    for k in 0..3 {
                        // edges.push(EdgeConstraint::from_particles(&particles, offset + j, offset + 3 + k));
                        let pa = points[offset + j];
                        let pb = points[offset + 3 + k];
                        world.spawn(Bond {
                            a: pa.0,
                            b: pb.0,
                            length: pa.1.distance(pb.1.0),
                            compliance: 0.1,
                        });
                    }
                }
            }
        }

        // Spawn tetras
        let tetra_ids = [[0, 1, 2, 5], [0, 3, 4, 5], [0, 1, 4, 5]];
        for i in 0..self.sections {
            let offset = i * 3;
            for tetra in tetra_ids.iter() {
                world.spawn(TetFrame {
                    a: points[offset + tetra[0]].0,
                    b: points[offset + tetra[1]].0,
                    c: points[offset + tetra[2]].0,
                    d: points[offset + tetra[3]].0,
                    length: 0.1,
                    compliance: 0.1,
                });
            }
        }

        // prev_ring = next_ring;
    }
}
