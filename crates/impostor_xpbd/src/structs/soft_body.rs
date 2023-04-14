use std::f32::consts::PI;

use bevy::prelude::*;
use serde_json::json;
use crate::constraints::{XPBDConstraint, IsometricBendingConstraint, VolumeConstraint};
use itertools::{izip, Itertools};

use super::Particle;

pub struct SoftBody {
    pub particles: Vec<Particle>,
    pub edges: Vec<Edge>,
    pub constraints: Vec<Box<dyn XPBDConstraint + Send + Sync>>,
    pub edge_compliance: f32,
    pub volume_compliance: f32,
    pub bending_compliance: f32,
}

pub struct Edge {
    pub a: usize,
    pub b: usize,
    pub rest_length: f32,
}
impl Edge {
    fn from_particles(particles: &Vec<Particle>, a: usize, b: usize) -> Self {
        let rest_length = (particles[a].position - particles[b].position).length();
        Self { a, b, rest_length }
    }
}




impl SoftBody {
    pub fn pre_solve(&mut self, gravity: Vec3, delta: f32) {
        for particle in self.particles.iter_mut() {
            particle.velocity += gravity * delta;
            particle.prev_position = particle.position;
            particle.position += particle.velocity * delta;

            // bounce off the ground
            if particle.position.y < 0.0 {
                particle.position = particle.prev_position;
                particle.position.y = 0.0;
            }
        }
    }

    pub fn solve(&mut self, delta: f32) {
        let delta_squared = delta * delta;

        for constrain in self.constraints.iter() {
            constrain.solve(&mut self.particles, delta_squared);
        }

        self.solve_edges(delta);
    }

    pub fn post_solve(&mut self, delta: f32) {
        for particle in self.particles.iter_mut() {
            particle.velocity = (particle.position - particle.prev_position) * delta;
        }
    }

    pub fn solve_edges(&mut self, delta: f32) {
        let alpha = self.edge_compliance / delta / delta;
        for edge in self.edges.iter() {
            let w = self.particles[edge.a].inverse_mass + self.particles[edge.b].inverse_mass;
            if w == 0.0 {
                continue;
            }
            let p1 = self.particles[edge.a].position;
            let p2 = self.particles[edge.b].position;
            let diff = p1 - p2;
            let distance = diff.length();
            if distance == 0.0 {
                continue;
            }
            let direction = diff / distance;
            let delta = p2 - p1;
            let distance = delta.length();
            let residual = -(distance - edge.rest_length) / (w + alpha);
            self.particles[edge.a].position =
                p1 + direction * residual * self.particles[edge.a].inverse_mass;
            self.particles[edge.b].position =
                p2 - direction * residual * self.particles[edge.b].inverse_mass;
        }
    }


    // pub fn new_triangle_pillar() -> Self {
    //     let section_length = 0.4;
    //     let radius = 0.2;
    //     let sections = 7;
    //     let mut particles = Vec::new();
    //     let mut edges = Vec::new();
    //     let mut tetras = Vec::new();

    //     for i in 0..=sections {
    //         // iterate over the angles of the triangle
    //         for angle in [0.0, PI * 2.0 / 3.0, PI * 4.0 / 3.0].iter() {
    //             let x = radius * angle.cos();
    //             let z = radius * angle.sin();
    //             let y = i as f32 * section_length;
    //             particles.push(Particle::from_position(Vec3::new(x, y, z)));
    //         }
    //     }

    //     // create three tetraherons filling up each section without overlap
    //     let tetra_ids = [[0, 1, 2, 5], [0, 3, 4, 5], [0, 1, 4, 5]];
    //     for i in 0..sections {
    //         let offset = i * 3;
    //         for tetra in tetra_ids.iter() {
    //             tetras.push(Tetra::from_particles(
    //                 &mut particles,
    //                 offset + tetra[0],
    //                 offset + tetra[1],
    //                 offset + tetra[2],
    //                 offset + tetra[3],
    //             ));
    //         }
    //     }

    //     // create edges between neighboring particles
    //     for i in 0..=sections {
    //         let offset = i * 3;
    //         // connect vertices on this level
    //         edges.push(Edge::from_particles(&particles, offset, offset + 1));
    //         edges.push(Edge::from_particles(&particles, offset + 1, offset + 2));
    //         edges.push(Edge::from_particles(&particles, offset + 2, offset));
    //         // connect with vertices on the next level
    //         if i < sections {
    //             for j in 0..3 {
    //                 for k in 0..3 {
    //                     edges.push(Edge::from_particles(&particles, offset + j, offset + 3 + k));
    //                 }
    //             }
    //         }
    //     }

    //     // set the inverse mass of the first section to zero
    //     for i in 0..3 {
    //         particles[i].inverse_mass = 0.0;
    //     }

    //     SoftBody {
    //         particles,
    //         edges,
    //         tetras,
    //         bending_constraints: Vec::new(),
    //         edge_compliance: 0.9,
    //         volume_compliance: 0.9,
    //         bending_compliance: 0.9,
    //     }
    // }

    pub fn new_octaeder_pillar() -> Self {
        let section_length = 0.4;
        let radius = 0.2;
        let sections = 7;
        let mut particles = Vec::new();
        let mut edges = Vec::new();
        let mut constraints: Vec<Box<dyn XPBDConstraint + Send + Sync>> = Vec::new();

        for i in 0..=sections {
            // iterate over the angles of the triangle
            for angle in [0.0, PI * 2.0 / 3.0, PI * 4.0 / 3.0].iter() {
                // make every following sector rotated by 1/3 PI
                let angle = angle + (PI * 1.0 / 3.0) * i as f32;
                let x = radius * angle.cos();
                let z = radius * angle.sin();
                let y = i as f32 * section_length;
                particles.push(Particle::from_position(Vec3::new(x, y, z)));
            }
        }

        // create three tetraherons filling up each section without overlap
        let tetra_ids = [[0, 5, 2, 4], [0, 3, 5, 4], [0, 2, 1, 3], [3, 4, 2, 1]];
        for i in 0..sections {
            let offset = i * 3;
            for tetra in tetra_ids.iter() {
                constraints.push(Box::new(VolumeConstraint::from_particles(
                    &mut particles,
                    offset + tetra[0],
                    offset + tetra[1],
                    offset + tetra[2],
                    offset + tetra[3],
                )));
            }
        }

        // add bending constraints
        for i in 1..sections {
            for j in 0..3 {
                //head of the lower triangle
                let c = (i - 1) * 3 + j;
                // the common mase of the triangles
                let a = i * 3 + j;
                let b = i * 3 + (j + 2) % 3;
                // head of the upper triangle
                let d = (i + 1) * 3 + (j + 2) % 3;
                let constraint = IsometricBendingConstraint::from_particles(&particles, a, b, c, d);
                constraints.push(Box::new(constraint));
            }
        }

        // create edges between neighboring particles
        for i in 0..=sections {
            let offset = i * 3;
            // connect vertices on this level
            edges.push(Edge::from_particles(&particles, offset, offset + 1));
            edges.push(Edge::from_particles(&particles, offset + 1, offset + 2));
            edges.push(Edge::from_particles(&particles, offset + 2, offset));
            // connect with vertices on the next level
            if i < sections {
                for j in 0..3 {
                    for k in 0..3 {
                        edges.push(Edge::from_particles(&particles, offset + j, offset + 3 + k));
                        let k_next = (k + 1) % 3;
                        edges.push(Edge::from_particles(
                            &particles,
                            offset + j,
                            offset + 3 + k_next,
                        ));
                    }
                }
            }
        }

        // set the inverse mass of the first section to zero
        for i in 0..3 {
            particles[i].inverse_mass = 0.0;
        }

        SoftBody {
            particles,
            edges,
            constraints,
            edge_compliance: 0.9,
            volume_compliance: 0.9,
            bending_compliance: 0.9,
        }
    }
}