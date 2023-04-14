use std::f32::consts::PI;

use bevy::prelude::*;
use crate::constraints::*;

use super::Particle;

pub struct SoftBody {
    pub particles: Vec<Particle>,
    pub constraints: Vec<Box<dyn XPBDConstraint + Send + Sync>>,
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
    }

    pub fn post_solve(&mut self, delta: f32) {
        for particle in self.particles.iter_mut() {
            if particle.inverse_mass == 0.0 {
                continue;
            }
            if delta > 0.0 {
                particle.velocity = (particle.position - particle.prev_position) / delta * 0.9;
            }
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
        let sections = 6;
        let mut particles = Vec::new();
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

        // // add bending constraints
        // for i in 1..sections {
        //     for j in 0..3 {
        //         //head of the lower triangle
        //         let c = (i - 1) * 3 + j;
        //         // the common mase of the triangles
        //         let a = i * 3 + j;
        //         let b = i * 3 + (j + 2) % 3;
        //         // head of the upper triangle
        //         let d = (i + 1) * 3 + (j + 2) % 3;
        //         let constraint = IsometricBendingConstraint::from_particles(&particles, a, b, c, d);
        //         constraints.push(Box::new(constraint));
        //     }
        // }

        // create edges between neighboring particles
        for i in 0..=sections {
            let offset = i * 3;
            // connect vertices on this level
            constraints.push(Box::new(EdgeConstraint::from_particles(&particles, offset, offset + 1)));
            constraints.push(Box::new(EdgeConstraint::from_particles(&particles, offset + 1, offset + 2)));
            constraints.push(Box::new(EdgeConstraint::from_particles(&particles, offset + 2, offset)));
            // connect with vertices on the next level
            if i < sections {
                for j in 0..3 {
                    for k in 0..3 {
                        constraints.push(Box::new(EdgeConstraint::from_particles(&particles, offset + j, offset + 3 + k)));
                        let k_next = (k + 1) % 3;
                        constraints.push(Box::new(EdgeConstraint::from_particles(
                            &particles,
                            offset + j,
                            offset + 3 + k_next,
                        )));
                    }
                }
            }
            // add bending constraints betwen up-down triangle pairs
            if i < (sections - 1) {
                for j in 0..3 {
                    let j_prev = (j + 2) % 3;
                    let mut constraint = EdgeConstraint::from_particles(
                        &particles,
                        offset + j,
                        offset + 6 + j_prev,
                    );
                    constraints.push(Box::new(constraint));
                }

            }
        }

        // set the inverse mass of the first section to zero
        for i in 0..3 {
            particles[i].inverse_mass = 0.0;
        }

        SoftBody {
            particles,
            constraints,
        }
    }
}