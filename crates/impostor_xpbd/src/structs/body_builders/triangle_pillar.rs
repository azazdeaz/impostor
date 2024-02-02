use std::f32::consts::PI;

use crate::{structs::{SoftBody, Particle, Orientation}, constraints::{BendTwistConstraint, EdgeConstraint, StretchShearConstraint}};
use bevy::prelude::*;

impl SoftBody {
    pub fn build_triangle_pillar() -> Self {
        let section_length = 0.4;
        let radius = 0.2;
        let sections = 7;
        let mut body = SoftBody::default();
        let mut edges = Vec::new();
        let mut tetras = Vec::new();

        for i in 0..=sections {
            // iterate over the angles of the triangle
            for angle in [0.0, PI * 2.0 / 3.0, PI * 4.0 / 3.0].iter() {
                let x = radius * angle.cos();
                let z = radius * angle.sin();
                let y = i as f32 * section_length;
                body.particles.push(Particle::from_position(Vec3::new(x, y, z)));
            }
        }

        // create three tetraherons filling up each section without overlap
        let tetra_ids = [[0, 1, 2, 5], [0, 3, 4, 5], [0, 1, 4, 5]];
        for i in 0..sections {
            let offset = i * 3;
            for tetra in tetra_ids.iter() {
                tetras.push(Tetra::from_particles(
                    &mut particles,
                    offset + tetra[0],
                    offset + tetra[1],
                    offset + tetra[2],
                    offset + tetra[3],
                ));
            }
        }

        // create edges between neighboring particles
        for i in 0..=sections {
            let offset = i * 3;
            // connect vertices on this level
            edges.push(EdgeConstraint::from_particles(&body.particles, offset, offset + 1));
            edges.push(EdgeConstraint::from_particles(&body.particles, offset + 1, offset + 2));
            edges.push(EdgeConstraint::from_particles(&body.particles, offset + 2, offset));
            // connect with vertices on the next level
            if i < sections {
                for j in 0..3 {
                    for k in 0..3 {
                        edges.push(EdgeConstraint::from_particles(&particles, offset + j, offset + 3 + k));
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
            tetras,
            bending_constraints: Vec::new(),
            edge_compliance: 0.9,
            volume_compliance: 0.9,
            bending_compliance: 0.9,
        }
    }
}