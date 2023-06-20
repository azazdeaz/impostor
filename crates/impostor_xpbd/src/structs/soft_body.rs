use bevy::prelude::*;
use slotmap::{new_key_type, SlotMap};
use crate::constraints::*;

use super::{Particle, Orientation, DebugFigure, orientation, time_integration};

new_key_type! { pub struct OrientationKey; }
new_key_type! { pub struct ParticleKey; }

#[derive(Default)]
pub struct SoftBodyData {
    pub particles: SlotMap<ParticleKey, Particle>,
    pub orientations: SlotMap<OrientationKey, Orientation>,
    pub debug_figures: Vec<DebugFigure>,
}

impl SoftBodyData {
    pub fn add_fig(&mut self, fig: DebugFigure) {
        self.debug_figures.push(fig);
    }
}   

#[derive(Default)]
pub struct SoftBody {
    pub data: SoftBodyData,
    pub constraints: Vec<Box<dyn XPBDConstraint + Send + Sync>>,
}

impl SoftBody {
    pub fn pre_solve(&mut self, gravity: Vec3, delta: f32) {
        for particle in self.data.particles.values_mut() {
            if particle.inverse_mass == 0.0 {
                continue;
            }
            particle.prev_position = particle.position;
            
            // Semi-implicit Euler integration
            particle.velocity += gravity * delta;
            particle.position += particle.velocity * delta;
        }

        for orientation in self.data.orientations.values_mut() {
            if orientation.inverse_mass == 0.0 {
                continue;
            }
            orientation.old_quaternion = orientation.quaternion;
            time_integration::semi_implicit_euler_rotation(delta, Vec3::ZERO, orientation);
        }
    }

    pub fn solve(&mut self, delta: f32) {
        let delta_squared = delta * delta;

        for constrain in self.constraints.iter() {
            constrain.solve(&mut self.data, delta_squared);
        }
    }

    pub fn post_solve(&mut self, delta: f32) {
        for particle in self.data.particles.values_mut() {
            if particle.inverse_mass == 0.0 {
                continue;
            }
            if delta > 0.0 {
                particle.velocity = (particle.position - particle.prev_position) / delta;
            }
        }

        for orientation in self.data.orientations.values_mut() {
            time_integration::angular_velocity_update_first_order(delta, orientation);
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

    // pub fn new_octaeder_pillar() -> Self {
    //     let section_length = 0.4;
    //     let radius = 0.2;
    //     let sections = 6;
    //     let mut particles = Vec::new();
    //     let mut constraints: Vec<Box<dyn XPBDConstraint + Send + Sync>> = Vec::new();

    //     for i in 0..=sections {
    //         // iterate over the angles of the triangle
    //         for angle in [0.0, PI * 2.0 / 3.0, PI * 4.0 / 3.0].iter() {
    //             // make every following sector rotated by 1/3 PI
    //             let angle = angle + (PI * 1.0 / 3.0) * i as f32;
    //             let x = radius * angle.cos();
    //             let z = radius * angle.sin();
    //             let y = i as f32 * section_length;
    //             particles.push(Particle::from_position(Vec3::new(x, y, z)));
    //         }
    //     }

    //     // create three tetraherons filling up each section without overlap
    //     let tetra_ids = [[0, 5, 2, 4], [0, 3, 5, 4], [0, 2, 1, 3], [3, 4, 2, 1]];
    //     for i in 0..sections {
    //         let offset = i * 3;
    //         for tetra in tetra_ids.iter() {
    //             constraints.push(Box::new(VolumeConstraint::from_particles(
    //                 &mut particles,
    //                 offset + tetra[0],
    //                 offset + tetra[1],
    //                 offset + tetra[2],
    //                 offset + tetra[3],
    //             )));
    //         }
    //     }

    //     // add bending constraints
    //     for i in 1..sections {
    //         for j in 0..3 {
    //             //head of the lower triangle
    //             let c = (i - 1) * 3 + j;
    //             // the common mase of the triangles
    //             let a = i * 3 + j;
    //             let b = i * 3 + (j + 2) % 3;
    //             // head of the upper triangle
    //             let d = (i + 1) * 3 + (j + 2) % 3;
    //             let constraint = IsometricBendingConstraint::from_particles(&particles, a, b, c, d);
    //             constraints.push(Box::new(constraint));
    //         }
    //     }

    //     // create edges between neighboring particles
    //     for i in 0..=sections {
    //         let offset = i * 3;
    //         // connect vertices on this level
    //         constraints.push(Box::new(EdgeConstraint::from_particles(&particles, offset, offset + 1)));
    //         constraints.push(Box::new(EdgeConstraint::from_particles(&particles, offset + 1, offset + 2)));
    //         constraints.push(Box::new(EdgeConstraint::from_particles(&particles, offset + 2, offset)));
    //         // connect with vertices on the next level
    //         if i < sections {
    //             for j in 0..3 {
    //                 for k in 0..3 {
    //                     constraints.push(Box::new(EdgeConstraint::from_particles(&particles, offset + j, offset + 3 + k)));
    //                     let k_next = (k + 1) % 3;
    //                     constraints.push(Box::new(EdgeConstraint::from_particles(
    //                         &particles,
    //                         offset + j,
    //                         offset + 3 + k_next,
    //                     )));
    //                 }
    //             }
    //         }
    //         // add bending constraints betwen up-down triangle pairs
    //         // if i < (sections - 1) {
    //         //     for j in 0..3 {
    //         //         let j_prev = (j + 2) % 3;
    //         //         let mut constraint = EdgeConstraint::from_particles(
    //         //             &particles,
    //         //             offset + j,
    //         //             offset + 6 + j_prev,
    //         //         );
    //         //         constraints.push(Box::new(constraint));
    //         //     }

    //         // }
    //     }

    //     // set the inverse mass of the first section to zero
    //     for i in 0..3 {
    //         particles[i].inverse_mass = 0.0;
    //     }

    //     SoftBody {
    //         particles,
    //         constraints,
    //         ..Default::default()
    //     }
    // }

    // pub fn new_triangle_pie_pillar() -> Self {
    //     let section_length = 0.17    ;
    //     let radius = 0.1;
    //     let sections = 7;
    //     // triangles per section
    //     let slices = 5;
    //     let mut particles = Vec::new();
    //     let mut constraints: Vec<Box<dyn XPBDConstraint + Send + Sync>> = Vec::new();
        

    //     for i in 0..=sections {
    //         let y = i as f32 * section_length;
    //         // center of the slice
    //         particles.push(Particle::from_position(Vec3::new(0.0, y, 0.0)));
    //         // iterate over the angles of the section
    //         for j in 0..slices {
    //             let angle = PI * 2.0 * j as f32 / slices as f32;
    //             let x = radius * angle.cos();
    //             let z = radius * angle.sin();
    //             particles.push(Particle::from_position(Vec3::new(x, y, z)));
    //         }
    //     }

    //     macro_rules! edge {
    //         ($a:expr, $b:expr) => {
    //             constraints.push(Box::new(EdgeConstraint::from_particles(&particles, $a, $b)));
    //         };
    //     }

    //     macro_rules! volume {
    //         ($a:expr, $b:expr, $c:expr, $d:expr) => {
    //             constraints.push(Box::new(VolumeConstraint::from_particles(
    //                 &mut particles,
    //                 $a,
    //                 $b,
    //                 $c,
    //                 $d,
    //             )));
    //         };
    //     }
        

    //     // number of particles per section (slices + 1 center particle)
    //     let up = slices + 1; 
        
    //     for i in 0..sections {
    //         // Fill each slice with three tetraherons
    //         let lower_center = i * up;
    //         let upper_center = (i + 1) * up;
    //         for j in 0..slices {
    //             let pa_offset = 1 + j;
    //             let pb_offset = 1 + ((j + 1) % slices);
    //             println!("lower_center: {}, upper_center: {}, pa_offset: {}, pb_offset: {}", lower_center, upper_center, pa_offset, pb_offset);
    //             let lower = [lower_center, lower_center + pa_offset, lower_center + pb_offset];
    //             let upper = [upper_center, upper_center + pa_offset, upper_center + pb_offset];
                
    //             // create three tetraherons filling up each slice without overlap
    //             // If up=6, the first three tetraherons are [[0,1,2,9], [0,1,8,9], [0,7,8,9]]
    //             volume!(lower[0], lower[1], lower[2], upper[2]);
    //             volume!(lower[0], lower[1], upper[1], upper[2]);
    //             volume!(lower[0], upper[0], upper[1], upper[2]);
    //             }
                
    //     }

    //     // // create edges between neighboring particles
    //     // for i in 0..=sections {
    //     //     // Connect each neighboring particle with an EdgeConstraint
    //     //     let lower_center = i * up;
    //     //     let upper_center = (i + 1) * up;
    //     //     // Center vertical edge
    //     //     if i < sections {
    //     //         edge!(lower_center, upper_center);
    //     //     }
    //     //     for j in 0..slices {
    //     //         let pb_offset = 1 + ((j + 1) % slices);
    //     //         let pa_offset = 1 + j;
                
    //     //         if i < sections {
    //     //             // Side vertical edge
    //     //             edge!(lower_center + pa_offset, upper_center + pa_offset);
    //     //             // Diagonal edges
    //     //             edge!(lower_center + pa_offset, upper_center + pb_offset); 
    //     //             edge!(lower_center + pb_offset, upper_center + pa_offset);
    //     //             edge!(lower_center, upper_center + pa_offset);
    //     //             edge!(upper_center, lower_center + pa_offset)
    //     //         }
    //     //         // Horizontal edges
    //     //         edge!(lower_center, lower_center + pa_offset);
    //     //         edge!(lower_center + pa_offset, lower_center + pb_offset);

    //     //     }
            
    //     // }

    //     // Add EdgeConstraints between each particle on the same level and the next level
    //     for i in 0..=sections {
    //         for j in 0..slices {
    //             let start = i * up + j;
    //             // iterate all the following particles up until the next level
    //             for k in (start+1)..(start + 2 * up - 1 - j) {
    //                 if k >= particles.len() {
    //                     break;
    //                 }
    //                 edge!(start, k);
    //             }
    //         }
    //     }  

    //     // set the inverse mass of the first section
    //     for i in 0..=slices {
    //         particles[i].inverse_mass = 0.0;
    //     }

    //     SoftBody {
    //         particles,
    //         constraints,
    //         ..Default::default(),
    //     }
    // }

    // pub fn new_stem_bend_based() -> Self {
    //     let sections = 3;
    //     let section_length = 0.4;
    //     let mut particles = Vec::new();
    //     let mut constraints: Vec<Box<dyn XPBDConstraint + Send + Sync>> = Vec::new();

    //     for i in 0..sections {
    //         let y = i as f32 * section_length;
    //         particles.push(Particle::from_position(Vec3::new(0.1*i as f32, y, 0.0)));
    //     }
    //     for i in 0..(sections-1) {
    //         let constraint = StemBendConstraint::from_particles(&particles, i, i+1);
    //         constraints.push(Box::new(constraint));
    //     }
    //     SoftBody {
    //         particles,
    //         constraints,
    //         ..Default::default()
    //     }
    // }
}