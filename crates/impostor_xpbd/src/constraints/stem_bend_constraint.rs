use crate::constraints::XPBDConstraint;
use crate::structs::*;
use bevy::prelude::*;
use bevy_prototype_debug_lines::DebugShapes;

pub struct StemBendConstraint {
    pub a: usize,
    pub b: usize,
    pub rest_direction: f32,
    pub rest_radius: f32,
    pub rest_length: f32,
    pub compliance: f32,
}
impl StemBendConstraint {
    pub fn from_particles(particles: &Vec<Particle>, a: usize, b: usize) -> Self {
        let rest_length = (particles[a].position - particles[b].position).length();
        let transform_a =
            Transform::from_translation(particles[a].position).with_rotation(particles[a].rotation);
        let transform_b =
            Transform::from_translation(particles[b].position).with_rotation(particles[b].rotation);
        let local = Transform::from_matrix(
            transform_a.compute_matrix().inverse() * transform_b.compute_matrix(),
        );
        Self {
            a,
            b,
            rest_direction: local.translation.z.atan2(local.translation.x),
            rest_radius: local.translation.x.hypot(local.translation.z),
            rest_length,
            compliance: 0.1,
        }
    }

    fn compute_params(&self, particles: &Vec<Particle>) -> (f32, f32, f32) {
        let transform_a =
            Transform::from_translation(particles[self.a].position).with_rotation(particles[self.a].rotation);
        let transform_b =
            Transform::from_translation(particles[self.b].position).with_rotation(particles[self.b].rotation);
        let local = Transform::from_matrix(
            transform_a.compute_matrix().inverse() * transform_b.compute_matrix()
        );
        let direction = local.translation.z.atan2(local.translation.x);
        let radius = local.translation.x.hypot(local.translation.z);
        let length = radius.hypot(local.translation.y);
        (direction, radius, length)
    }
}

impl XPBDConstraint for StemBendConstraint {
    fn get_compliance(&self) -> f32 {
        self.compliance
    }
    fn solve(&self, particles: &mut Vec<Particle>, delta_squared: f32) {
        let alpha = self.compliance / delta_squared;
        // let w = particles[self.a].inverse_mass + particles[self.b].inverse_mass;
        // if w == 0.0 {
        //     return;
        // }
        // let p1 = particles[self.a].position;
        // let p2 = particles[self.b].position;
        // let diff = p1 - p2;
        // let distance = diff.length();
        // if distance == 0.0 {
        //     return;
        // }
        // let direction = diff / distance;
        // let delta = p2 - p1;
        // let distance = delta.length();
        // let residual = -(distance - self.rest_length) / (w + alpha);
        // particles[self.a].position = p1 + direction * residual * particles[self.a].inverse_mass;
        // particles[self.b].position = p2 - direction * residual * particles[self.b].inverse_mass;
    }

    fn debug_draw(&self, particles: &Vec<Particle>, shapes: &mut DebugShapes) {
        shapes
            .line()
            .start(particles[self.a].position)
            .end(particles[self.b].position)
            .color(Color::WHITE);
    }
}

#[cfg(test)]
mod tests {}
