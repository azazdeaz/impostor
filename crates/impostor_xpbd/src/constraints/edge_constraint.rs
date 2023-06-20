use crate::constraints::XPBDConstraint;
use crate::structs::*;
use bevy::prelude::*;
use bevy_prototype_debug_lines::DebugShapes;
use itertools::Itertools;

pub struct EdgeConstraint {
    pub a: usize,
    pub b: usize,
    pub rest_length: f32,
    pub compliance: f32,
}
impl EdgeConstraint {
    pub fn from_particles(particles: &Vec<Particle>, a: usize, b: usize) -> Self {
        let rest_length = (particles[a].position - particles[b].position).length();
        Self {
            a,
            b,
            rest_length,
            compliance: 0.1,
        }
    }
}

impl XPBDConstraint for EdgeConstraint {
    fn get_compliance(&self) -> f32 {
        self.compliance
    }
    fn solve(&self, particles: &mut Vec<Particle>, delta_squared: f32) {
        let alpha = self.compliance / delta_squared;
        let w = particles[self.a].inverse_mass + particles[self.b].inverse_mass;
        if w == 0.0 {
            return;
        }
        let p1 = particles[self.a].position;
        let p2 = particles[self.b].position;
        let diff = p1 - p2;
        let distance = diff.length();
        if distance == 0.0 {
            return;
        }
        let direction = diff / distance;
        let delta = p2 - p1;
        let distance = delta.length();
        let residual = -(distance - self.rest_length) / (w + alpha);
        particles[self.a].position =
            p1 + direction * residual * particles[self.a].inverse_mass;
        particles[self.b].position =
            p2 - direction * residual * particles[self.b].inverse_mass;
        
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
