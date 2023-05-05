use crate::constraints::XPBDConstraint;
use crate::structs::*;
use bevy::prelude::*;
use bevy_prototype_debug_lines::DebugShapes;

#[derive(Default)]
pub struct StemBendConstraint {
    pub a: usize,
    pub b: usize,
    pub rest_direction: f32,
    pub rest_radius: f32,
    pub rest_length: f32,
    pub normal: Vec3,
    pub zero: Vec3,
    pub compliance: f32,
}
impl StemBendConstraint {
    pub fn from_particles(particles: &Vec<Particle>, a: usize, b: usize) -> Self {
        let mut constraint = Self {
            a,
            b,
            normal: Vec3::Y,
            zero: Vec3::Z,
            ..Default::default()
        };
        let (direction, radius, length) = constraint.compute_params(particles);
        constraint.rest_direction = direction;
        constraint.rest_radius = radius;
        constraint.rest_length = length;
        constraint
    }

    fn compute_params(&self, particles: &Vec<Particle>) -> (f32, f32, f32) {
        let pa = particles[self.a].position;
        let pb = particles[self.b].position;
        // project pb onto the plane defined by pa and normal
        let ab = pb - pa;
        let length = ab.length();
        let projection = ab - self.normal * ab.dot(self.normal);
        let radius = projection.length();
        // find the unsigned angle distance between the projection and the zero vector 
        let mut direction = self.zero.angle_between(projection);  
        // if the projection is in the negative direction, flip the angle 
        // normal ⋅ (zero × b) < 0 (compute the triple product)
        if projection.cross(self.zero).dot(self.normal) < 0.0 {
            direction = -direction;
        }
        (direction, radius, length)
    }
}

impl XPBDConstraint for StemBendConstraint {
    fn get_compliance(&self) -> f32 {
        self.compliance
    }
    fn solve(&self, particles: &mut Vec<Particle>, delta_squared: f32) {
        println!("solve {} -> {} * {}", self.a, self.b, delta_squared);
        let alpha = self.compliance / delta_squared;
        let (direction, radius, length) = self.compute_params(particles);
        let radius = radius + (self.rest_radius - radius) * alpha;
        let length = length + (self.rest_length - length) * alpha;
        println!("direction {} -> {}", self.rest_direction, direction);
        println!("radius {} -> {}", self.rest_radius, radius);
        println!("length {} -> {}", self.rest_length, length);
        // let direction = self.rest_direction;
        // let radius = self.rest_radius;
        // let length = self.rest_length;

        // Compute the position on the section plane
        let mut ab_translation = Quat::from_axis_angle(self.normal, self.rest_direction) * self.zero;
        // Add the height
        ab_translation += self.normal * (length.powi(2) - radius.powi(2)).sqrt();
        particles[self.b].position = particles[self.a].position + ab_translation;

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
        shapes
            .cuboid()
            .position(particles[self.a].position)
            .size(Vec3::ONE * 0.1)
            .color(Color::PINK);
    }
}

#[cfg(test)]
mod tests {}
