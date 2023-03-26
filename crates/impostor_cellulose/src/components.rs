use bevy::{log, prelude::*};
use nalgebra::Const;

#[derive(Component, Copy, Clone)]
pub struct Particle {
    pub previous_position: Vec3,
    pub position: Vec3,
    pub velocity: Vec3,
    pub acceleration: Vec3,
    pub mass: f32,
    pub is_fixed: bool,
}

impl Particle {
    pub fn accelerate(&mut self, rate: Vec3) {
        self.acceleration += rate;
    }
    pub fn simulate(&mut self, delta: f32) {
        if self.is_fixed || self.mass == 0.0 {
            return;
        }
        let friction = 0.95;
        self.velocity = self.position - self.previous_position;
        self.previous_position = self.position;
        self.position += self.velocity * friction + self.acceleration * delta * delta;
        // self.velocity = self.position - self.previous_position;
        // self.acceleration = Vec3::ZERO;

        // self.velocity = self.position - self.previous_position;
        // self.previous_position = self.position;
        // let friction = 0.98;
        // self.position += self.velocity * friction + self.acceleration * delta * delta;
        // self.velocity = self.position - self.previous_position;
        // self.acceleration = Vec3::ZERO;
        // self.velocity = self.position - self.previous_position;
    }
    pub fn apply_force(&mut self, force: Vec3) {
        if self.mass == 0.0 {
            return;
        }
        self.acceleration += force / self.mass;
        println!(
            "\tapply force: force {:?} mass {:?} new acceleration: {:?}",
            force, self.mass, self.acceleration
        );
    }
    pub fn apply_impulse(&mut self, impulse: Vec3) {
        if self.mass == 0.0 {
            return;
        }
        self.position += impulse / self.mass;
    }
    pub fn reset_forces(&mut self) {
        self.acceleration = Vec3::ZERO;
    }
    pub fn restrain(&mut self) {
        if self.position.y < 0.0 {
            let bounce = 0.95;
            self.position = self.position - 2.0 * self.position.dot(Vec3::Y) * Vec3::Y;
            self.velocity = self.velocity - 2.0 * self.velocity.dot(Vec3::Y) * Vec3::Y;
            self.previous_position = self.position - self.velocity * bounce;
        }
    }
}

#[derive(Component, Copy, Clone)]
pub struct Constraint {
    pub particle_a: Entity,
    pub particle_b: Entity,
    pub target: f32,
    pub stiffness: f32,
    pub damping: f32,
}

impl Constraint {
    pub fn new(particle_a: Entity, particle_b: Entity, target: f32) -> Self {
        Self {
            particle_a,
            particle_b,
            target,
            stiffness: 0.5,
            damping: 0.0,
        }
    }
    pub fn relax_old(&self, particle_a: &mut Particle, particle_b: &mut Particle) {
        let distance = particle_b.position - particle_a.position;
        // let distance_length = dista
        if particle_a.mass != 0.0 && particle_b.mass != 0.0 && distance.length() != self.target {
            let force = 0.5 * self.stiffness * (distance.length() - self.target) / self.target
                * distance.normalize();
            println!(
                "relaxing target {} distance {} force {:?} ",
                self.target,
                distance.length(),
                force
            );
            particle_a.apply_force(-force);
            particle_b.apply_force(force);
        }
    }

    pub fn relax(&self, particle_a: &mut Particle, particle_b: &mut Particle) {
        if particle_a.is_fixed && particle_b.is_fixed {
            return;
        }

        let center = (particle_a.position + particle_b.position) / 2.0;
        let direction = match (particle_b.position - particle_a.position).try_normalize() {
            None => {
                log::warn!("Failed handle stick between points {} and {} which are too close to each other", particle_a.position, particle_b.position);
                return;
            }
            Some(dir) => dir * self.target / 2.0,
        };
        // let slowing = 0.8;
        // println!(
        //     "\tcenter {:?} direction {:?} target {:?}",
        //     center, direction, self.target
        // );

        if particle_a.is_fixed {
            particle_b.position = particle_a.position + direction * 2.0;
        } else if particle_b.is_fixed {
            particle_a.position = particle_b.position - direction * 2.0;
        } else {
            particle_a.position = center - direction;
            particle_b.position = center + direction;
        }
    }

    
    pub fn relax_weighted(&self, particle_a: &mut Particle, particle_b: &mut Particle) {
        if particle_a.is_fixed && particle_b.is_fixed {
            return;
        }

        let ab_vector = particle_b.position - particle_a.position;
        let ab_offset = match (ab_vector).try_normalize() {
            None => {
                log::warn!("Failed handle stick between points {} and {} which are too close to each other", particle_a.position, particle_b.position);
                return;
            }
            Some(dir) => dir * (ab_vector.length() - self.target),
        };

        
        if particle_a.is_fixed {
            particle_b.position -= ab_offset;
        } else if particle_b.is_fixed {
            particle_a.position += ab_offset;
        } else {
            // Move more towards the first particle of the constraint. 
            //  This requires to set particler higher in the hierarchy (lower in the plant) as first
            let weight_a = 0.48;
            let weight_b = 1.0 - weight_a;
            particle_a.position += ab_offset * weight_a;
            particle_b.position -= ab_offset * weight_b;
        }
    }
}
