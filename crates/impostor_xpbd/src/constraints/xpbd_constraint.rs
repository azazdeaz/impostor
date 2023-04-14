use crate::structs::Particle;

pub trait XPBDConstraint {
    fn solve(&self, particles: &mut Vec<Particle>, delta_squared: f32);
    fn get_compliance(&self) -> f32;
}