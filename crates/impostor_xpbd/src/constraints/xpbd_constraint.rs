use bevy_prototype_debug_lines::DebugShapes;

use crate::structs::Particle;

pub trait XPBDConstraint {
    fn solve(&self, particles: &mut Vec<Particle>, delta_squared: f32);
    fn get_compliance(&self) -> f32;
    fn debug_draw(&self, particles: &Vec<Particle>, shapes: &mut DebugShapes);
}