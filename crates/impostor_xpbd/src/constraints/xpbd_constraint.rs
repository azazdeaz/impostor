use bevy_prototype_debug_lines::DebugShapes;

use crate::structs::{Particle, SoftBodyData};

pub trait XPBDConstraint {
    fn solve(&self, body: &mut SoftBodyData, delta_squared: f32);
    fn get_compliance(&self) -> f32;
    fn debug_draw(&self, body: &SoftBodyData, shapes: &mut DebugShapes);
}