use bevy::prelude::*;

#[derive(Default)]
pub struct Particle {
    pub position: Vec3,
    pub prev_position: Vec3,
    pub velocity: Vec3,
    pub inverse_mass: f32,
}

impl Particle {
    pub fn from_position(position: Vec3) -> Self {
        Self {
            position,
            prev_position: position,
            inverse_mass: 0.0,
            ..default()
        }
    }
}