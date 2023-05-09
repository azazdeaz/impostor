use bevy::prelude::*;

#[derive(Default)]
pub struct Orientation {
    pub quaternion: Quat,
    pub angular_velocity: Vec3,
    pub inverse_mass: f32,
}

impl Orientation {
    pub fn from_quaternion(quaternion: Quat) -> Self {
        Self {
            quaternion,
            inverse_mass: 1.0,
            ..default()
        }
    }
}