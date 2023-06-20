use bevy::prelude::*;

#[derive(Default)]
pub struct Orientation {
    pub quaternion: Quat,
    pub old_quaternion: Quat,
    pub rest_quaternion: Quat,
    pub angular_velocity: Vec3,
    pub inverse_mass: f32,
}

impl Orientation {
    pub fn from_quaternion(quaternion: Quat) -> Self {
        Self {
            quaternion,
            rest_quaternion: quaternion,
            old_quaternion: quaternion,
            inverse_mass: 1.0,
            ..default()
        }
    }
}