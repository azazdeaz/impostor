use bevy::{prelude::*};

#[derive(Component)]
pub struct PlantBase {}

#[derive(Component, Clone, Copy)]
pub struct SegmentData {
    pub collider: Entity,
    pub forward: Option<Entity>,
    pub backward: Option<Entity>,
    pub length: f32,
    pub previous_rotation: Quat,
}
