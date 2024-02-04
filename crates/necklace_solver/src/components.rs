use std::ops::Deref;

use bevy::prelude::*;

#[derive(Component, Clone, Copy)]
pub struct Point(pub Vec3);
impl Deref for Point {
    type Target = Vec3;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl From<Vec3> for Point {
    fn from(v: Vec3) -> Self {
        Self(v)
    }
}

#[derive(Component, Clone, Copy)]
pub struct Bond {
    pub a: Entity,
    pub b: Entity,
    pub length: f32,
    pub compliance: f32,
}