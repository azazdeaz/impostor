use std::ops::Deref;

use bevy::prelude::*;

#[derive(Component, Clone, Copy, Debug, PartialEq)]
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
impl Point {
    pub fn to_rr(self) -> rerun::Vec3D {
        rerun::Vec3D::from(self.0.to_array())
    }
}

#[derive(Component, Clone, Copy, Debug)]
pub struct Bond {
    pub a: Entity,
    pub b: Entity,
    pub length: f32,
    pub compliance: f32,
}

impl Bond {
    pub fn new(a: Entity, b: Entity, length: f32, compliance: f32) -> Self {
        Self {
            a,
            b,
            length,
            compliance,
        }
    }
    pub fn get_from_to(&self, from_points: &Vec<Entity>) -> (Entity, Entity) {
        if from_points.contains(&self.a) {
            (self.a, self.b)
        } else {
            assert!(from_points.contains(&self.b)); // the bond should be between the two points
            (self.b, self.a)
        }
    }
}

#[derive(Component)]
pub struct StressLevel(pub f32);