use bevy::prelude::Resource;
use crate::constraints::XPBDConstraint;

use super::SoftBody;

#[derive(Default, Resource)]
pub struct XPBDContext {
    bodies: Vec<SoftBody>,
}

impl XPBDContext {
    pub fn new() -> Self {
        Self { bodies: vec![] }
    }

    pub fn add_body(&mut self, body: SoftBody) {
        self.bodies.push(body);
    }

    pub fn get_bodies(&self) -> &Vec<SoftBody> {
        &self.bodies
    }

    pub fn get_bodies_mut(&mut self) -> &mut Vec<SoftBody> {
        &mut self.bodies
    }
}

#[derive(Default, Resource)]
pub struct XPBDContextRes {
    bodies: Vec<Box<dyn XPBDConstraint + Send + Sync>>,
}