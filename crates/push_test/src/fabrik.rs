use bevy::{prelude::*, utils::HashMap};

use crate::SegmentData;

pub struct FabrikComputer {
    pub global_xyz: HashMap<Entity, Vec3>,
    pub segments: HashMap<Entity, SegmentData>,
    // contacts:
}

pub enum FabrikDirection {
    Forward,
    Backward,
}

impl FabrikComputer {
    pub fn iterate_half(&mut self, from: Entity, to: Entity, direction: FabrikDirection) {
        let mut from = from;
        while from != to {
            let from_segment = self.segments[&from];
            let from_xyz = self.global_xyz[&from];
            let next = match direction {
                FabrikDirection::Forward => from_segment.forward.unwrap(),
                FabrikDirection::Backward => from_segment.backward.unwrap(),
            };
            let next_segment = self.segments[&next];
            let next_xyz = self.global_xyz[&next];
            let length = match direction {
                FabrikDirection::Forward => from_segment.length,
                FabrikDirection::Backward => next_segment.length,
            };
            let from_to_next = next_xyz - from_xyz;
            self.global_xyz
                .insert(next, from_xyz + (from_to_next.normalize() * length));

            from = next;
        }
    }

    pub fn iterate(&mut self, start: Entity, end: Entity) {
        let start_xyz = self.global_xyz[&start];
        self.iterate_half(end, start, FabrikDirection::Backward);
        // recover start joint position befor iterating forward
        self.global_xyz.insert(start, start_xyz);
        self.iterate_half(start, end, FabrikDirection::Forward);
    }
}