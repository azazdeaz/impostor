use std::ops::Deref;

use bevy::prelude::*;

use crate::{Bond, Point, StressLevel};

pub fn draw_points(mut gizmos: Gizmos, points: Query<(&Point, Option<&StressLevel>)>) {
    for (point, stress) in points.iter() {
        let color = colorgrad::viridis().repeat_at(stress.map(|s| s.0 as f64).unwrap_or(0.0));
        gizmos.sphere(
            **point,
            Quat::IDENTITY,
            0.1,
            Color::rgba(
                color.r as f32,
                color.g as f32,
                color.b as f32,
                color.a as f32,
            ),
        );
    }
}

pub fn draw_bonds(mut gizmos: Gizmos, bonds: Query<&Bond>, points: Query<&Point>) {
    for bond in bonds.iter() {
        if let (Ok(a), Ok(b)) = (points.get(bond.a), points.get(bond.b)) {
            gizmos.line(**a, **b, Color::WHITE);
        }
    }
}

#[derive(Resource)]
pub struct Rec(pub rerun::RecordingStream);

impl Deref for Rec {
    type Target = rerun::RecordingStream;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[derive(Resource, Debug, Default)]
pub struct RecTime(pub f64);
impl Deref for RecTime {
    type Target = f64;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl RecTime {
    pub fn set(&mut self, time: f64) {
        self.0 = time;
    }
    pub fn step(&mut self) -> f64 {
        self.0 += 0.1;
        self.0
    }
}
