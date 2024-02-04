use bevy::prelude::*;

use crate::{Bond, Point};

pub fn draw_points(mut gizmos: Gizmos, points: Query<&Point>) {
    for point in points.iter() {
        gizmos.sphere(**point, Quat::IDENTITY, 0.1, Color::RED);
    }
}

pub fn draw_bonds(mut gizmos: Gizmos, bonds: Query<&Bond>, points: Query<&Point>) {
    for bond in bonds.iter() {
        if let (Ok(a), Ok(b)) = (points.get(bond.a), points.get(bond.b)) {
            gizmos.line(**a, **b, Color::WHITE);
        }
    }
}