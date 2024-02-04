use bevy::prelude::*;

use crate::Point;

pub fn draw_points(mut gizmos: Gizmos, points: Query<&Point>) {
    for point in points.iter() {
        gizmos.sphere(**point, Quat::IDENTITY, 0.1, Color::RED);
    }
}