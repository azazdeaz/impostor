use bevy::prelude::*;

use crate::{Bond, Point};

pub fn relax_bonds(
    mut points: Query<&mut Point>,
    bonds: Query<&Bond>,
    time: Res<Time>,
) {
    let dt = time.delta_seconds();
    for bond  in &bonds {
        for [mut pa, mut pb] in points.get_many_mut([bond.a, bond.b]) {
            let distance = pa.distance(pb.0);
            let displacement = distance - bond.length;
            let direction = (pb.0 - pa.0).normalize();
            pa.0 += direction * displacement * 0.5;
            pb.0 -= direction * displacement * 0.5;


        }
    }
}