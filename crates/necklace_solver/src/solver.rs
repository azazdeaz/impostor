use bevy::{
    prelude::*,
    utils::hashbrown::{HashMap, HashSet},
};

use crate::{Bond, Point};

pub fn relax_bonds(mut points: Query<&mut Point>, bonds: Query<&Bond>) {
    for bond in &bonds {
        if let Ok([mut pa, mut pb]) = points.get_many_mut([bond.a, bond.b]) {
            let distance = pa.distance(pb.0);
            let displacement = distance - bond.length;
            let direction = (pb.0 - pa.0).normalize();
            pa.0 += direction * displacement * 0.5;
            pb.0 -= direction * displacement * 0.5;
        }
    }
}

pub fn grapgh_relax_bonds(mut points: Query<&mut Point>, bonds: Query<&Bond>) {
    // find the most stressed point
    let mut stresses = HashMap::new();
    for bond in &bonds {
        if let Ok([pa, pb]) = points.get_many([bond.a, bond.b]) {
            let distance = pa.distance(pb.0);
            let displacement = distance - bond.length;
            let stress = displacement.abs() / bond.length;
            stresses.entry(bond.a).or_insert(Vec::new()).push(stress);
            stresses.entry(bond.b).or_insert(Vec::new()).push(stress);
        }
    }
    let stresses = stresses
        .into_iter()
        .map(|(k, v)| (k, v.iter().sum::<f32>() / v.len() as f32))
        .collect::<HashMap<_, _>>();

    let Some(most_stressed) = stresses
        .iter()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
        .map(|(k, _)| *k)
    else {
        return;
    };

    let mut visiteds = HashSet::new();
    let mut prev_ring = vec![most_stressed];

    while prev_ring.len() > 0 {
        let mut next_ring = Vec::new();
        for bond in &bonds {
            if prev_ring.contains(&bond.a) && !visiteds.contains(&bond.b) {
                next_ring.push(bond.b);
                visiteds.insert(bond.b);
            }
            if prev_ring.contains(&bond.b) && !visiteds.contains(&bond.a) {
                next_ring.push(bond.a);
                visiteds.insert(bond.a);
            }
        }
        // relax bonds between prev_ring and next_ring
        for bond in &bonds {
            let (prev, next) = if prev_ring.contains(&bond.a) && next_ring.contains(&bond.b) {
                (bond.a, bond.b)
            } else if prev_ring.contains(&bond.b) && next_ring.contains(&bond.a) {
                (bond.b, bond.a)
            } else {
                continue;
            };
            if let Ok([pa, mut pb]) = points.get_many_mut([prev, next]) {
                let distance = pa.distance(pb.0);
                let displacement = distance - bond.length;
                let direction = (pb.0 - pa.0).normalize();
                pb.0 -= direction * displacement;
            }
        }
        prev_ring = next_ring;
    }

    for bond in &bonds {
        if let Ok([mut pa, mut pb]) = points.get_many_mut([bond.a, bond.b]) {
            let distance = pa.distance(pb.0);
            let displacement = distance - bond.length;
            let direction = (pb.0 - pa.0).normalize();
            pa.0 += direction * displacement * 0.5;
            pb.0 -= direction * displacement * 0.5;
        }
    }
}
