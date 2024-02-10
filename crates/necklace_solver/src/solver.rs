use bevy::{
    prelude::*,
    utils::hashbrown::{HashMap, HashSet},
};

use crate::{Bond, Point, StressLevel};

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

pub fn graph_relax_bonds(
    mut commands: Commands,
    mut points: Query<&mut Point>,
    bonds: Query<&Bond>,
) {
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

    // Update stress levels
    for (entity, stress) in &stresses {
        commands.entity(*entity).insert(StressLevel(*stress));
    }

    let min_stress = 0.1;
    let Some(most_stressed) = stresses
        .iter()
        .filter(|(_, stress)| **stress > min_stress)
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(k, _)| *k)
    else {
        return;
    };

    let mut visiteds = HashSet::new();
    let mut prev_ring = vec![most_stressed];
    let substeps = 12;

    while prev_ring.len() > 0 {
        // find the next ring
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

        // all the bonds connecting prev_ring to next_ring
        let reach_bonds = bonds
            .iter()
            .filter(|bond| prev_ring.contains(&bond.a) && !prev_ring.contains(&bond.b))
            .collect::<Vec<_>>();
        // all the bonds connecting points in the next_ring
        let arch_bonds = bonds
            .iter()
            .filter(|bond| next_ring.contains(&bond.a) && next_ring.contains(&bond.b))
            .collect::<Vec<_>>();

        // Create virtual points. For each point in next_ring, create a point on the prev ring
        //  by cloning the other side of the forward bond
        let mut virtual_points = HashMap::new();
        for bond in &reach_bonds {
            let (prev, next) = bond.get_from_to(&prev_ring);
            let point = points.get(prev).unwrap();
            virtual_points.insert(next, point.clone());
        }

        // relax bonds between prev_ring and next_ring
        for substep in 0..substeps {
            let step_progress = (substep + 1) as f32 / substeps as f32;
            println!("\n\nstep_progress: {:?}", step_progress);
            for bond in &reach_bonds {
                println!("\nbond: {:?}", bond);
                let (_, next) = bond.get_from_to(&prev_ring);
                let virtual_point = virtual_points.get_mut(&next).unwrap();
                let target_point = points.get(next).unwrap();
                println!(
                    "virtual_point>: {:?} target_point: {:?}",
                    virtual_point.0, target_point.0
                );

                let distance = virtual_point.distance(target_point.0);
                if distance < 0.0001 {
                    continue;
                }
                let direction = (target_point.0 - virtual_point.0).normalize();
                println!(
                    "distance: {:?} direction: {:?}",
                    distance, direction
                );
                println!("change: {:?}", direction * bond.length * step_progress);
                virtual_point.0 += direction * bond.length * step_progress;
                println!("virtual_point<: {:?}", virtual_point.0);
            }

            for bond in &arch_bonds {
                let (pa_id, pb_id) = bond.get_from_to(&next_ring);
                let Some([pa, pb]) = virtual_points.get_many_mut([&pa_id, &pb_id]) else {
                    continue;
                };
                let distance = pa.distance(pb.0);
                let displacement = distance - bond.length;
                let direction = (pb.0 - pa.0).normalize();
                pa.0 += direction * displacement * 0.5 * step_progress;
                pb.0 -= direction * displacement * 0.5 * step_progress;
            }
        }
        // Write the virtual points back to the real points
        for (entity, point) in &virtual_points {
            println!("entity: {:?} {:?}", entity, point);
            println!(
                " updating: {:?} -> {:?}",
                points.get_mut(*entity).unwrap().0,
                point.0
            );
            points.get_mut(*entity).unwrap().0 = point.0;
        }
        prev_ring = next_ring;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grapgh_relax_bonds() {
        let mut app = App::new();
        app.add_systems(Update, graph_relax_bonds);
        // Create a triangle
        let pa = Point(Vec3::new(0.0, 0.0, 0.0));
        let pb = Point(Vec3::new(1.0, 0.0, 0.0));
        let pc = Point(Vec3::new(0.5, 1.0, 0.0));
        let pa_id = app.world.spawn(pa).id();
        let pb_id = app.world.spawn(pb).id();
        let pc_id = app.world.spawn(pc).id();
        let bond_ab = Bond::new(pa_id, pb_id, pa.distance(pb.0), 0.1);
        let bond_bc = Bond::new(pb_id, pc_id, pb.distance(pc.0), 0.1);
        let bond_ca = Bond::new(pc_id, pa_id, pc.distance(pa.0), 0.1);
        app.world.spawn(bond_ab);
        app.world.spawn(bond_bc);
        app.world.spawn(bond_ca);

        app.update();

        assert_eq!(pa, *app.world.get::<Point>(pa_id).unwrap());
        assert_eq!(pb, *app.world.get::<Point>(pb_id).unwrap());
        assert_eq!(pc, *app.world.get::<Point>(pc_id).unwrap());
    }
}
