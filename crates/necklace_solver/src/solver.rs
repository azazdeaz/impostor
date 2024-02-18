use bevy::{
    prelude::*,
    utils::hashbrown::{HashMap, HashSet},
};
use itertools::Itertools;
use rerun::Vec3D;

use crate::{Bond, Point, Rec, RecTime, StressLevel, TetFrame};

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
    bonds: Query<(Entity, &Bond)>,
    tets: Query<(Entity, &TetFrame)>,
    rec: ResMut<Rec>,
    mut time: ResMut<RecTime>,
) {
    // find the most stressed point
    let mut stresses = HashMap::new();
    for (bond_id, bond) in &bonds {
        if let Ok([pa, pb]) = points.get_many([bond.a, bond.b]) {
            rec.log(
                format!("bonds/all/{}", bond_id.index()),
                &rerun::LineStrips3D::new(vec![vec![pa.to_rr(), pb.to_rr()]])
                    .with_colors([rerun::Color::from_rgb(128, 128, 128)]),
            )
            .ok();
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
        info!("No points with stress > {}", min_stress);
        return;
    };

    // All the point ids that have been visited by the solver
    let mut reached = HashSet::new();
    reached.insert(most_stressed);
    let substeps = 12;

    loop {
        rec.set_time_seconds("bevy_time", time.step());
        rec.log("reacher", &rerun::Clear::recursive()).ok();
        rec.log("archer", &rerun::Clear::recursive()).ok();
        rec.log("closer", &rerun::Clear::recursive()).ok();
        // All the tetrahedra that contain a point in the prev_ring and have unreached points
        let next_tets = tets
            .iter()
            .filter(|(_, tet)| {
                // has solved points
                (reached.contains(&tet.a)
                    || reached.contains(&tet.b)
                    || reached.contains(&tet.c)
                    || reached.contains(&tet.d))
                // has unreached points
                && (!reached.contains(&tet.a)
                    || !reached.contains(&tet.b)
                    || !reached.contains(&tet.c)
                    || !reached.contains(&tet.d))
            })
            .collect::<Vec<_>>();

        if next_tets.len() == 0 {
            break;
        }
        info!("next_tets: {:?}", next_tets.len());

        let reacher = |bond_id: Entity, bond: Bond, flip: bool| {
            {
                let rec = rec.clone();
                move |points: &Query<&mut Point>,
                      virtual_points: &mut HashMap<(Entity, Entity), Point>,
                      progress: f32| {
                    let (target_id, start_id) = if !flip {
                        (bond.b, bond.a)
                    } else {
                        (bond.a, bond.b)
                    };
                    let target = points.get(target_id).unwrap();
                    let start = points.get(start_id).unwrap();
                    let mut virtual_point = virtual_points.get_mut(&(bond_id, target_id));
                    let Some(virtual_point) = virtual_point.as_mut() else {
                        warn!("virtual_point not found for bond {:?}", bond);
                        return;
                    };

                    let new_travel = bond.length * progress;
                    let curr_travel = virtual_point.0.distance(start.0);
                    if curr_travel.is_infinite() {
                        panic!(
                            "distance between {:?} and {:?} is inf{:?}",
                            virtual_point.0,
                            target.0,
                            virtual_point.0 - target.0
                        );
                    }
                    info!(
                        "new_travel: {:?} curr_travel: {:?}",
                        new_travel, curr_travel
                    );
                    let step = new_travel - curr_travel;
                    // pointing from the virtual point to the target
                    info!(
                        "virtual_point: {:?} target: {:?}",
                        virtual_point.0, target.0
                    );
                    let direction = (target.0 - virtual_point.0).normalize();
                    if direction.is_nan() {
                        warn!("direction is NaN");
                        return;
                    }
                    info!("step: {:?} direction: {:?}", step, direction);
                    virtual_point.0 += direction * step;
                    if virtual_point.0.is_nan() {
                        panic!("virtual_point is NaN");
                    }

                    let origin = start.to_rr();
                    let vector = Point(virtual_point.0 - start.0).to_rr();
                    rec.log(
                        format!("reacher/{}_{}", bond_id.index(), target_id.index()),
                        &rerun::Arrows3D::from_vectors(vec![vector])
                            .with_origins(vec![origin])
                            .with_colors(vec![rerun::Color::from_rgb(255, 128, 128)])
                            .with_radii([0.012]),
                    )
                    .ok();
                }
            }
        };
        let archer = |bond_id: Entity, bond: Bond| {
            let rec = rec.clone();
            move |virtual_points: &mut HashMap<(Entity, Entity), Point>, progress: f32| {
                info!("archer: {:?}", bond_id.index());
                let remaining_distance = bond.length * (1.0 - progress);
                let Some([pa, pb]) =
                    virtual_points.get_many_mut([&(bond_id, bond.a), &(bond_id, bond.b)])
                else {
                    warn!("virtual_points not found for bond {:?}", bond);
                    return;
                };
                let distance = pa.distance(pb.0);
                let displacement = distance - remaining_distance;
                let direction = (pb.0 - pa.0).normalize();
                let update = direction * displacement * 0.5;

                rec.log(
                    format!("archer/{}", bond_id.index()),
                    &rerun::Arrows3D::from_vectors(vec![
                        rerun::Vec3D::from(update.to_array()),
                        rerun::Vec3D::from((-update).to_array()),
                    ])
                    .with_origins(vec![pa.to_rr(), pb.to_rr()])
                    .with_colors(vec![rerun::Color::from_rgb(128, 128, 255)])
                    .with_radii([0.012]),
                )
                .ok();

                pa.0 += update;
                pb.0 -= update;
            }
        };

        let mut reachers = Vec::new();
        let mut archers = Vec::new();

        // Create virtual points. For each point in next_ring, create a point on the prev ring
        //  by cloning the other side of the forward bond
        let mut virtual_points = HashMap::new();

        let mut newly_reached = HashSet::new();
        for tet in &next_tets {
            for bond in &bonds {
                if tet.1.has_bond(bond.1) {
                    let reached_a = reached.contains(&bond.1.a);
                    let reached_b = reached.contains(&bond.1.b);
                    // Skip bonds that are already solved
                    if reached_a && reached_b {
                        continue;
                    } else if reached_a != reached_b {
                        reachers.push(reacher(bond.0, bond.1.clone(), reached_b));
                        // set the virtual point (the non-reached point) to the reached point of the Bond
                        if reached_a {
                            virtual_points
                                .insert((bond.0, bond.1.b), points.get(bond.1.a).unwrap().clone());
                        } else {
                            virtual_points
                                .insert((bond.0, bond.1.a), points.get(bond.1.b).unwrap().clone());
                        }
                    } else {
                        archers.push(archer(bond.0, bond.1.clone()));
                    }

                    // Update reacheds
                    if !reached_a {
                        newly_reached.insert(bond.1.a);
                    }
                    if !reached_b {
                        newly_reached.insert(bond.1.b);
                    }
                }
            }
        }
        reached.extend(newly_reached);

        // relax bonds between prev_ring and next_ring
        for substep in 0..substeps {
            rec.set_time_seconds("bevy_time", time.step());
            let step_progress = (substep + 1) as f32 / substeps as f32;
            println!("\n\nstep_progress: {:?}", step_progress);
            for reacher in &mut reachers {
                reacher(&points, &mut virtual_points, step_progress);
            }
            for archer in &mut archers {
                archer(&mut virtual_points, step_progress);
            }
            let mut points = Vec::new();
            let mut colors = Vec::new();
            // let mut vectors = Vec::new();
            for (_, point) in &virtual_points {
                points.push(point.to_rr());
                colors.push(rerun::Color::from_rgb(255, 0, 0));
            }
            rec.log(
                "virtual_points",
                &rerun::Points3D::new(points)
                    .with_colors(colors)
                    .with_radii([0.008]),
            )
            .ok();
        }
        // Write the virtual points back to the real points
        let aggregated_virtual_points = virtual_points
            .iter()
            .into_group_map_by(|((_, point_id), _)| point_id)
            .iter()
            .map(|(point_id, virtual_points)| {
                let mut sum = Vec3::ZERO;
                for (_, point) in virtual_points {
                    sum += point.0;
                }
                (**point_id, sum / virtual_points.len() as f32)
            })
            .collect::<HashMap<_, _>>();
        for (entity, point) in &aggregated_virtual_points {
            println!("entity: {:?} {:?}", entity, point);
            println!(
                " updating: {:?} -> {:?}",
                points.get_mut(*entity).unwrap().0,
                point
            );
            points.get_mut(*entity).unwrap().0 = *point;
        }
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
