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

type PointUpdate = (Entity, Vec3);
struct UpdateAggregator {
    updates: HashMap<Entity, Vec3>,
    count: HashMap<Entity, usize>,
}
impl UpdateAggregator {
    fn new() -> Self {
        Self {
            updates: HashMap::new(),
            count: HashMap::new(),
        }
    }
    fn add(&mut self, update: PointUpdate) {
        log::info!("update: {:?}", update);
        let entry = self.updates.entry(update.0).or_insert(Vec3::ZERO);
        *entry += update.1;
        let entry = self.count.entry(update.0).or_insert(0);
        *entry += 1;
    }
    fn aggregate(&self) -> HashMap<Entity, Vec3> {
        self.updates
            .iter()
            .map(|(entity, update)| {
                let count = *self.count.get(entity).unwrap();
                (*entity, *update / (count as f32))
            })
            .collect()
    }
    fn apply(&self, virtual_points: &mut HashMap<Entity, Point>) {
        for (entity, update) in self.aggregate() {
            let vp = virtual_points.get_mut(&entity).unwrap();
            log::info!("update {:?}/ {:?} -> {:?}", entity, vp.0, update);
            vp.0 += update;
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
    rec.set_time_seconds("bevy_time", time.step());
    // find the most stressed point
    let mut stresses = HashMap::new();
    for (_, bond) in &bonds {
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
        info!("No points with stress > {}", min_stress);
        return;
    };

    // All the point ids that have been visited by the solver
    let mut reached = HashSet::new();
    reached.insert(most_stressed);
    let substeps = 1;

    let reacher = |bond: Bond, flip: bool| {
        {
            let rec = rec.clone();
            move |points: &Query<&mut Point>,
                  virtual_points: &HashMap<Entity, Point>,
                  progress: f32|
                  -> Option<PointUpdate> {
                let (target_id, start_id) = if !flip {
                    (bond.b, bond.a)
                } else {
                    (bond.a, bond.b)
                };
                let target = points.get(target_id).unwrap();
                let start = points.get(start_id).unwrap();
                let virtual_point = virtual_points.get(&target_id);
                let Some(virtual_point) = virtual_point.as_ref() else {
                    warn!("virtual_point not found for bond {:?}", bond);
                    return None;
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
                    return None;
                }
                info!("step: {:?} direction: {:?}", step, direction);
                let update = direction * step;
                if update.is_nan() {
                    panic!("update is NaN");
                }

                let origin = start.to_rr();
                let vector = Point(virtual_point.0 + update - start.0).to_rr();
                rec.log(
                    format!("solver/reacher/{}", target_id.index()),
                    &rerun::Arrows3D::from_vectors(vec![vector])
                        .with_origins(vec![origin])
                        .with_colors(vec![rerun::Color::from_rgb(255, 128, 128)])
                        .with_radii([0.008]),
                )
                .ok();
                info!("target_id: {:?} update: {:?}", target_id, update);
                Some((target_id, update))
            }
        }
    };
    let archer = |bond: Bond| {
        let rec = rec.clone();
        move |virtual_points: &HashMap<Entity, Point>,
              progress: f32|
              -> Option<(PointUpdate, PointUpdate)> {
            let remaining_distance = bond.length * (1.0 - progress);
            let (Some(pa), Some(pb)) = (virtual_points.get(&bond.a), virtual_points.get(&bond.b))
            else {
                panic!("virtual_points not found for bond {:?}", bond);
            };
            let distance = pa.distance(pb.0);
            let displacement = distance - remaining_distance;
            let direction = (pb.0 - pa.0).normalize();
            let update = direction * displacement * 0.5;

            rec.log(
                format!("solver/archer/{}-{}", bond.a.index(), bond.b.index()),
                &rerun::Arrows3D::from_vectors(vec![
                    rerun::Vec3D::from(update.to_array()),
                    rerun::Vec3D::from((-update).to_array()),
                ])
                .with_origins(vec![pa.to_rr(), pb.to_rr()])
                .with_colors(vec![rerun::Color::from_rgb(128, 128, 255)])
                .with_radii([0.012]),
            )
            .ok();

            Some(((bond.a, update), (bond.b, -update)))
        }
    };

    loop {
        rec.set_time_seconds("bevy_time", time.step());
        rec.log("solver/reacher", &rerun::Clear::recursive()).ok();
        rec.log("solver/archer", &rerun::Clear::recursive()).ok();
        rec.log("solver/next_tets", &rerun::Clear::recursive()).ok();

        for (bond_id, bond) in &bonds {
            if let Ok([pa, pb]) = points.get_many([bond.a, bond.b]) {
                rec.log(
                    format!("solver/bonds/all/{}", bond_id.index()),
                    &rerun::LineStrips3D::new(vec![vec![pa.to_rr(), pb.to_rr()]])
                        .with_colors([rerun::Color::from([128, 128, 128, 128])]),
                )
                .ok();
            }
        }
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

        for tet in next_tets.iter() {
            for bond in &bonds {
                if tet.1.has_bond(bond.1) {
                    let pa = points.get(bond.1.a).unwrap();
                    let pb = points.get(bond.1.b).unwrap();
                    rec.log(
                        format!("solver/next_tets/{}", bond.0.index()),
                        &rerun::LineStrips3D::new(vec![vec![pa.to_rr(), pb.to_rr()]])
                            .with_colors([rerun::Color::from([64, 64, 255, 255])]),
                    )
                    .ok();
                }
            }
        }

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
                        reachers.push(reacher(bond.1.clone(), reached_b));
                        // set the virtual point (the non-reached point) to the reached point of the Bond
                        if reached_a {
                            virtual_points.insert(bond.1.b, points.get(bond.1.a).unwrap().clone());
                        } else {
                            virtual_points.insert(bond.1.a, points.get(bond.1.b).unwrap().clone());
                        }
                    } else {
                        archers.push(archer(bond.1.clone()));
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
            let mut updates = UpdateAggregator::new();
            for reacher in &mut reachers {
                if let Some(update) = reacher(&points, &virtual_points, step_progress) {
                    rec.log(
                        format!("solver/add_update/{:?}", update.0.index()),
                        &rerun::Scalar::new(update.1.length() as f64),
                    )
                    .ok();
                    updates.add(update);
                }
            }
            // for archer in &mut archers {
            //     if let Some((update1, update2)) = archer(&virtual_points, step_progress) {
            //         updates.add(update1);
            //         updates.add(update2);
            //     }
            // }

            updates.apply(&mut virtual_points);

            let mut points = Vec::new();
            let mut colors = Vec::new();
            // let mut vectors = Vec::new();
            for (_, point) in &virtual_points {
                points.push(point.to_rr());
                colors.push(rerun::Color::from([255, 0, 0, 128]));
            }
            rec.log(
                "solver/virtual_points",
                &rerun::Points3D::new(points)
                    .with_colors(colors)
                    .with_radii([0.008]),
            )
            .ok();
        }

        // Write updates back to the points
        for (entity, point) in &virtual_points {
            points.get_mut(*entity).unwrap().0 = point.0;
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
