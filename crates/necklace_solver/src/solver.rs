use std::f32::consts::PI;
use bevy::utils::petgraph::matrix_graph::Zero;
use bevy::{
    prelude::*,
    utils::hashbrown::{HashMap, HashSet},
};
use itertools::Itertools;
use rerun::{RecordingStream, Vec3D};

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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct VirtualPointKey(Entity, Entity);

impl VirtualPointKey {
    /// This point marks the initial position of the moving point
    fn start_point(&self) -> Entity {
        self.0
    }
    /// This point gets updated by the solver
    fn moving_point(&self) -> Entity {
        self.1
    }
}

impl From<(Entity, Entity)> for VirtualPointKey {
    fn from((a, b): (Entity, Entity)) -> Self {
        Self(a, b)
    }
}

#[derive(Debug, Clone, PartialEq)]
struct VirtualPoints(HashMap<VirtualPointKey, Point>);

impl VirtualPoints {
    fn new() -> Self {
        Self(HashMap::new())
    }
    fn add(&mut self, key: VirtualPointKey, point: Point) {
        self.0.insert(key, point);
    }
    fn get(&self, key: VirtualPointKey) -> Option<&Point> {
        self.0.get(&key)
    }
    fn get_mut(&mut self, key: VirtualPointKey) -> Option<&mut Point> {
        self.0.get_mut(&key)
    }
    /// Return a list of the moving points. This will merge all the instances of the same points by taking their average.
    fn merged_points(&self) -> HashMap<Entity, Vec3> {
        self.0
            .iter()
            .into_group_map_by(|(VirtualPointKey(_, point_id), _)| point_id)
            .iter()
            .map(|(point_id, virtual_points)| {
                let mut sum = Vec3::ZERO;
                for (_, point) in virtual_points {
                    sum += point.0;
                }
                (**point_id, sum / virtual_points.len() as f32)
            })
            .collect::<HashMap<_, _>>()
    }
}

type PointUpdate = (VirtualPointKey, Vec3);

struct UpdateAggregator {
    updates: HashMap<VirtualPointKey, Vec3>,
    count: HashMap<VirtualPointKey, usize>,
}

impl UpdateAggregator {
    fn new() -> Self {
        Self {
            updates: HashMap::new(),
            count: HashMap::new(),
        }
    }
    fn add(&mut self, update: PointUpdate) {
        let entry = self.updates.entry(update.0).or_insert(Vec3::ZERO);
        *entry += update.1;
        let entry = self.count.entry(update.0).or_insert(0);
        *entry += 1;
    }
    fn aggregate(&self) -> HashMap<VirtualPointKey, Vec3> {
        self.updates
            .iter()
            .map(|(key, update)| {
                let count = *self.count.get(key).unwrap();
                (*key, *update / (count as f32))
            })
            .collect()
    }
    fn apply(&self, virtual_points: &mut HashMap<VirtualPointKey, Point>) {
        for (entity, update) in self.aggregate() {
            let vp = virtual_points.get_mut(&entity).unwrap();
            // log::info!("update {:?}/ {:?} -> {:?}", entity, vp.0, update);
            vp.0 += update;
        }
    }
}

struct Reacher {
    origin: Entity,
    target: Entity,
    target_length: f32,
    rec: RecordingStream,
}

impl Reacher {
    pub fn new(bond: Bond, flip: bool, rec: RecordingStream) -> Self {
        Self {
            origin: if flip { bond.b } else { bond.a },
            target: if flip { bond.a } else { bond.b },
            target_length: bond.length,
            rec,
        }
    }
    pub fn step(
        &self,
        points: &Query<&mut Point>,
        virtual_points: &HashMap<VirtualPointKey, Point>,
        progress: f32,
    ) -> Option<PointUpdate> {
        let target = points.get(self.target).unwrap();
        let start = points.get(self.origin).unwrap();
        let virtual_point_key = (self.origin, self.target).into();
        let virtual_point = virtual_points.get(&virtual_point_key);
        let Some(virtual_point) = virtual_point.as_ref() else {
            warn!("virtual_point not found for {:?}", virtual_point_key);
            return None;
        };

        let new_travel = self.target_length * progress;
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

        // Vector from the virtual point to the target
        let to_start = start.0 - virtual_point.0;
        // Distance from the target
        let curr_distance = to_start.length();

        self.rec
            .log(
                format!("solver/reacher/{}", self.target.index()),
                &rerun::TextLog::new(format!(
                    "curr_distance: {:?}, curr_travel: {:?}, new_travel: {:?}",
                    curr_distance, curr_travel, new_travel
                )),
            )
            .ok();
        let step = new_travel - curr_travel;
        // let step = if curr_distance.is_zero() {
        //     new_travel - curr_travel
        // } else {
        //     // Angle between start->virtual_point and virtual_point->target
        //     let angle = (virtual_point.0 - start.0).angle_between(to_start);
        //     // If moving directly to the target
        //     if (angle.abs() - PI).abs() < 0.01 {
        //         new_travel - curr_travel
        //     } else {
        //         // How much do we have to move the virtual point in the direction of the target
        //         //  so the distance between the start and the virtual point is new_travel
        //         log::info!("> reacher: {:?}", self.target.index());
        //         log::info!("angle: {:?}", angle);
        //         log::info!("curr_distance: {:?}", curr_distance);
        //         log::info!("curr_travel: {:?}", curr_travel);
        //         log::info!("new_travel: {:?}", new_travel);
        //         self.rec
        //             .log(
        //                 format!("solver/reacher/{}", self.target.index()),
        //                 &rerun::TextLog::new(format!(
        //                     "angle: {:?}, curr_distance: {:?}, curr_travel: {:?}, new_travel: {:?}",
        //                     angle, curr_distance, curr_travel, new_travel
        //                 )),
        //             )
        //             .ok();
        //         curr_travel.powi(2) + curr_distance.powi(2)
        //             - 2.0 * curr_travel * curr_distance * angle.cos()
        //     }
        // };
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

        let origin = virtual_point.to_rr();
        let vector = Point(update).to_rr();

        self.rec
            .log(
                format!("solver/reacher/{}", self.target.index()),
                &rerun::Arrows3D::from_vectors(vec![vector])
                    .with_origins(vec![origin])
                    .with_colors(vec![rerun::Color::from_rgb(255, 128, 128)])
                    .with_radii([0.008]),
            )
            .ok();
        info!("self.target: {:?} update: {:?}", self.target, update);
        Some((virtual_point_key, update))
    }
}

struct Archer {
    pa: VirtualPointKey,
    pb: VirtualPointKey,
    target_length: f32,
    rec: RecordingStream,
}

impl Archer {
    pub fn new(
        pa: VirtualPointKey,
        pb: VirtualPointKey,
        target_length: f32,
        rec: RecordingStream,
    ) -> Self {
        Self {
            pa,
            pb,
            target_length,
            rec,
        }
    }

    pub fn log_key(&self, sub: &str) -> String {
        // Sort the indices so that the key is the same for both directions
        let mut indices = [
            self.pa.moving_point().index(),
            self.pb.moving_point().index(),
        ];
        indices.sort();
        format!("solver/archer/{:?}-{:?}/{}", indices[0], indices[1], sub)
    }
    pub fn step(
        &self,
        virtual_points: &HashMap<VirtualPointKey, Point>,
        progress: f32,
    ) -> Option<(PointUpdate, PointUpdate)> {
        let (Some(pa), Some(pb)) = (virtual_points.get(&self.pa), virtual_points.get(&self.pb))
            else {
                panic!(
                    "virtual_points not found pa:{:?}, pb:{:?}",
                    self.pa, self.pb
                );
            };
        let distance = pa.distance(pb.0);
        let target_distance = self.target_length * progress;
        let displacement = target_distance - distance;
        log::info!("{}: {:?}", self.log_key("distance"), distance);
        log::info!("{}: {:?}", self.log_key("target_distance"), target_distance);
        log::info!("{}: {:?}", self.log_key("displacement"), displacement);
        let direction = (pb.0 - pa.0).normalize();
        if direction.is_nan() {
            return None;
        }
        let update = direction * displacement * 0.5;

        self.rec
            .log(
                self.log_key("update"),
                &rerun::Arrows3D::from_vectors(vec![
                    rerun::Vec3D::from(update.to_array()),
                    rerun::Vec3D::from((-update).to_array()),
                ])
                    .with_origins(vec![pa.to_rr(), pb.to_rr()])
                    .with_colors(vec![rerun::Color::from_rgb(128, 128, 255)])
                    .with_radii([0.006]),
            )
            .ok();

        Some(((self.pa, update), (self.pb, -update)))
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
    let substeps = 12;

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
        let mut virtual_points = HashMap::<VirtualPointKey, Point>::new();
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
                        reachers.push(Reacher::new(bond.1.clone(), reached_b, rec.clone()));
                        // set the virtual point (the non-reached point) to the reached point of the Bond
                        if reached_a {
                            virtual_points.insert(
                                (bond.1.a, bond.1.b).into(),
                                points.get(bond.1.a).unwrap().clone(),
                            );
                        } else {
                            virtual_points.insert(
                                (bond.1.b, bond.1.a).into(),
                                points.get(bond.1.b).unwrap().clone(),
                            );
                        }
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

        // If two reachers are targeting the opposite ends of the same bond and starting
        //  from the same point, create an archer
        for bond in &bonds {
            for reacher_a in &reachers {
                for reacher_b in &reachers {
                    if reacher_a.origin == reacher_b.origin
                        && reacher_a.target != reacher_b.target
                        && bond.1.has_point(reacher_a.target)
                        && bond.1.has_point(reacher_b.target)
                    {
                        archers.push(Archer::new(
                            (reacher_a.origin, reacher_a.target).into(),
                            (reacher_b.origin, reacher_b.target).into(),
                            bond.1.length,
                            rec.clone(),
                        ));
                    }
                }
            }
        }

        // relax bonds between prev_ring and next_ring
        for substep in 0..substeps {
            rec.set_time_seconds("bevy_time", time.step());
            rec.log("solver/substep", &rerun::Scalar::new(substep as f64))
                .ok();
            let step_progress = (substep + 1) as f32 / substeps as f32;
            println!("\n\nstep_progress: {:?}", step_progress);
            let mut updates = UpdateAggregator::new();
            for reacher in &mut reachers {
                if let Some(update) = reacher.step(&points, &virtual_points, step_progress) {
                    rec.log(
                        format!("solver/add_update/{:?}", update.0),
                        &rerun::Scalar::new(update.1.length() as f64),
                    )
                        .ok();
                    updates.add(update);
                }
            }
            updates.apply(&mut virtual_points);

            let mut updates = UpdateAggregator::new();
            for archer in &mut archers {
                if let Some((update1, update2)) = archer.step(&virtual_points, step_progress) {
                    updates.add(update1);
                    updates.add(update2);
                }
            }
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
        for (entity, point) in VirtualPoints(virtual_points).merged_points() {
            points.get_mut(entity).unwrap().0 = point;
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
