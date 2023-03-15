use bevy::{
    log,
    prelude::*,
    utils::{HashMap, HashSet},
};
use std::collections::VecDeque;

#[derive(Eq, Hash, PartialEq, Clone, Copy, Component)]
pub struct PointId(pub usize);
pub struct PointIdGen {
    last: usize,
}
impl PointIdGen {
    fn new() -> Self {
        Self { last: 0 }
    }
    fn next(&mut self) -> PointId {
        self.last += 1;
        PointId(self.last)
    }
}

#[derive(Eq, Hash, PartialEq, Clone, Copy)]
pub struct StickId(pub usize);
pub struct StickIdGen {
    last: usize,
}
impl StickIdGen {
    fn new() -> Self {
        Self { last: 0 }
    }
    fn next(&mut self) -> StickId {
        self.last += 1;
        StickId(self.last)
    }
}

pub struct Point {
    pub previous_position: Vec3,
    pub position: Vec3,
    pub velocity: Vec3,
    pub acceleration: Vec3,
    pub mass: f32,
    pub is_fixed: bool,
    pub sticks: Vec<StickId>,
}

pub struct Stick {
    pub point_a: PointId,
    pub point_b: PointId,
    pub target: f32,
    pub stiffness: f32,
    pub damping: f32,
}

#[derive(Resource)]
pub struct PlantBody {
    pub point_id_gen: PointIdGen,
    pub points: HashMap<PointId, Point>,
    pub stick_id_gen: StickIdGen,
    pub sticks: HashMap<StickId, Stick>,
}

impl PlantBody {
    pub fn new() -> Self {
        PlantBody {
            point_id_gen: PointIdGen::new(),
            points: HashMap::new(),
            stick_id_gen: StickIdGen::new(),
            sticks: HashMap::new(),
        }
    }
    pub fn add_point(&mut self, position: Vec3, is_fixed: bool) -> PointId {
        let id = self.point_id_gen.next();
        let point = Point {
            previous_position: position,
            position,
            velocity: Vec3::ZERO,
            acceleration: Vec3::ZERO,
            mass: 0.1,
            is_fixed,
            sticks: Vec::new(),
        };
        self.points.insert(id, point);
        id
    }
    pub fn add_stick(&mut self, point_a: PointId, point_b: PointId) -> StickId {
        let position_a = self
            .points
            .get(&point_a)
            .expect("Unknown point ID")
            .position;
        let position_b = self
            .points
            .get(&point_b)
            .expect("Unknown point ID")
            .position;
        let stick = Stick {
            point_a: point_a,
            point_b: point_b,
            // use the initial distance as the target
            target: (position_a - position_b).length(),
            stiffness: 1.0,
            damping: 0.1,
        };
        let id = self.stick_id_gen.next();
        // Sticks are saved in the points to make exploring neighbours easier
        self.points.get_mut(&point_a).unwrap().sticks.push(id);
        self.points.get_mut(&point_b).unwrap().sticks.push(id);
        self.sticks.insert(id, stick);
        id
    }
    pub fn update_point_mass(&mut self, id: PointId, mass: f32) {
        self.points.get_mut(&id).expect("Unknown point ID").mass = mass;
    }

    pub fn step_physics(&mut self, delta: f32) {
        for mut point in self.points.values_mut() {
            if point.is_fixed || point.mass == 0.0 {
                return;
            }
            let friction = 0.95;
            point.velocity = point.position - point.previous_position;
            point.previous_position = point.position;
            point.position += point.velocity * friction + point.acceleration * delta * delta;
        }

        // self.velocity = self.position - self.previous_position;
        // self.acceleration = Vec3::ZERO;

        // self.velocity = self.position - self.previous_position;
        // self.previous_position = self.position;
        // let friction = 0.98;
        // self.position += self.velocity * friction + self.acceleration * delta * delta;
        // self.velocity = self.position - self.previous_position;
        // self.acceleration = Vec3::ZERO;
        // self.velocity = self.position - self.previous_position;
    }

    pub fn ripple(&mut self, start_points: Vec<PointId>) {
        let mut explore = start_points;
        let mut explore_pointer = 0;
        let mut relaxed = HashSet::<StickId>::new();
        while explore.len() > explore_pointer {
            let point_id = explore[explore_pointer];
            explore_pointer += 1;
            let sticks = self.points.get(&point_id).unwrap().sticks.clone();
            for stick in sticks.iter() {
                if !relaxed.contains(stick) {
                    relaxed.insert(*stick);
                    let (point_a, point_b) = self.relax(stick);
                    if !explore.contains(&point_a) {
                        explore.push(point_a);
                    }
                    if !explore.contains(&point_b) {
                        explore.push(point_b);
                    }
                }
            }
        }
    }

    pub fn relax(&mut self, stick_id: &StickId) -> (PointId, PointId) {
        let stick = self.sticks.get(stick_id).unwrap();
        let result = (stick.point_a, stick.point_b);
        let [mut point_a, mut point_b] = self
            .points
            .get_many_mut([&stick.point_a, &stick.point_b])
            .unwrap();
        if point_a.is_fixed && point_b.is_fixed {
            return result;
        }

        let center = (point_a.position + point_b.position) / 2.0;
        let direction = match (point_b.position - point_a.position).try_normalize() {
            None => {
                log::warn!("Failed handle stick between points {} and {} which are too close to each other", point_a.position, point_b.position);
                return result;
            }
            Some(dir) => dir * stick.target / 2.0,
        };
        // let slowing = 0.8;
        // println!(
        //     "\tcenter {:?} direction {:?} target {:?}",
        //     center, direction, self.target
        // );

        if point_a.is_fixed {
            point_b.position = point_a.position + direction * 2.0;
        } else if point_b.is_fixed {
            point_a.position = point_b.position - direction * 2.0;
        } else {
            point_a.position = center - direction;
            point_b.position = center + direction;
        }

        result
    }

    pub fn get_fixed_points(&self) -> Vec<PointId> {
        self.points
            .iter()
            .filter_map(|(id, point)| if point.is_fixed { Some(id.clone()) } else { None })
            .collect()
    }
}
