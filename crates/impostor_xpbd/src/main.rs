use std::f32::consts::PI;

use bevy::prelude::*;
use bevy_prototype_debug_lines::{DebugLinesPlugin, DebugShapes};
use itertools::Itertools;
use smooth_bevy_cameras::{
    controllers::orbit::{OrbitCameraBundle, OrbitCameraController, OrbitCameraPlugin},
    LookTransformPlugin,
};
fn main() {
    App::new()
        .insert_resource(Msaa::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(DebugLinesPlugin::default())
        .add_plugin(LookTransformPlugin)
        .add_plugin(OrbitCameraPlugin::default())
        .add_startup_system(setup)
        .add_system(demo)
        .add_system(draw_soft_bodies)
        .add_system(drag_particles)
        .add_system(simulate)
        .run();
}

#[derive(Default)]
struct Particle {
    position: Vec3,
    prev_position: Vec3,
    velocity: Vec3,
    inverse_mass: f32,
}
impl Particle {
    fn from_position(position: Vec3) -> Self {
        Self {
            position,
            prev_position: position,
            inverse_mass: 0.0,
            ..default()
        }
    }
}

struct Edge {
    a: usize,
    b: usize,
    rest_length: f32,
}
impl Edge {
    fn from_particles(particles: &Vec<Particle>, a: usize, b: usize) -> Self {
        let rest_length = (particles[a].position - particles[b].position).length();
        Self { a, b, rest_length }
    }
}

struct Tetra {
    a: usize,
    b: usize,
    c: usize,
    d: usize,
    rest_volume: f32,
}
impl Tetra {
    fn from_particles(
        particles: &mut Vec<Particle>,
        a: usize,
        b: usize,
        c: usize,
        d: usize,
    ) -> Self {
        let mut tetra = Self {
            a,
            b,
            c,
            d,
            rest_volume: 0.0,
        };
        tetra.rest_volume = tetra.volume(particles);
        if tetra.rest_volume > 0.0 {
            let quarter_inverse_mass = 1.0 / (tetra.rest_volume / 4.0);
            particles[a].inverse_mass += quarter_inverse_mass;
            particles[b].inverse_mass += quarter_inverse_mass;
            particles[c].inverse_mass += quarter_inverse_mass;
            particles[d].inverse_mass += quarter_inverse_mass;
        }
        tetra
    }
    fn volume(&self, particles: &Vec<Particle>) -> f32 {
        let v1 = particles[self.b].position - particles[self.a].position;
        let v2 = particles[self.c].position - particles[self.a].position;
        let v3 = particles[self.d].position - particles[self.a].position;
        v1.cross(v2).dot(v3) / 6.0
    }
}

#[derive(Component)]
struct SoftBody {
    particles: Vec<Particle>,
    edges: Vec<Edge>,
    tetras: Vec<Tetra>,
    edge_compliance: f32,
    volume_compliance: f32,
}

impl SoftBody {
    fn pre_solve(&mut self, gravity: Vec3, delta: f32) {
        for particle in self.particles.iter_mut() {
            particle.velocity += gravity * delta;
            particle.prev_position = particle.position;
            particle.position += particle.velocity * delta;

            // bounce off the ground
            if particle.position.y < 0.0 {
                particle.position = particle.prev_position;
                particle.position.y = 0.0;
            }
        }
    }

    fn solve(&mut self, delta: f32) {
        self.solve_edges(delta);
        self.solve_volumes(delta);
    }

    fn post_solve(&mut self, delta: f32) {
        for particle in self.particles.iter_mut() {
            particle.velocity = (particle.position - particle.prev_position) * delta;
        }
    }

    fn solve_edges(&mut self, delta: f32) {
        let alpha = self.edge_compliance / delta / delta;
        for edge in self.edges.iter() {
            let w = self.particles[edge.a].inverse_mass + self.particles[edge.b].inverse_mass;
            if w == 0.0 {
                continue;
            }
            let p1 = self.particles[edge.a].position;
            let p2 = self.particles[edge.b].position;
            let diff = p1 - p2;
            let distance = diff.length();
            if distance == 0.0 {
                continue;
            }
            let direction = diff / distance;
            let delta = p2 - p1;
            let distance = delta.length();
            let residual = -(distance - edge.rest_length) / (w + alpha);
            self.particles[edge.a].position =
                p1 + direction * residual * self.particles[edge.a].inverse_mass;
            self.particles[edge.b].position =
                p2 - direction * residual * self.particles[edge.b].inverse_mass;
        }
    }

    fn solve_volumes(&mut self, delta: f32) {
        let alpha = self.volume_compliance / delta / delta;
        for tetra in self.tetras.iter() {
            let mut w = 0.0;
            // all combinations of [id, ...opposite ids]
            // volIdOrder = [[1, 3, 2], [0, 2, 3], [0, 3, 1], [0, 1, 2]];
            let id_views = [
                (tetra.a, tetra.b, tetra.d, tetra.c),
                (tetra.b, tetra.a, tetra.c, tetra.d),
                (tetra.c, tetra.a, tetra.d, tetra.b),
                (tetra.d, tetra.a, tetra.b, tetra.c),
            ];
            let gradients = id_views
                .iter()
                .map(|(pivot, a, b, c)| {
                    let pa = self.particles[*a].position;
                    let pb = self.particles[*b].position;
                    let pc = self.particles[*c].position;
                    let gradient = (pb - pa).cross(pc - pa) / 6.0;
                    w += self.particles[*pivot].inverse_mass * gradient.length_squared();
                    gradient
                })
                .collect_vec();
            if w == 0.0 {
                continue;
            }
            let volume = tetra.volume(&self.particles);
            let residual = -(volume - tetra.rest_volume) / (w + alpha);
            for (index, gradient) in gradients.into_iter().enumerate() {
                let inverse_mass = self.particles[id_views[index].0].inverse_mass;
                let push = gradient * residual * inverse_mass;
                self.particles[id_views[index].0].position += push;
            }
        }
    }

    fn new_triangle_pillar() -> Self {
        let section_length = 0.4;
        let radius = 0.2;
        let sections = 7;
        let mut particles = Vec::new();
        let mut edges = Vec::new();
        let mut tetras = Vec::new();

        for i in 0..=sections {
            // iterate over the angles of the triangle
            for angle in [0.0, PI * 2.0 / 3.0, PI * 4.0 / 3.0].iter() {
                let x = radius * angle.cos();
                let z = radius * angle.sin();
                let y = i as f32 * section_length;
                particles.push(Particle::from_position(Vec3::new(x, y, z)));
            }
        }

        // create three tetraherons filling up each section without overlap
        let tetra_ids = [[0, 1, 2, 5], [0, 3, 4, 5], [0, 1, 4, 5]];
        for i in 0..sections {
            let offset = i * 3;
            for tetra in tetra_ids.iter() {
                tetras.push(Tetra::from_particles(
                    &mut particles,
                    offset + tetra[0],
                    offset + tetra[1],
                    offset + tetra[2],
                    offset + tetra[3],
                ));
            }
        }

        // create edges between neighboring particles
        for i in 0..=sections {
            let offset = i * 3;
            // connect vertices on this level
            edges.push(Edge::from_particles(&particles, offset, offset + 1));
            edges.push(Edge::from_particles(&particles, offset + 1, offset + 2));
            edges.push(Edge::from_particles(&particles, offset + 2, offset));
            // connect with vertices on the next level
            if i < sections {
                for j in 0..3 {
                    for k in 0..3 {
                        edges.push(Edge::from_particles(&particles, offset + j, offset + 3 + k));
                    }
                }
            }
        }

        // set the inverse mass of the first section to zero
        for i in 0..3 {
            particles[i].inverse_mass = 0.0;
        }

        SoftBody {
            particles,
            edges,
            tetras,
            edge_compliance: 0.9,
            volume_compliance: 0.9,
        }
    }

    fn new_octaeder_pillar() -> Self {
        let section_length = 0.4;
        let radius = 0.2;
        let sections = 7;
        let mut particles = Vec::new();
        let mut edges = Vec::new();
        let mut tetras = Vec::new();

        for i in 0..=sections {
            // iterate over the angles of the triangle
            for angle in [0.0, PI * 2.0 / 3.0, PI * 4.0 / 3.0].iter() {
                // make every other sector rotated by 1/3 PI
                let mut angle = *angle;
                if i % 2 == 0 {
                    angle += PI * 1.0 / 3.0;
                }
                let x = radius * angle.cos();
                let z = radius * angle.sin();
                let y = i as f32 * section_length;
                particles.push(Particle::from_position(Vec3::new(x, y, z)));
            }
        }

        // create three tetraherons filling up each section without overlap
        let tetra_ids = [[0, 2, 4, 5], [0, 3, 4, 5], [0, 1, 2, 3], [3, 1, 2, 4]];
        for i in 0..sections {
            let offset = i * 3;
            for tetra in tetra_ids.iter() {
                tetras.push(Tetra::from_particles(
                    &mut particles,
                    offset + tetra[0],
                    offset + tetra[1],
                    offset + tetra[2],
                    offset + tetra[3],
                ));
            }
        }

        // create edges between neighboring particles
        for i in 0..=sections {
            let offset = i * 3;
            // connect vertices on this level
            edges.push(Edge::from_particles(&particles, offset, offset + 1));
            edges.push(Edge::from_particles(&particles, offset + 1, offset + 2));
            edges.push(Edge::from_particles(&particles, offset + 2, offset));
            // connect with vertices on the next level
            if i < sections {
                for j in 0..3 {
                    for k in 0..3 {
                        edges.push(Edge::from_particles(&particles, offset + j, offset + 3 + k));
                        let k_next = (k + 1) % 3;
                        edges.push(Edge::from_particles(
                            &particles,
                            offset + j,
                            offset + 3 + k_next,
                        ));
                    }
                }
            }
        }

        // set the inverse mass of the first section to zero
        for i in 0..3 {
            particles[i].inverse_mass = 0.0;
        }

        SoftBody {
            particles,
            edges,
            tetras,
            edge_compliance: 0.9,
            volume_compliance: 0.9,
        }
    }
}

fn setup(mut commands: Commands) {
    commands
        .spawn(Camera3dBundle {
            transform: Transform::from_xyz(0.0, 1.0, 5.0),
            ..default()
        })
        .insert(OrbitCameraBundle::new(
            OrbitCameraController::default(),
            Vec3::new(0.0, 1.0, 5.0),
            Vec3::new(0., 0.0, 0.),
            Vec3::Y,
        ));

    // commands.spawn(SoftBody::new_triangle_pillar());
    commands.spawn(SoftBody::new_octaeder_pillar());
}

struct DragInfo {
    soft_body: Entity,
    index: usize,
    grab_distance: f32,
}

#[derive(Default)]
struct DragParticleState {
    info: Option<DragInfo>,
}

fn drag_particles(
    camera_query: Query<(&Camera, &GlobalTransform)>,
    mut soft_bodies: Query<(Entity, &mut SoftBody)>,
    buttons: Res<Input<MouseButton>>,
    windows: Query<&Window>,
    mut shapes: ResMut<DebugShapes>,
    mut drag_state: Local<DragParticleState>,
) {
    let Some(cursor_position) = windows.single().cursor_position() else { return; };
    let (camera, camera_transform) = camera_query.single();
    // Calculate a ray pointing from the camera into the world based on the cursor's position.
    let Some(ray) = camera.viewport_to_world(camera_transform, cursor_position) else { return; };

    if buttons.just_pressed(MouseButton::Left) {
        for (soft_body_id, mut soft_body) in soft_bodies.iter_mut() {
            let mut closest: Option<(usize, f32)> = None;
            for (idx, particle) in soft_body.particles.iter().enumerate() {
                // calculate the particle distance from the ray
                let particle_from_origin = particle.position - ray.origin;
                let closest_point_on_ray = ray.direction * particle_from_origin.dot(ray.direction);
                let distance = (particle_from_origin - closest_point_on_ray).length();
                if distance < 0.1 && (closest.is_none() || distance < closest.unwrap().1) {
                    closest = Some((idx, distance));
                }
            }
            if let Some((index, _)) = closest {
                let particle_position = soft_body.particles[index].position;
                drag_state.info = Some(DragInfo {
                    soft_body: soft_body_id,
                    index,
                    grab_distance: (particle_position - ray.origin).length(),
                });
            }
        }
    } else if buttons.just_released(MouseButton::Left) {
        drag_state.info = None;
    }

    if let Some(info) = &drag_state.info {
        let new_pos = ray.origin + ray.direction * info.grab_distance;
        shapes
            .cuboid()
            .position(new_pos)
            .size(Vec3::ONE * 0.1)
            .color(Color::PINK);
        if let Ok((_, mut soft_body)) = soft_bodies.get_mut(info.soft_body) {
            soft_body.particles[info.index].position = new_pos;
        }
    }
}

fn simulate(time: Res<Time>, mut soft_bodies: Query<&mut SoftBody>) {
    let delta = time.delta_seconds();
    let gravity = Vec3::new(0.0, -9.81, 0.0);
    let substeps = 12;
    let sub_delta = delta / substeps as f32;
    for _ in 0..substeps {
        for mut soft_body in soft_bodies.iter_mut() {
            soft_body.pre_solve(gravity, sub_delta);
            soft_body.solve(sub_delta);
            soft_body.post_solve(sub_delta);
        }
    }
}

fn draw_soft_bodies(mut shapes: ResMut<DebugShapes>, soft_bodies: Query<&SoftBody>) {
    for soft_body in soft_bodies.iter() {
        for edge in soft_body.edges.iter() {
            shapes
                .line()
                .start(soft_body.particles[edge.a].position)
                .end(soft_body.particles[edge.b].position)
                .color(Color::WHITE);
        }
    }

    // Draw each tetrahedron slightly smaller
    for soft_body in soft_bodies.iter() {
        for tetra in soft_body.tetras.iter() {
            let a = soft_body.particles[tetra.a].position;
            let b = soft_body.particles[tetra.b].position;
            let c = soft_body.particles[tetra.c].position;
            let d = soft_body.particles[tetra.d].position;
            let center = (a + b + c + d) / 4.0;
            let scale = 0.7;
            let a = (a - center) * scale + center;
            let b = (b - center) * scale + center;
            let c = (c - center) * scale + center;
            let d = (d - center) * scale + center;

            let color = Color::YELLOW;
            shapes.line().start(a).end(b).color(color);
            shapes.line().start(b).end(c).color(color);
            shapes.line().start(c).end(a).color(color);
            shapes.line().start(a).end(d).color(color);
            shapes.line().start(b).end(d).color(color);
            shapes.line().start(c).end(d).color(color);
        }
    }
}

fn demo(time: Res<Time>, mut shapes: ResMut<DebugShapes>) {
    use std::f32::consts::FRAC_PI_4;

    let seconds = time.elapsed_seconds();

    shapes
        .cuboid()
        .position(Vec3::new(2.0, 0.0, 0.0))
        .size(Vec3::ONE)
        .rotation(Quat::from_rotation_x(seconds * FRAC_PI_4))
        .color(Color::RED);

    shapes
        .cuboid()
        .min_max(Vec3::NEG_ONE, Vec3::ONE)
        .rotation(Quat::from_rotation_y(seconds * FRAC_PI_4))
        .color(Color::PURPLE);
}
