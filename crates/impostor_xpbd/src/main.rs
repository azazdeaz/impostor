use std::{f32::consts::PI, ops::Mul};

use bevy::prelude::*;
use bevy_prototype_debug_lines::{DebugLinesPlugin, DebugShapes};
use itertools::{Itertools, izip};
use serde_json::json;
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
        // .add_system(demo)
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
    // The particles should beordered so the volume function returns a positive value
    // The order good if the normal of (b-a) x (c-a) should point to (d-a) (and not in the opposite direction)
    // Another way to validate the order:
    //  - Right thumb points from a to d
    //  - Right index finger points from a to b
    //  - If you rotate your hand around the right thumb to point the index finger to c, 
    //  - The order is correct if the rotation is clockwise (when the thumb points towards you)
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
        assert!(
            tetra.rest_volume > 0.0,
            "initial rest_volume must be positive (make sure the tetra indices order is correct)"
        );
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
        let a_position = particles[self.a].position;
        let v1 = particles[self.b].position - a_position;
        let v2 = particles[self.c].position - a_position;
        let v3 = particles[self.d].position - a_position;
        v1.cross(v2).dot(v3) / 6.0
    }



    fn solve(&self, particles: &mut Vec<Particle>, alpha: f32) {
        let mut w = 0.0;
        // all combinations of [id, ...opposite ids]
        // TODO: I think the order doesnt matter because ||AB x AC||^2 is the same regardless of the order, but i should check if this is true
        let id_views = [
            (self.a, self.b, self.d, self.c),
            (self.b, self.a, self.c, self.d),
            (self.c, self.a, self.d, self.b),
            (self.d, self.a, self.b, self.c),
        ];
        let gradients = id_views
            .iter()
            .map(|(pivot, a, b, c)| {
                let pa = particles[*a].position;
                let pb = particles[*b].position;
                let pc = particles[*c].position;
                let gradient = (pb - pa).cross(pc - pa) / 6.0;
                w += particles[*pivot].inverse_mass * gradient.length_squared();
                gradient
            })
            .collect_vec();
        if w == 0.0 {
            return;
        }
        let volume = self.volume(&particles);
        let residual = -(volume - self.rest_volume) / (w + alpha);
        for (index, gradient) in gradients.into_iter().enumerate() {
            let inverse_mass = particles[id_views[index].0].inverse_mass;
            let push = gradient * residual * inverse_mass;
            particles[id_views[index].0].position += push;
        }
    }
}

struct BendConstraint {
    // pivot particle (shared by both triangles)
    a: usize,
    // the other particle shared by both triangles
    b: usize,
    // head of the first triangle
    c: usize,
    // head of the second triangle
    d: usize,
    rest_bend: f32,
}

impl BendConstraint {
    fn from_particles(particles: &Vec<Particle>, a: usize, b: usize, c: usize, d: usize) -> Self {
        let mut bend = Self {
            a,
            b,
            c,
            d,
            rest_bend: 0.0,
        };
        bend.rest_bend = bend.get_bend(particles);
        bend
    }

    fn get_bend(&self, particles: &Vec<Particle>) -> f32 {
        let pa = particles[self.a].position;
        let pb = particles[self.b].position;
        let pc = particles[self.c].position;
        let pd = particles[self.d].position;
        let vab = pb - pa;
        let vac = pc - pa;
        let vad = pd - pa;
        let norm1 = vab.cross(vac).normalize();
        let norm2 = vab.cross(vad).normalize();
        let bend = norm1.dot(norm2).acos();
        bend
    }

    fn solve(&self, particles: &mut Vec<Particle>, alpha: f32) {
        // As described  in "Position Based Dynamics" Appendix A, by Müller et al.
        let pa = particles[self.a].position;
        let pb = particles[self.b].position;
        let pc = particles[self.c].position;
        let pd = particles[self.d].position;
        let wa = particles[self.a].inverse_mass;
        let wb = particles[self.b].inverse_mass;
        let wc = particles[self.c].inverse_mass;
        let wd = particles[self.d].inverse_mass;
        let vab = pb - pa;
        let vac = pc - pa;
        let vad = pd - pa;

        let outer = |a: Vec3, b: Vec3| -> Mat3 { Mat3::from_cols(a * b.x, a * b.y, a * b.z) };

        // Transposed cross product matrix
        let tcpm = |vec: Vec3| -> Mat3 {
            Mat3::from_cols(
                Vec3::new(0.0, vec.z, -vec.y),
                Vec3::new(-vec.z, 0.0, vec.x),
                Vec3::new(vec.y, -vec.x, 0.0),
            )
        };

        // Gradient of the normalized cross product
        let norm_grads = |p1: Vec3, p2: Vec3| {
            let normal = p1.cross(p2);
            let normal_length = normal.length();
            let normal = normal / normal_length;
            let inverse_normal_length = 1.0 / normal_length;
            let grad1 = inverse_normal_length * (-tcpm(p2) + outer(normal, normal.cross(p2)));
            let grad2 = inverse_normal_length * (-tcpm(p1) + outer(normal, normal.cross(p1)));
            (normal, grad1, grad2)
        };

        let (n1, dn1_dpb, dn1_dpc) = norm_grads(vab, vac);
        let (n2, dn2_dpb, dn2_dpd) = norm_grads(vab, vad);

        let d = n1.dot(n2);
        let d_arccos_dx = -1.0 / (1.0 - d * d).sqrt();
        let delta_c = d_arccos_dx * (dn1_dpc.transpose() * n2);
        let delta_d = d_arccos_dx * (dn2_dpd.transpose() * n1);
        let delta_b = d_arccos_dx * (dn1_dpb.transpose() * n2 + dn2_dpb.transpose() * n1);
        let delta_a = -delta_b - delta_c - delta_d;

        let ids = [self.a, self.b, self.c, self.d];
        let ws = [wa, wb, wc, wd];
        let deltas = [delta_a, delta_b, delta_c, delta_d];
        let divisor = izip!(&ws, &deltas).map(|(w, delta)| w * delta.length_squared()).sum::<f32>();
        for (particle, &delta, w) in izip!(&ids, &deltas, &ws) {
            let offset = -(((w * (1.0 - d * d).sqrt()) * d.acos() - self.rest_bend) / divisor) * delta;
            println!("particle {} push: {:?}", particle, alpha * offset);
            let particle = &mut particles[*particle];
            // particle.position += alpha * offset;
        }
    }
}

#[derive(Component)]
struct SoftBody {
    particles: Vec<Particle>,
    edges: Vec<Edge>,
    tetras: Vec<Tetra>,
    bending_constraints: Vec<BendConstraint>,
    edge_compliance: f32,
    volume_compliance: f32,
    bending_compliance: f32,
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
        
        let volume_alpha = self.volume_compliance / delta / delta;
        for volume_constraint in self.tetras.iter() {
            volume_constraint.solve(&mut self.particles, volume_alpha);
        }

        println!("bending constraints---------------------");
        let bending_alpha = self.bending_compliance / delta / delta;
        for bending_constraint in self.bending_constraints.iter() {
            bending_constraint.solve(&mut self.particles, bending_alpha);
        }
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

    fn export(&self) {
        // flat vertices
        let vertices = self
            .particles
            .iter()
            .map(|p| [p.position.x, p.position.y, p.position.z])
            .flatten()
            .collect_vec();
        // flat edge indices
        let edges = self
            .edges
            .iter()
            .map(|e| [e.a as u32, e.b as u32])
            .flatten()
            .collect_vec();
        // flat tetra indices
        let tetras = self
            .tetras
            .iter()
            .map(|t| [t.a as u32, t.b as u32, t.c as u32, t.d as u32])
            .flatten()
            .collect_vec();
        // surfate mesh triangle indices
        let triangles = self
            .tetras
            .iter()
            .flat_map(|t| {
                [
                    [t.a, t.b, t.d],
                    [t.b, t.a, t.c],
                    [t.c, t.a, t.d],
                    [t.d, t.a, t.b],
                ]
            })
            .flatten()
            .collect_vec();

        // print as json
        let json = json!({
                "tet": {
                    // "verts": vertices,
                    "inverse_masses": self.particles.iter().map(|p| p.inverse_mass).collect_vec(),
                //     "tetIds": tetras,
                //     "tetVolumes": self.tetras.iter().map(|t| t.rest_volume).collect_vec(),
                //     "edgeIds": edges,
                // },
                // "viz": {
                //     "triIds": triangles
                }
            }
        );
        println!("{}", json);
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
            bending_constraints: Vec::new(),
            edge_compliance: 0.9,
            volume_compliance: 0.9,
            bending_compliance: 0.9,
        }
    }

    fn new_octaeder_pillar() -> Self {
        let section_length = 0.4;
        let radius = 0.2;
        let sections = 2;
        let mut particles = Vec::new();
        let mut edges = Vec::new();
        let mut tetras = Vec::new();
        let mut bending_constraints = Vec::new();

        for i in 0..=sections {
            // iterate over the angles of the triangle
            for angle in [0.0, PI * 2.0 / 3.0, PI * 4.0 / 3.0].iter() {
                // make every following sector rotated by 1/3 PI
                let angle = angle + (PI * 1.0 / 3.0) * i as f32;
                let x = radius * angle.cos();
                let z = radius * angle.sin();
                let y = i as f32 * section_length;
                particles.push(Particle::from_position(Vec3::new(x, y, z)));
            }
        }

        // create three tetraherons filling up each section without overlap
        let tetra_ids = [[0, 5, 2, 4], [0, 3, 5, 4], [0, 2, 1, 3], [3, 4, 2, 1]];
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

        // add bending constraints
        for i in 1..sections {
            for j in 0..3 {
                //head of the lower triangle
                let c = (i-1) * 3 + j;
                // the common mase of the triangles
                let a = i * 3 + j;
                let b = i * 3 + (j + 2) % 3;
                // head of the upper triangle
                let d = (i+1) * 3 + (j + 2) % 3;
                bending_constraints.push(BendConstraint::from_particles(&particles, a, b, c, d));
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
            bending_constraints,
            edge_compliance: 0.1,
            volume_compliance: 0.1,
            bending_compliance: 0.9,
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
    let body = SoftBody::new_octaeder_pillar();
    body.export();
    commands.spawn(body);
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
    // for soft_body in soft_bodies.iter() {
    //     for edge in soft_body.edges.iter() {
    //         shapes
    //             .line()
    //             .start(soft_body.particles[edge.a].position)
    //             .end(soft_body.particles[edge.b].position)
    //             .color(Color::WHITE);
    //     }
    // }

    // // Draw each tetrahedron slightly smaller
    // for soft_body in soft_bodies.iter() {
    //     for tetra in soft_body.tetras.iter() {
    //         let a = soft_body.particles[tetra.a].position;
    //         let b = soft_body.particles[tetra.b].position;
    //         let c = soft_body.particles[tetra.c].position;
    //         let d = soft_body.particles[tetra.d].position;
    //         let center = (a + b + c + d) / 4.0;
    //         let scale = 0.7;
    //         let a = (a - center) * scale + center;
    //         let b = (b - center) * scale + center;
    //         let c = (c - center) * scale + center;
    //         let d = (d - center) * scale + center;

    //         let color = Color::YELLOW;
    //         shapes.line().start(a).end(b).color(color);
    //         shapes.line().start(b).end(c).color(color);
    //         shapes.line().start(c).end(a).color(color);
    //         shapes.line().start(a).end(d).color(color);
    //         shapes.line().start(b).end(d).color(color);
    //         shapes.line().start(c).end(d).color(color);
    //     }
    // }

    // Draw each bending constraint
    for soft_body in soft_bodies.iter() {
        for constraint in soft_body.bending_constraints.iter() {
            let a = soft_body.particles[constraint.a].position;
            let b = soft_body.particles[constraint.b].position;
            let c = soft_body.particles[constraint.c].position;
            let d = soft_body.particles[constraint.d].position;
            let center = (a + b + c + d) / 4.0;
            let scale = 0.7;
            let a = (a - center) * scale + center;
            let b = (b - center) * scale + center;
            let c = (c - center) * scale + center;
            let d = (d - center) * scale + center;

            let color_base = Color::BLUE;
            let color_up = Color::ALICE_BLUE;
            let color_down = Color::MIDNIGHT_BLUE;
            shapes.line().start(a).end(b).color(color_base);
            shapes.line().start(a).end(c).color(color_down);
            shapes.line().start(a).end(d).color(color_up);
            shapes.line().start(b).end(c).color(color_down);
            shapes.line().start(b).end(d).color(color_up);
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
