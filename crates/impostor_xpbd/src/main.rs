use std::f32::consts::PI;

use bevy::{prelude::*, utils::petgraph::matrix_graph::Zero};
use bevy_prototype_debug_lines::{DebugLinesPlugin, DebugShapes};
use impostor_xpbd::{structs::*, constraints::XPBDConstraint};
use itertools::{izip, Itertools};
use serde_json::json;
use smooth_bevy_cameras::{
    controllers::orbit::{OrbitCameraBundle, OrbitCameraController, OrbitCameraPlugin},
    LookTransformPlugin,
};


fn main() {
    App::new()
        .insert_resource(Msaa::default())
        .insert_resource(XPBDContext::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(DebugLinesPlugin::default())
        .add_plugin(LookTransformPlugin)
        .add_plugin(OrbitCameraPlugin::default())
        .add_startup_system(setup)
        .add_system(draw_soft_bodies)
        .add_system(manual_step)
        .add_system(drag_particles)
        .add_system(simulate)
        .run();
}





fn setup(mut commands: Commands, mut xpbd: ResMut<XPBDContext>) {
    commands
        .spawn(Camera3dBundle {
            transform: Transform::from_xyz(0.0, 1.0, 5.0),
            ..default()
        })
        .insert(OrbitCameraBundle::new(
            OrbitCameraController::default(),
            Vec3::new(0.0, 1.0, 5.0),
            Vec3::new(0., 1.0, 0.),
            Vec3::Y,
        ));

    // commands.spawn(SoftBody::new_triangle_pillar());
    // let body = SoftBody::new_octaeder_pillar();
    // let body = SoftBody::new_triangle_pie_pillar();
    // let body = SoftBody::build_helix(Vec3::ZERO, Quat::IDENTITY, 0.5, -3.0, 10.0 * PI, 60 );
    // let body = SoftBody::build_rod(Vec3::ZERO, Quat::from_axis_angle(Vec3::Z, 0.1), 3.0, 12);
    let body = SoftBody::build_stem(Vec3::ZERO, Quat::from_axis_angle(Vec3::Z, 0.1), 3.0, 12);
    xpbd.add_body(body);
}

struct DragInfo {
    soft_body_index: usize,
    key: ParticleKey,
    grab_distance: f32,
}

#[derive(Default)]
struct DragParticleState {
    info: Option<DragInfo>,
}

fn drag_particles(
    camera_query: Query<(&Camera, &GlobalTransform)>,
    mut xpbd: ResMut<XPBDContext>,
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
        for (soft_body_id, soft_body) in xpbd.get_bodies().iter().enumerate() {
            let mut closest: Option<(ParticleKey, f32)> = None;
            for (key, particle) in soft_body.data.particles.iter() {
                // calculate the particle distance from the ray
                let particle_from_origin = particle.position - ray.origin;
                let closest_point_on_ray = ray.direction * particle_from_origin.dot(ray.direction);
                let distance = (particle_from_origin - closest_point_on_ray).length();
                if distance < 0.1 && (closest.is_none() || distance < closest.unwrap().1) {
                    closest = Some((key, distance));
                }
            }
            if let Some((key, _)) = closest {
                let particle_position = soft_body.data.particles[key].position;
                drag_state.info = Some(DragInfo {
                    soft_body_index: soft_body_id,
                    key,
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
        if let Some( body) = xpbd.get_bodies_mut().get_mut(info.soft_body_index) {
            body.data.particles[info.key].position = new_pos;
        }
    }
}

fn step(xpbd: &mut XPBDContext, delta: f32) {
    let gravity = Vec3::new(0.0, -9.81, 0.0);
    // let gravity = Vec3::ZERO;
    let substeps = 1;
    let sub_delta = delta / substeps as f32;
    for mut body in xpbd.get_bodies_mut().iter_mut() {
        body.data.debug_figures.clear();
    }
    for _ in 0..substeps {
        for mut body in xpbd.get_bodies_mut().iter_mut() {
            body.pre_solve(gravity, sub_delta);
            body.solve(sub_delta);
            body.post_solve(sub_delta);
        }
    }
}

fn simulate(
    time: Res<Time>,
    mut xpbd: ResMut<XPBDContext>,
) {
    let delta = time.delta_seconds();
    if delta.is_zero() {
        return;
    }
    step(&mut xpbd, delta);
}

fn draw_soft_bodies(mut shapes: ResMut<DebugShapes>, mut xpbd: ResMut<XPBDContext>,) {
    for body in xpbd.get_bodies_mut().iter_mut() {
        for fig in body.data.debug_figures.iter() {
            fig.draw_with_debug_shapes(&mut *shapes);
        }
        for constraint in body.constraints.iter() {
            constraint.debug_draw(&body.data, &mut *shapes)
        }
    }
}


fn manual_step(
    keys: Res<Input<KeyCode>>,
    mut xpbd: ResMut<XPBDContext>,
) {
    if keys.just_pressed(KeyCode::Space) {
        step(&mut xpbd, 0.01);
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
