use std::f32::consts::PI;

use bevy::prelude::*;
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
        // .add_system(demo)
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
    let body = SoftBody::new_stem_bend_based();
    xpbd.add_body(body);
}

struct DragInfo {
    soft_body_index: usize,
    index: usize,
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
                    soft_body_index: soft_body_id,
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
        if let Some( body) = xpbd.get_bodies_mut().get_mut(info.soft_body_index) {
            body.particles[info.index].position = new_pos;
        }
    }
}

fn simulate(
    time: Res<Time>,
    mut xpbd: ResMut<XPBDContext>,
    mut shapes: ResMut<DebugShapes>,
) {
    let delta = time.delta_seconds();
    let gravity = Vec3::new(0.0, -9.81, 0.0);
    let gravity = Vec3::ZERO;
    let substeps = 36;
    let sub_delta = delta / substeps as f32;
    for _ in 0..substeps {
        for mut body in xpbd.get_bodies_mut().iter_mut() {
            body.pre_solve(gravity, sub_delta);
            body.solve(sub_delta);
            body.post_solve(sub_delta);
        }
    }
}

fn draw_soft_bodies(mut shapes: ResMut<DebugShapes>, xpbd: Res<XPBDContext>,) {
    for body in xpbd.get_bodies().iter() {
        for constraint in body.constraints.iter() {
            constraint.debug_draw(&body.particles, &mut *shapes)
        }
    }
    // for soft_body in xpbd.get_bodies().iter() {
    //     for edge in soft_body.edges.iter() {
    //         shapes
    //             .line()
    //             .start(soft_body.particles[edge.a].position)
    //             .end(soft_body.particles[edge.b].position)
    //             .color(Color::WHITE);
    //     }
    // }

    // // Draw each tetrahedron slightly smaller
    // for soft_body in xpbd.get_bodies().iter() {
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
    // for soft_body in xpbd.get_bodies().iter() {
    //     for constraint in soft_body.bending_constraints.iter() {
    //         let a = soft_body.particles[constraint.a].position;
    //         let b = soft_body.particles[constraint.b].position;
    //         let c = soft_body.particles[constraint.c].position;
    //         let d = soft_body.particles[constraint.d].position;
    //         let center = (a + b + c + d) / 4.0;
    //         let scale = 1.0;
    //         let a = (a - center) * scale + center;
    //         let b = (b - center) * scale + center;
    //         let c = (c - center) * scale + center;
    //         let d = (d - center) * scale + center;

    //         let color_base = Color::BLUE;
    //         let color_up = Color::ALICE_BLUE;
    //         let color_down = Color::MIDNIGHT_BLUE;
    //         shapes.line().start(a).end(b).color(color_base);
    //         shapes.line().start(a).end(c).color(color_down);
    //         shapes.line().start(a).end(d).color(color_up);
    //         shapes.line().start(b).end(c).color(color_down);
    //         shapes.line().start(b).end(d).color(color_up);
    //     }
    // }
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
