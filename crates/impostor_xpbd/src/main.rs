use std::f32::consts::PI;

use bevy::{input::mouse::MouseMotion, prelude::*};

use bevy_prototype_debug_lines::{DebugLinesPlugin, DebugShapes};

fn main() {
    App::new()
        .insert_resource(Msaa::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(DebugLinesPlugin::default())
        .add_startup_system(setup)
        .add_system(demo)
        .add_system(draw_soft_bodies)
        .add_system(drag_particles)
        .run();
}

#[derive(Component)]
struct SoftBody {
    verticles: Vec<Vec3>,
    sticks: Vec<(usize, usize)>,
    tetras: Vec<(usize, usize, usize, usize)>,
}

fn setup(mut commands: Commands) {
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(0.0, 0.0, 5.0),
        ..default()
    });

    let length = 6.0;
    let radius = 0.4;
    let sections = 7;
    let mut verticles = Vec::new();
    let mut sticks = Vec::new();
    let mut tetras = Vec::new();

    for i in 0..=sections {
        // iterate over the angles of the triangle
        for angle in [0.0, PI * 2.0 / 3.0, PI * 4.0 / 3.0].iter() {
            let x = radius * angle.cos();
            let z = radius * angle.sin();
            let y = i as f32 / sections as f32 * length;
            verticles.push(Vec3::new(x, y, z));
        }
    }

    // create three tetraherons filling up each section without overlap
    for i in 0..sections {
        let offset = i * 3;
        tetras.push((offset, offset + 1, offset + 2, offset + 5));
        tetras.push((offset, offset + 3, offset + 4, offset + 5));
        tetras.push((offset, offset + 1, offset + 4, offset + 5));
    }

    // create sticks between neighboring verticles
    for i in 0..sections {
        let offset = i * 3;
        // connect vertices on this level
        sticks.push((offset, offset + 1));
        sticks.push((offset + 1, offset + 2));
        sticks.push((offset + 2, offset));
        // connect with vertices on the next level
        for j in 0..3 {
            for k in 0..3 {
                sticks.push((offset + j, offset + 3 + k));
            }
        }
    }

    commands.spawn(SoftBody {
        verticles,
        sticks,
        tetras,
    });
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
    mut motion_evr: EventReader<MouseMotion>,
    mut drag_state: Local<DragParticleState>,
) {
    let Some(cursor_position) = windows.single().cursor_position() else { return; };
    let (camera, camera_transform) = camera_query.single();
    // Calculate a ray pointing from the camera into the world based on the cursor's position.
    let Some(ray) = camera.viewport_to_world(camera_transform, cursor_position) else { return; };

    if buttons.just_pressed(MouseButton::Left) {
        for (soft_body_id, mut soft_body) in soft_bodies.iter_mut() {
            let mut closest: Option<(&Vec3, f32)> = None;
            for verticle in soft_body.verticles.iter() {
                // calculate the verticle distance from the ray
                let verticle_from_origin = *verticle - ray.origin;
                let closest_point_on_ray = ray.direction * verticle_from_origin.dot(ray.direction);
                let distance = (verticle_from_origin - closest_point_on_ray).length();
                if distance < 0.1 && (closest.is_none() || distance < closest.unwrap().1) {
                    closest = Some((verticle, distance));
                }
            }
            if let Some((verticle, _)) = closest {
                drag_state.info = Some(DragInfo {
                    soft_body: soft_body_id,
                    index: soft_body
                        .verticles
                        .iter()
                        .position(|v| v == verticle)
                        .unwrap(),
                    grab_distance: (*verticle - ray.origin).length(),
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
            soft_body.verticles[info.index] = new_pos;
        }
    }
}

fn mouse_motion(mut motion_evr: EventReader<MouseMotion>) {
    for ev in motion_evr.iter() {
        println!("Mouse moved: X: {} px, Y: {} px", ev.delta.x, ev.delta.y);
    }
}

fn draw_soft_bodies(mut shapes: ResMut<DebugShapes>, soft_bodies: Query<&SoftBody>) {
    for soft_body in soft_bodies.iter() {
        for (a, b) in soft_body.sticks.iter() {
            shapes
                .line()
                .start(soft_body.verticles[*a])
                .end(soft_body.verticles[*b])
                .color(Color::WHITE);
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
