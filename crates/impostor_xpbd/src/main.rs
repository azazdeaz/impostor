use std::f32::consts::PI;

use bevy::prelude::*;

use bevy_prototype_debug_lines::{DebugLinesPlugin, DebugShapes};

fn main() {
    App::new()
        .insert_resource(Msaa::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(DebugLinesPlugin::default())
        .add_startup_system(setup)
        .add_system(demo)
        .add_system(draw_soft_bodies)
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
