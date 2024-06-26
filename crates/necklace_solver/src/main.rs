use bevy::prelude::*;
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use necklace_solver::{
    draw_bonds, draw_points, graph_relax_bonds, DragParticlePlugin, Rec, RecTime,
    TetPillar,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = rerun::RecordingStreamBuilder::new("rerun_example_minimal").spawn()?;

    rerun::Logger::new(rec.clone()) // recording streams are ref-counted
        .with_path_prefix("logs")
        .with_filter(rerun::default_log_filter())
        .init()?;

    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_plugins(PanOrbitCameraPlugin)
        .add_plugins(DragParticlePlugin)
        .insert_resource(Rec(rec))
        .insert_resource(RecTime(0.0))
        .add_systems(
            Update,
            (
                draw_points,
                draw_bonds,
                update_config,
                graph_relax_bonds,
            ),
        )
        .run();

    Ok(())
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0., 1.5, 6.).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        PanOrbitCamera::default(),
    ));
    // plane
    commands.spawn(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Plane::from_size(5.0))),
        material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
        ..default()
    });

    // light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });

    // let mut stem = StemStructure {
    //     sections: 10,
    //     sides: 6,
    //     radius: 0.1,
    //     section_height: 0.5,
    //     start_translation: Vec3::new(0., 0.5, 0.),
    //     ..default()
    // };
    // stem.build();
    // commands.add(stem);

    let stem = TetPillar {
        sections: 2,
        radius: 0.1,
        section_height: 0.5,
        start_translation: Vec3::new(0., 0.5, 0.),
        ..default()
    };
    commands.add(stem);
}

fn update_config(mut config: ResMut<GizmoConfig>, keyboard: Res<Input<KeyCode>>, time: Res<Time>) {
    if keyboard.just_pressed(KeyCode::D) {
        config.depth_bias = if config.depth_bias == 0. { -1. } else { 0. };
    }
    if keyboard.just_pressed(KeyCode::P) {
        // Toggle line_perspective
        config.line_perspective ^= true;
        // Increase the line width when line_perspective is on
        config.line_width *= if config.line_perspective { 5. } else { 1. / 5. };
    }

    if keyboard.pressed(KeyCode::Right) {
        config.line_width += 5. * time.delta_seconds();
    }
    if keyboard.pressed(KeyCode::Left) {
        config.line_width -= 5. * time.delta_seconds();
    }
}
