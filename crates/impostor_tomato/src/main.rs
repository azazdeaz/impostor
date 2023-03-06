use bevy::prelude::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_prototype_debug_lines::*;
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_plugin(DebugLinesPlugin::default())
        .add_plugin(WorldInspectorPlugin)
        .add_plugin(impostor_editor_camera::EditorCameraPlugin)
        .add_plugins(bevy_mod_picking::DefaultPickingPlugins)
        .add_plugin(bevy_transform_gizmo::TransformGizmoPlugin::default())
        // .add_plugin(TomatoPlugin)
        .add_startup_system(setup)
        .add_startup_system(impostor_tomato::create_demo_plant)
        // .add_system(handle_segment_collisions)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 9000.0,
            range: 100.,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(8.0, 16.0, 8.0),
        ..default()
    });

    // ground plane
    commands
        .spawn(PbrBundle {
            mesh: meshes.add(shape::Plane { size: 500. }.into()),
            material: materials.add(Color::SILVER.into()),
            ..default()
        })
        .insert(Friction {
            coefficient: 0.7,
            combine_rule: CoefficientCombineRule::Min,
        })
        .insert(Collider::cuboid(50.0, 0.1, 50.0));
}
