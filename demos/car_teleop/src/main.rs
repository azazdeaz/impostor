use core::panic;
use std::f32::consts::PI;

use bevy::{
    ecs::{archetype::Archetypes, component::Components, entity::Entities},
    prelude::*,
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
};
use bevy_egui::EguiPlugin;
use bevy_inspector_egui::{WorldInspectorParams, WorldInspectorPlugin};
use bevy_rapier3d::prelude::*;
use impostor_schemas::{schemas, ui, updaters::UpdatersPlugin};

fn main() {
    let mut world_inspector_params = WorldInspectorParams::default();
    world_inspector_params.ignore_component::<Collider>();
    world_inspector_params.ignore_component::<CollisionGroups>();
    world_inspector_params.ignore_component::<RigidBody>();
    world_inspector_params.ignore_component::<Restitution>();
    world_inspector_params.ignore_component::<RapierRigidBodyHandle>();
    world_inspector_params.ignore_component::<RapierColliderHandle>();
    world_inspector_params.ignore_component::<RapierImpulseJointHandle>();
    // world_inspector_params.ignore_component::<ImpulseJoint>();
    world_inspector_params.ignore_component::<ComputedVisibility>();
    world_inspector_params.ignore_component::<GlobalTransform>();

    App::new()
        .insert_resource(Msaa { samples: 4 })
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 1.0 / 5.0f32,
        })
        .insert_resource(world_inspector_params)
        .add_plugins(DefaultPlugins)
        .add_plugin(EguiPlugin)
        .add_plugin(schemas::SchemasPlugin)
        .add_plugin(impostor_teleop::TeleopPlugin::new(
            "LeftWheel".into(),
            "RightWheel".into(),
        ))
        .add_plugin(impostor_editor_camera::EditorCameraPlugin)
        .add_system(ui::schemas_ui)
        .add_plugin(UpdatersPlugin)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_plugin(WorldInspectorPlugin::new().filter::<With<schemas::Editable>>())
        .add_startup_system(setup)
        .add_startup_system(load_scene_system)
        .add_startup_system(impostor_plant::create_demo_plant)
        .add_plugin(impostor_plant::PlantPlugin)
        .add_startup_system(create_car)
        // .add_system(keyboard_input_system)
        // .add_system(update_transforms)
        // .add_system(rotate)
        // .add_system(inspect)
        .run();
}

#[derive(Component)]
struct LeftWheel {}
#[derive(Component)]
struct RightWheel {}

fn create_car(mut commands: Commands) {
    let width = 2.0;
    let length = 4.0;
    let height = 0.4;
    let wheel_width = 1.2;
    let wheel_radius = 1.0;

    let chasis = commands
        .spawn()
        .insert(schemas::Name("Chasis".into()))
        .insert(schemas::RigidBody("Dynamic".into()))
        .insert(schemas::ColliderCuboid::new(
            width / 2.,
            height / 2.,
            length / 2.,
        ))
        // .insert(Restitution::coefficient(0.7))
        .insert(schemas::Transform(Transform::from_xyz(20.0, 4.0, 20.0)))
        .insert(schemas::CollisionGroups::new(0b1111, 0b0111))
        .id();

    let mut add_wheel = |front: f32, left: f32| {
        let mut wheel = commands.spawn();
        wheel
            .insert(schemas::Name(format!("Wheel {} {}", front, left)))
            .insert(schemas::RigidBody("Dynamic".into()))
            .insert(schemas::ImpulseJoint {
                parent: chasis,
                joint: schemas::RevolutJoint {
                    axis: Vec3::X,
                    local_anchor1: Vec3::new(
                        (width / 2. + wheel_width * 0.6) * left,
                        0.0,
                        length / 2. * front,
                    ),
                    local_anchor2: Vec3::new(0.0, 0.0, 0.0),
                },
            })
            .insert(schemas::Transform(Transform::from_xyz(0., 6., 0.)))
            .with_children(|parent| {
                parent
                    .spawn()
                    .insert(schemas::ColliderCylinder::new(
                        wheel_width / 2.0,
                        wheel_radius,
                    ))
                    .insert(Friction {
                        coefficient: 0.7,
                        combine_rule: CoefficientCombineRule::Min,
                    })
                    .insert(schemas::Restitution { coefficient: 0.7 })
                    .insert(schemas::Transform(Transform::from_rotation(
                        Quat::from_axis_angle(Vec3::Z, PI / 2.),
                    )))
                    .insert(schemas::CollisionGroups::new(0b1000, 0b1111));
            });
        if left > 0. {
            wheel.insert(schemas::Tag("LeftWheel".into()));
        } else {
            wheel.insert(schemas::Tag("RightWheel".into()));
        }
    };

    add_wheel(1., 1.);
    add_wheel(1., -1.);
    add_wheel(-1., 1.);
    add_wheel(-1., -1.);
}

fn load_scene_system(mut commands: Commands, asset_server: Res<AssetServer>) {
    // "Spawning" a scene bundle creates a new entity and spawns new instances
    // of the given scene's entities as children of that entity.
    commands
        .spawn_bundle(DynamicSceneBundle {
            // Scenes are loaded just like any other asset.
            scene: asset_server.load("scenes/start_scene.scn.ron"),
            ..default()
        })
        .insert(Name::new("Loaded scene"))
        .insert(schemas::Editable::default());

    // This tells the AssetServer to watch for changes to assets.
    // It enables our scenes to automatically reload in game when we modify their files
    asset_server.watch_for_changes().unwrap();
}

/// A marker component for our shapes so we can query them separately from the ground plane
#[derive(Component, Reflect, Default)]
#[reflect(Component)]
struct Shape;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn_bundle(PointLightBundle {
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
        .spawn_bundle(PbrBundle {
            mesh: meshes.add(shape::Plane { size: 50. }.into()),
            material: materials.add(Color::SILVER.into()),
            ..default()
        })
        .insert(Friction {
            coefficient: 0.7,
            combine_rule: CoefficientCombineRule::Min,
        })
        .insert(Collider::cuboid(50.0, 0.1, 50.0));
}
