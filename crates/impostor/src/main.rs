use core::panic;
use std::io::Write;

use bevy::{
    ecs::{archetype::Archetypes, component::Components, entity::Entities},
    prelude::*,
    reflect::TypeRegistry,
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
    utils::Duration,
};
use bevy_rapier3d::prelude::{SphericalJointBuilder, RigidBody, ImpulseJoint};
use bevy_inspector_egui::{Inspectable, InspectorPlugin, WorldInspectorPlugin};
use impostor_schemas::schemas;

fn main() {
    App::new()
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 1.0 / 5.0f32,
        })
        .register_type::<schemas::Primitive>()
        .register_type::<schemas::Transform>()
        .add_plugins(DefaultPlugins)
        .add_plugin(WorldInspectorPlugin::new())
        .add_startup_system( setup)
        .add_startup_system(load_scene_system)
        .add_system(update_primitives)
        // .add_system(update_transforms)
        // .add_system(rotate)
        .add_system(inspect)
        .run();
}


fn load_scene_system(mut commands: Commands, asset_server: Res<AssetServer>) {
    // "Spawning" a scene bundle creates a new entity and spawns new instances
    // of the given scene's entities as children of that entity.
    commands.spawn_bundle(DynamicSceneBundle {
        // Scenes are loaded just like any other asset.
        scene: asset_server.load("scenes/start_scene.scn.ron"),
        ..default()
    });

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
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let debug_material = materials.add(StandardMaterial {
        base_color_texture: Some(images.add(uv_debug_texture())),
        ..default()
    });
    commands.spawn().insert(DebugMaterial {
        handle: debug_material,
    });

    // let shapes = [
    //     meshes.add(shape::Cube::default().into()),
    //     meshes.add(shape::Box::default().into()),
    //     meshes.add(shape::Capsule::default().into()),
    //     meshes.add(shape::Torus::default().into()),
    //     meshes.add(shape::Icosphere::default().into()),
    //     meshes.add(shape::UVSphere::default().into()),
    // ];

    // let num_shapes = shapes.len();

    // for (i, shape) in shapes.into_iter().enumerate() {
    //     commands
    //         .spawn_bundle(PbrBundle {
    //             mesh: shape,
    //             material: debug_material.clone(),
    //             transform: Transform {
    //                 translation: Vec3::new(
    //                     -X_EXTENT / 2. + i as f32 / (num_shapes - 1) as f32 * X_EXTENT,
    //                     2.0,
    //                     0.0,
    //                 ),
    //                 rotation: Quat::from_rotation_x(-std::f32::consts::PI / 4.),
    //                 ..default()
    //             },
    //             ..default()
    //         })
    //         .insert(Shape);
    // }

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
    commands.spawn_bundle(PbrBundle {
        mesh: meshes.add(shape::Plane { size: 50. }.into()),
        material: materials.add(Color::SILVER.into()),
        ..default()
    });

    commands.spawn_bundle(Camera3dBundle {
        transform: Transform::from_xyz(0.0, 6., 12.0).looking_at(Vec3::new(0., 1., 0.), Vec3::Y),
        ..default()
    });
}

#[derive(Component)]
struct DebugMaterial {
    handle: Handle<StandardMaterial>,
}

fn update_primitives(
    mut commands: Commands,
    mut primitives: Query<
        (
            Entity,
            &schemas::Primitive,
            Option<&mut Handle<Mesh>>,
            Added<schemas::Primitive>,
        ),
        Changed<schemas::Primitive>,
    >,
    mut material: Query<&DebugMaterial>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let material = material.get_single();
    if material.is_err() {
        return;
    }
    let material = material.unwrap();

    for (entity, primitive, mesh_handle, is_added) in primitives.iter() {
        println!("Update primitive {:?} {}", entity, primitive.shape);

        let new_mesh: Mesh = match &*primitive.shape {
            "cube" => shape::Cube::default().into(),
            "box" => shape::Box::default().into(),
            "capsule" => shape::Capsule::default().into(),
            "torus" => shape::Torus::default().into(),
            "icosphere" => shape::Icosphere::default().into(),
            "uvsphere" => shape::UVSphere::default().into(),
            _ => shape::Icosphere::default().into(),
        };

        let mesh_handle = if let Some(mesh_handle) = mesh_handle {
            *meshes.get_mut(mesh_handle).unwrap() = new_mesh;
            mesh_handle.clone()
        } else {
            let handle = meshes.add(new_mesh);
            commands.entity(entity).insert(handle.clone());
            handle
        };

        if is_added {
            commands
                .entity(entity)
                // .insert_bundle(PbrBundle {
                //     mesh: mesh_handle,
                //     material: material.handle.clone(),
                //     transform: transform.clone()    ,
                //     ..default()
                // })
                .insert_bundle((
                    mesh_handle,
                    material.handle.clone(),
                    GlobalTransform::default(),
                    Visibility::default(),
                    Transform::default(),
                    ComputedVisibility::default(),
                ))
                .insert(Shape);
        } else {
            // commands.entity(entity).insert(mesh_handle);
        }
    }
}


fn update_transforms(
    mut commands: Commands,
    mut transforms: Query<
        (
            Entity,
            &schemas::Transform,
            &mut Transform,
        ),
        Changed<schemas::Transform>,
    >,
) {

    for (entity, schemas::Transform(transform), mut t) in transforms.iter_mut() {
        println!("Update transform {:?} {:?}", entity, transform);
        // t.translation.y = transform.translation.y;
        // commands.entity(entity).insert(transform.clone());
    }
}

fn rotate(mut query: Query<&mut Transform, With<schemas::Primitive>>, time: Res<Time>) {
    for mut transform in &mut query {
        transform.rotate_y(time.delta_seconds() / 2.);
    }
}

/// Creates a colorful test pattern
fn uv_debug_texture() -> Image {
    const TEXTURE_SIZE: usize = 8;

    let mut palette: [u8; 32] = [
        255, 102, 159, 255, 255, 159, 102, 255, 236, 255, 102, 255, 121, 255, 102, 255, 102, 255,
        198, 255, 102, 198, 255, 255, 121, 102, 255, 255, 236, 102, 255, 255,
    ];

    let mut texture_data = [0; TEXTURE_SIZE * TEXTURE_SIZE * 4];
    for y in 0..TEXTURE_SIZE {
        let offset = TEXTURE_SIZE * y * 4;
        texture_data[offset..(offset + TEXTURE_SIZE * 4)].copy_from_slice(&palette);
        palette.rotate_right(4);
    }

    Image::new_fill(
        Extent3d {
            width: TEXTURE_SIZE as u32,
            height: TEXTURE_SIZE as u32,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        &texture_data,
        TextureFormat::Rgba8UnormSrgb,
    )
}

fn inspect(
    keyboard: Res<Input<KeyCode>>,
    all_entities: Query<Entity>,
    entities: &Entities,
    archetypes: &Archetypes,
    components: &Components,
) {
    if keyboard.just_pressed(KeyCode::Space) {
        for entity in all_entities.iter() {
            println!("Entity: {:?}", entity);
            if let Some(entity_location) = entities.get(entity) {
                if let Some(archetype) = archetypes.get(entity_location.archetype_id) {
                    for component in archetype.components() {
                        if let Some(info) = components.get_info(component) {
                            println!("\tComponent: {}", info.name());
                        }
                    }
                }
            }
        }
    }
}
