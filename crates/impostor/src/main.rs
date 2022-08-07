use core::panic;
use std::f32::consts::PI;

use bevy::{
    ecs::{archetype::Archetypes, component::Components, entity::Entities},
    prelude::*,
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
};
use bevy_inspector_egui::WorldInspectorPlugin;
use bevy_rapier3d::prelude::*;
use impostor_schemas::schemas;

fn main() {
    App::new()
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 1.0 / 5.0f32,
        })
        .add_plugins(DefaultPlugins)
        .add_plugin(schemas::SchemasPlugin)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_plugin(WorldInspectorPlugin::new())
        .add_startup_system(setup)
        // .add_startup_system(load_scene_system)
        .add_system(update_primitives)
        .add_startup_system(create_car)
        .add_system(keyboard_input_system)
        // .add_system(update_transforms)
        // .add_system(rotate)
        .add_system(inspect)
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
        .insert(RigidBody::Dynamic)
        .insert(Collider::cuboid(width / 2., height / 2., length / 2.))
        // .insert(Restitution::coefficient(0.7))
        .insert_bundle(TransformBundle::from(Transform::from_xyz(0.0, 4.0, 0.0)))
        .insert(CollisionGroups::new(0b1111, 0b0111))
        .id();

    let mut add_wheel = |front: f32, left: f32| {
        let joint = RevoluteJointBuilder::new(Vect::X)
            .local_anchor1(Vec3::new(width / 2. * left, 0.0, length / 2. * front))
            .local_anchor2(Vec3::new(0.0, 0.0, 0.0));

        commands
            .spawn()
            .insert(RigidBody::Dynamic)
            .insert_bundle(TransformBundle::from(Transform::from_xyz(0., 6., 0.)))
            .with_children(|parent| {
                let mut wheel = parent.spawn();
                wheel
                    .insert(Collider::cylinder(wheel_width / 2.0, wheel_radius))
                    // .insert(Restitution::coefficient(0.7))
                    .insert_bundle(TransformBundle::from(Transform::from_rotation(
                        Quat::from_axis_angle(Vec3::Z, PI / 2.),
                    )))
                    .insert(ImpulseJoint::new(chasis, joint))
                    .insert(CollisionGroups::new(0b1000, 0b1111));
                if left > 0. {
                    wheel.insert(LeftWheel {});
                } else {
                    wheel.insert(RightWheel {});
                }
            });
    };

    add_wheel(1., 1.);
    add_wheel(1., -1.);
    add_wheel(-1., 1.);
    add_wheel(-1., -1.);
}

fn keyboard_input_system(
    keyboard_input: Res<Input<KeyCode>>,
    mut joints: Query<&mut ImpulseJoint>,
    left_wheels: Query<Entity, With<LeftWheel>>,
    right_wheels: Query<Entity, With<RightWheel>>,
) {
    let mut speed = (0., 0.);
    if keyboard_input.pressed(KeyCode::Up) {
        speed = (12., 12.);
    } else if keyboard_input.pressed(KeyCode::Down) {
        speed = (-12., -12.);
    } else if keyboard_input.pressed(KeyCode::Left) {
        speed = (-12., 12.);
    } else if keyboard_input.pressed(KeyCode::Right) {
        speed = (12., -12.);
    }

    for entity in left_wheels.iter() {
        if let Ok(mut joint) = joints.get_mut(entity) {
            if let Some(joint) = joint.data.as_revolute_mut() {
                joint.set_motor_velocity(speed.0, 10.);
            }
        }
    }

    for entity in right_wheels.iter() {
        if let Ok(mut joint) = joints.get_mut(entity) {
            if let Some(joint) = joint.data.as_revolute_mut() {
                joint.set_motor_velocity(speed.1, 10.);
            }
        }
    }
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
    commands
        .spawn_bundle(PbrBundle {
            mesh: meshes.add(shape::Plane { size: 50. }.into()),
            material: materials.add(Color::SILVER.into()),
            ..default()
        })
        .insert(Collider::cuboid(50.0, 0.1, 50.0));

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
