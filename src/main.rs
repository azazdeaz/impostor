use core::panic;
use std::io::Write;

use bevy::{prelude::*, reflect::TypeRegistry, utils::Duration, render::render_resource::{Extent3d, TextureDimension, TextureFormat}};
// use bevy_inspector_egui::{Inspectable, InspectorPlugin, WorldInspectorPlugin};

fn main() {
    static CREATE: &str = "create";
    static SAVE: &str = "save";
    App::new()
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 1.0 / 5.0f32,
        })
        .register_type::<Primitive>()
        .add_plugins(DefaultPlugins)
        .add_startup_stage_after(StartupStage::Startup, CREATE, SystemStage::parallel())
        .add_startup_stage_after(CREATE, SAVE, SystemStage::parallel())
        // .add_plugin(WorldInspectorPlugin::new())
        .add_startup_system_to_stage(CREATE, setup)
        // .add_startup_system_to_stage(SAVE, save_scene_system.exclusive_system().at_end())
        .add_startup_system(load_scene_system)
        .add_system(update_primitives)
        .add_system(rotate)
        .run();
}


#[derive(Component, Reflect, Default)]
#[reflect(Component)] // this tells the reflect derive to also reflect component behaviors
struct ComponentA {
    pub x: f32,
    pub y: f32,
}

#[derive(Component, Reflect)]
#[reflect(Component)]
struct ComponentB {
    pub value: String,
    #[reflect(ignore)]
    pub _time_since_startup: Duration,
}

#[derive(Component, Reflect, Default)]
#[reflect(Component)] // this tells the reflect derive to also reflect component behaviors
struct Primitive {
    pub shape: String,
}

impl FromWorld for ComponentB {
    fn from_world(world: &mut World) -> Self {
        let time = world.resource::<Time>();
        ComponentB {
            _time_since_startup: time.time_since_startup(),
            value: "Default Value".to_string(),
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

fn save_scene_system(world: &mut World) {
    // Scenes can be created from any ECS World. You can either create a new one for the scene or
    // use the current World.
    // let mut scene_world = World::new();
    // scene_world.spawn().insert(Primitive { shape: "cube".into() }).insert(Transform::identity());
    
    // Scenes can be created from any ECS World. You can either create a new one for the scene or
    // use the current World.
    let mut scene_world = World::new();

    scene_world
        .spawn()
        .insert_bundle((Primitive { shape: "cube".into() }, Transform::default()));

    // The TypeRegistry resource contains information about all registered types (including
    // components). This is used to construct scenes.
    let type_registry = world.resource::<TypeRegistry>();

    // The TypeRegistry resource contains information about all registered types (including
    // components). This is used to construct scenes.
    // let type_registry = scene_world.resource::<TypeRegistry>();
    // let type_registry = TypeRegistry::default();
    type_registry.write().register::<Primitive>();
    // type_registry.write().register::<Transform>();
    // type_registry.write().register::<String>();
    // type_registry.write().register::<Player>();
    // type_registry.write().register::<Enemy>();
    let scene = DynamicScene::from_world(&scene_world, &type_registry);

    // Scenes can be serialized like this:
    info!("{}", scene.serialize_ron(&type_registry).unwrap());

    let mut file = std::fs::File::create("assets/scenes/start_scene.scn.ron").unwrap();
    file.write_all(
        scene
            .serialize_ron(&type_registry)
            .unwrap()
            .as_bytes(),
    )
    .unwrap();
}


/// A marker component for our shapes so we can query them separately from the ground plane
#[derive(Component, Reflect, Default)]
#[reflect(Component)] 
struct Shape;

const X_EXTENT: f32 = 14.;

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
    commands.spawn().insert(DebugMaterial { handle: debug_material });

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
    handle: Handle<StandardMaterial>
}



fn update_primitives(
    mut commands: Commands,
    mut primitives: Query<(Entity, &Primitive, Option<&mut Handle<Mesh>>), Changed<Primitive>>,
    mut material: Query<&DebugMaterial>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    let material = material.get_single();
    if material.is_err() {
        return;
    }
    let material = material.unwrap();

    for (entity, primitive, mesh_handle) in primitives.iter() {
        let new_mesh:  Mesh = match &*primitive.shape {
            "cube" => shape::Cube::default().into(),
            "box" => shape::Box::default().into(),
            "capsule" => shape::Capsule::default().into(),
            "torus" => shape::Torus::default().into(),
            "icosphere" => shape::Icosphere::default().into(),
            "uvsphere" => shape::UVSphere::default().into(),
            _ => panic!("Unknown Primitive shape {}", primitive.shape)
        };

        let mesh_handle = if let Some(mesh_handle) = mesh_handle {
            *meshes.get_mut(mesh_handle).unwrap() = new_mesh;
            mesh_handle.clone()
        }
        else {
            let handle = meshes.add(new_mesh);
            commands.entity(entity).insert(handle.clone());
            handle
        };

        commands
            .spawn_bundle(PbrBundle {
                mesh: mesh_handle,
                material: material.handle.clone(),
                ..default()
            })
            .insert(Shape);
    }

}

fn rotate(mut query: Query<&mut Transform, With<Shape>>, time: Res<Time>) {
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