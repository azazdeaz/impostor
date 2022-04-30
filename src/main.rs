use itertools::Itertools;
use rand::Rng;
use std::f32::consts::PI;

use bevy::{
    pbr::AmbientLight,
    prelude::*,
    render::mesh::{
        skinning::{SkinnedMesh, SkinnedMeshInverseBindposes},
        Indices, PrimitiveTopology,
    },
};
use bevy_config_cam::ConfigCam;
// use bevy_config_cam::ConfigCam;

/// Skinned mesh example with mesh and joints data defined in code.
/// Example taken from <https://github.com/KhronosGroup/glTF-Tutorials/blob/master/gltfTutorial/gltfTutorial_019_SimpleSkin.md>
fn main() {
    App::new()
        .insert_resource(Msaa { samples: 4 })
        .add_plugins(DefaultPlugins)
        .add_plugin(ConfigCam)
        .insert_resource(AmbientLight {
            brightness: 0.3,
            ..Default::default()
        })
        .add_startup_system(setup)
        .add_system(joint_animation)
        .run();
}

/// Used to mark a joint to be animated in the [`joint_animation`] system.
#[derive(Component)]
struct AnimatedJoint(f32);

/// Construct a mesh and a skeleton with 2 joints for that mesh,
///   and mark the second joint to be animated.
/// It is similar to the scene defined in `models/SimpleSkin/SimpleSkin.gltf`
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut skinned_mesh_inverse_bindposes_assets: ResMut<Assets<SkinnedMeshInverseBindposes>>,
    asset_server: Res<AssetServer>,
) {
    let texture_handle = asset_server.load("tomato/AG15brn1.png");
    let enable_wireframe = false;
    // Create a camera
    // commands.spawn_bundle(PerspectiveCameraBundle {
    //     transform: Transform::from_xyz(-2.0, 2.5, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
    //     ..default()
    // });

    commands.spawn_bundle(PointLightBundle {
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        point_light: PointLight {
            shadow_depth_bias: 0.0,
            shadow_normal_bias: 0.0,
            shadows_enabled: true,
            ..default()
        },
        ..default()
    });


    commands.spawn_bundle(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Plane { size: 20.0 })),
        material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
        // transform: Transform::from_xyz(2.0, -1.0, 2.0),
        ..default()
    });

    let joint_count = 4;
    let levels = 64;
    let resolution = 12;
    let radius = 0.3;
    let level_height = 0.1;
    let joint_height = levels as f32 / joint_count as f32 * level_height;
    // Create inverse bindpose matrices for a skeleton consists of 2 joints
    let inverse_bindposes =
        skinned_mesh_inverse_bindposes_assets.add(SkinnedMeshInverseBindposes::from(vec![
            Mat4::from_translation(Vec3::new(-0.0, -0.0, -0.0)),
            Mat4::from_translation(Vec3::new(-0.0, -2.0, -0.0)),
            Mat4::from_translation(Vec3::new(-0.0, -4.0, -0.0)),
            Mat4::from_translation(Vec3::new(-0.0, -6.0, -0.0)),
        ]));

    let vertex_count = (levels + 1) * (resolution + 1);
    println!("vertex_count {}", vertex_count);
    let mut positions = Vec::with_capacity(vertex_count);
    let mut normals = Vec::with_capacity(vertex_count);
    let mut uvs = Vec::with_capacity(vertex_count);
    let mut joint_indices = Vec::with_capacity(vertex_count);
    let mut joint_weights = Vec::with_capacity(vertex_count);

    for n in 0..vertex_count {
        let level = (n / (resolution + 1)) as f32;
        let step = (n % (resolution + 1)) as f32;
        let y = level * level_height;
        let theta = (step / resolution as f32) * PI * 2.0;

        let x = theta.cos() * radius;
        let z = theta.sin() * radius;
        let normal_x = theta.cos();
        let normal_z = theta.sin();
        let normal_y = 0.0;
        let mut uv_x = (step / (resolution as f32 / 4.0)) % 2.0;
        if uv_x > 1.0 {
            uv_x = 2.0 - uv_x;
        }
        let mut uv_y = (y / 0.4) % 2.0;
        if uv_y > 1.0 {
            uv_y = 2.0 - uv_y;
        }
        println!("uv {} {}", uv_x, uv_y);
        positions.push([x, y, z]);
        normals.push([normal_x, normal_y, normal_z]);
        uvs.push([uv_x, uv_y]);
        joint_indices.push([0u16, 1, 2, 3]);

        let weights = (0..joint_count)
            .map(|joint_no| {
                ((joint_height - ((joint_no as f32 * joint_height) - y).abs()) / joint_height)
                    .max(0.0)
                    .powi(2)
            })
            .collect_vec();
        // println!("weights {:?}", weights);
        let weights = bevy::math::vec4(weights[0], weights[1], weights[2], weights[3]);
        let weights = weights / (weights[0] + weights[1] + weights[2] + weights[3]);
        joint_weights.push(weights.to_array()); //([1.0 - weight_1, weight_1, 0.0, 0.0]);
                                                // println!(
                                                //     "level {} step {} theta {} weights {:?} y {}",
                                                //     level, step, theta, weights, y
                                                // );
    }

    let quad_count = resolution * levels;
    let mut indices = if enable_wireframe {
        Vec::with_capacity(quad_count * 12)
    } else {
        Vec::with_capacity(quad_count * 6)
    };
    for n in 0..quad_count {
        let res = resolution as u32;
        // number of vertices at one level
        let level_up = res + 1;
        let level = n as u32 / res;
        let step1 = n as u32 % res;
        let step2 = step1 + 1;
        let start = level * level_up;
        // lower triangle
        let a = start + step1;
        let b = start + step2 + level_up;
        let c = start + step2;
        // upper triangle
        let d = start + step1;
        let e = start + step1 + level_up;
        let f = start + step2 + level_up;
        if enable_wireframe {
            indices.extend_from_slice(&[a, b, b, c, c, a, d, e, e, f, f, d])
        } else {
            indices.extend_from_slice(&[a, b, c, d, e, f])
        }
    }

    // Create a mesh
    let mut mesh = if enable_wireframe {
        Mesh::new(PrimitiveTopology::LineList)
    } else {
        Mesh::new(PrimitiveTopology::TriangleList)
    };
    // Set mesh vertex positions
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);

    // Set mesh vertex normals
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    // Set mesh vertex UVs. Although the mesh doesn't have any texture applied,
    //  UVs are still required by the render pipeline. So these UVs are zeroed out.
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    // Set mesh vertex joint indices for mesh skinning.
    // Each vertex gets 4 indices used to address the `JointTransforms` array in the vertex shader
    //  as well as `SkinnedMeshJoint` array in the `SkinnedMesh` component.
    // This means that a maximum of 4 joints can affect a single vertex.

    mesh.insert_attribute(Mesh::ATTRIBUTE_JOINT_INDEX, joint_indices);
    // Set mesh vertex joint weights for mesh skinning.
    // Each vertex gets 4 joint weights corresponding to the 4 joint indices assigned to it.
    // The sum of these weights should equal to 1.
    mesh.insert_attribute(Mesh::ATTRIBUTE_JOINT_WEIGHT, joint_weights);
    // Tell bevy to construct triangles from a list of vertex indices,
    //  where each 3 vertex indices form an triangle.
    mesh.set_indices(Some(Indices::U32(indices)));

    let mesh = meshes.add(mesh);
    for i in -5..5 {
        // Create joint entities
        let joint_0 = commands
            .spawn_bundle((
                Transform::from_xyz(i as f32 * 1.5, 0.0, 0.0),
                GlobalTransform::identity(),
            ))
            .id();
        let joint_1 = commands
            .spawn_bundle((
                AnimatedJoint(1.0),
                Transform::identity(),
                GlobalTransform::identity(),
            ))
            .id();
        let joint_2 = commands
            .spawn_bundle((
                AnimatedJoint(-2.0),
                Transform::identity(),
                GlobalTransform::identity(),
            ))
            .id();
        let joint_3 = commands
            .spawn_bundle((
                AnimatedJoint(3.0),
                Transform::identity(),
                GlobalTransform::identity(),
            ))
            .id();

        // Set joint_1 as a child of joint_0.
        commands.entity(joint_0).push_children(&[joint_1]);
        commands.entity(joint_1).push_children(&[joint_2]);
        commands.entity(joint_2).push_children(&[joint_3]);

        // Each joint in this vector corresponds to each inverse bindpose matrix in `SkinnedMeshInverseBindposes`.
        let joint_entities = vec![joint_0, joint_1, joint_2, joint_3];

        let material_handle = materials.add(StandardMaterial {
            // base_color: Color::rgb(
            //     rand::thread_rng().gen_range(0.0..1.0),
            //     rand::thread_rng().gen_range(0.0..1.0),
            //     rand::thread_rng().gen_range(0.0..1.0),
            // ),
            base_color_texture: Some(texture_handle.clone()),
            alpha_mode: AlphaMode::Blend,
            unlit: false,
            // double_sided: true,
            metallic: 0.001,
            reflectance: 0.01,
            perceptual_roughness: 0.3,
            // flip_normal_map_y: true,
            ..default()
        });
        // let material_handle = materials.add(texture_handle.clone().into());

        // Create skinned mesh renderer. Note that its transform doesn't affect the position of the mesh.
        commands
            .spawn_bundle(PbrBundle {
                mesh: mesh.clone(),
                material: material_handle,
                ..Default::default()
            })
            .insert(SkinnedMesh {
                inverse_bindposes: inverse_bindposes.clone(),
                joints: joint_entities,
            });
    }
}

/// Animate the joint marked with [`AnimatedJoint`] component.
fn joint_animation(time: Res<Time>, mut query: Query<(&mut Transform, &AnimatedJoint)>) {
    for (mut transform, AnimatedJoint(way)) in query.iter_mut() {
        transform.rotation = Quat::from_euler(
            EulerRot::XYZ,
            0.02 * PI * time.time_since_startup().as_secs_f32().cos(),
            0.0,
            0.02 * PI * time.time_since_startup().as_secs_f32().sin(),
        );
        // println!("{:?} ", transform.translation);
        transform.translation = Vec3::from([
            0.1 * PI * time.time_since_startup().as_secs_f32().cos() * way,
            2.0,
            0.1 * PI * time.time_since_startup().as_secs_f32().sin() * way,
        ]);
        // transform.rotation = Quat::from_axis_angle(
        //     Vec3::Y,
        //     0.5 * PI * time.time_since_startup().as_secs_f32().sin() * way,
        // );
    }
}
