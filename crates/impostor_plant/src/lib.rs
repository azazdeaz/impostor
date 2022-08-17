use bevy::{
    prelude::*,
    render::mesh::{
        skinning::{SkinnedMesh, SkinnedMeshInverseBindposes},
        Indices, PrimitiveTopology,
    },
};
use bevy_rapier3d::{
    prelude::*,
    rapier::prelude::{JointAxis, MotorModel},
};
use itertools::Itertools;

fn create_mesh_stem(
    from: Vec3,
    to: Vec3,
    r_start: f32,
    r_end: f32,
    joints: Vec<Entity>,
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    skinned_mesh_inverse_bindposes_assets: &mut ResMut<Assets<SkinnedMeshInverseBindposes>>,
    asset_server: Res<AssetServer>,
) {
    use std::f32::consts::PI;
    let enable_wireframe = true;

    let texture_handle = asset_server.load("tomato/AG15brn1.png");

    let d = (to - from).length();
    // rings per joint
    let resolution = 2.0;
    let ring_resolution = 8;
    let levels = (d * resolution) as usize + 1;
    let level_height = d as f32 / levels as f32;
    let vertex_count = (levels + 1) * (ring_resolution + 1);
    println!("joints.len() {}", joints.len());
    println!("vertex_count {}", vertex_count);
    println!("levels {}", levels);
    println!("level_height {}", level_height);
    println!("d {}", d);
    let mut positions = Vec::with_capacity(vertex_count);
    let mut normals = Vec::with_capacity(vertex_count);
    let mut uvs = Vec::with_capacity(vertex_count);
    let mut joint_indices = Vec::with_capacity(vertex_count);
    let mut joint_weights = Vec::with_capacity(vertex_count);

    let inverse_bindposes = (0..(joints.len()))
        .map(|i| {
            let translation =
                (from + (to - from) * (i as f32 / (joints.len() - 1) as f32)) * Vec3::splat(-1.0);
            println!("T {:?}", translation);
            Mat4::from_translation(translation)
        })
        .collect_vec();
    println!(
        "Inverse bind poses {} {:?}",
        inverse_bindposes.len(),
        inverse_bindposes
    );
    let inverse_bindposes = skinned_mesh_inverse_bindposes_assets
        .add(SkinnedMeshInverseBindposes::from(inverse_bindposes));
    // let inverse_bindposes =
    // skinned_mesh_inverse_bindposes_assets.add(SkinnedMeshInverseBindposes::from(vec![
    //     Mat4::from_translation(Vec3::new(-0.0, -0.0, -0.0)),
    //     Mat4::from_translation(Vec3::new(-0.0, -2.0, -0.0)),
    //     Mat4::from_translation(Vec3::new(-0.0, -4.0, -0.0)),
    //     Mat4::from_translation(Vec3::new(-0.0, -6.0, -0.0)),
    // ]));

    for n in 0..vertex_count {
        let level = (n / (ring_resolution + 1)) as f32;
        let step = (n % (ring_resolution + 1)) as f32;
        let y = level * level_height;
        println!("y {} level {} level_height {}", y, level, level_height);
        let theta = (step / ring_resolution as f32) * PI * 2.0;

        let level_p = level as f32 / levels as f32;
        let radius = r_start + (r_end - r_start) * level_p;
        let x = theta.cos() * radius;
        let z = theta.sin() * radius;
        let normal_x = theta.cos();
        let normal_z = theta.sin();
        let normal_y = 0.0;
        let mut uv_x = (step / (ring_resolution as f32 / 4.0)) % 2.0;
        if uv_x > 1.0 {
            uv_x = 2.0 - uv_x;
        }
        let mut uv_y = (y / 0.4) % 2.0;
        if uv_y > 1.0 {
            uv_y = 2.0 - uv_y;
        }
        positions.push([x, y, z]);
        normals.push([normal_x, normal_y, normal_z]);
        uvs.push([uv_x, uv_y]);
        // joint before this vertex
        let joint_prev = (level_p * (joints.len() - 1) as f32).floor() as u16;
        joint_indices.push([joint_prev, (joint_prev + 1) % joints.len() as u16, 0, 0]);
        // joint_indices.push([0u16, 1, 2, 3]);

        // 0..1.0 place of vertex btwn the two joints
        let joint_p = (level_p * (joints.len()-1) as f32) % 1.0;
        let mut w0 = 1.0 - joint_p;
        let mut w1 = joint_p;
        println!("n {} level_p {}, joint_p: {}, w0 {}, w1 {}",n, level_p, joint_p, w0, w1);
        // normalize weights
        // let sum = w0 + w1;
        // w0 /= sum;
        // w1 /= sum;
        joint_weights.push([w0, w1, 0.0, 0.0]);
        // joint_weights.push([0.0f32, 1.0, 0.0, 0.0]);
    }

    let quad_count = ring_resolution * levels;
    let mut indices = if enable_wireframe {
        Vec::with_capacity(quad_count * 12)
    } else {
        Vec::with_capacity(quad_count * 6)
    };
    for n in (quad_count/levels*0)..(quad_count/levels*levels) {
        let res = ring_resolution as u32;
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
        println!("n {}\ta:{}|{:?}\tb:{}|{:?}\tc:{}|{:?}", n, a, positions[a as usize], b, positions[b as usize], c, positions[c as usize]);
        println!("n {}\ta:{:?}|{:?}\tb:{:?}|{:?}\tc:{:?}|{:?}", n, joint_indices[a as usize], joint_weights[a as usize], joint_indices[b as usize], joint_weights[b as usize], joint_indices[c as usize], joint_weights[c as usize]);
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
    // println!("\n\npositions: {} {:?}", positions.len(), positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);

    // Set mesh vertex normals
    // println!("\n\nnormals: {} {:?}", normals.len(), normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);

    // Set mesh vertex UVs. Although the mesh doesn't have any texture applied,
    //  UVs are still required by the render pipeline. So these UVs are zeroed out.
    // println!("\n\nuvs: {} {:?}", uvs.len(), uvs);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);

    // Set mesh vertex joint indices for mesh skinning.
    // Each vertex gets 4 indices used to address the `JointTransforms` array in the vertex shader
    //  as well as `SkinnedMeshJoint` array in the `SkinnedMesh` component.
    // This means that a maximum of 4 joints can affect a single vertex.

    println!(
        "\n\njoint_indices: {} {:?}",
        joint_indices.len(),
        joint_indices
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_JOINT_INDEX, joint_indices);

    // Set mesh vertex joint weights for mesh skinning.
    // Each vertex gets 4 joint weights corresponding to the 4 joint indices assigned to it.
    // The sum of these weights should equal to 1.
    println!(
        "\n\njoint_weights: {} {:?}",
        joint_weights.len(),
        joint_weights
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_JOINT_WEIGHT, joint_weights);

    // Tell bevy to construct triangles from a list of vertex indices,
    //  where each 3 vertex indices form an triangle.

    // println!("\n\nindices: {} {:?}", indices.len(), indices);
    mesh.set_indices(Some(Indices::U32(indices)));

    let mesh = meshes.add(mesh);

    // let material_handle = materials.add(StandardMaterial {
    //     // base_color: Color::rgb(
    //     //     rand::thread_rng().gen_range(0.0..1.0),
    //     //     rand::thread_rng().gen_range(0.0..1.0),
    //     //     rand::thread_rng().gen_range(0.0..1.0),
    //     // ),
    //     base_color_texture: Some(texture_handle.clone()),
    //     alpha_mode: AlphaMode::Blend,
    //     unlit: false,
    //     // double_sided: true,
    //     metallic: 0.001,
    //     reflectance: 0.01,
    //     perceptual_roughness: 0.3,
    //     // flip_normal_map_y: true,
    //     ..default()
    // });
    let material_handle = materials.add(texture_handle.clone().into());

    // Create skinned mesh renderer. Note that its transform doesn't affect the position of the mesh.
    commands
        .spawn_bundle(PbrBundle {
            mesh,
            material: material_handle,
            ..Default::default()
        })
        .insert(SkinnedMesh {
            inverse_bindposes: inverse_bindposes.clone(),
            joints: joints,
        })
        ;
}

fn create_stem_skeleton(
    commands: &mut Commands,
    joint_count: i32,
    prev_section: Entity,
    prev_section_height: f32,
    rotation: Quat,
    radius: (f32, f32),
) -> Vec<Entity> {
    let joint_height = 4.0;
    let mut prev_section = prev_section;
    let mut prev_section_height = prev_section_height;
    let mut sections = Vec::with_capacity(joint_count as usize + 1);

    let draft = (0..joint_count+1).map(|i| {
        let transform =
            Transform::from_xyz(0.0, joint_height * (i as f32), 0.0).with_rotation(rotation);

        let r = radius.0 + (radius.1 - radius.0) * (i as f32 / (joint_count - 1) as f32);
        (transform, r)
    }).collect_vec();
    let (draft_end, draft_joints) =  draft.split_last().unwrap();

    for (i, (transform, r)) in draft_joints.iter().enumerate() {
        let (rot_y, rot_z, rot_x) = if i == 0 {
            rotation.to_euler(EulerRot::YZX)
        } else {
            (0.0, 0.0, 0.0)
        };

        let section = commands
            .spawn()
            .insert(RigidBody::Dynamic)
            .insert(*transform)
            .insert(GlobalTransform::identity())
            .with_children(|children| {
                children
                    .spawn()
                    .insert(Collider::capsule_y(joint_height / 2.0, *r))
                    .insert(CollisionGroups::new(0b1000, 0b0100))
                    .insert(Transform::from_xyz(0.0, joint_height / 2.0, 0.0));
            })
            .id();

        let rapier_joint = SphericalJointBuilder::new()
            .local_anchor1(Vec3::new(0.0, prev_section_height, 0.0))
            .local_anchor2(Vec3::new(0.0, 0.0, 0.0))
            .motor_position(JointAxis::AngX, rot_x, 90000.0 * *r, 10000.0)
            .motor_position(JointAxis::AngY, rot_y, 90000.0 * *r, 10000.0)
            .motor_position(JointAxis::AngZ, rot_z, 90000.0 * *r, 10000.0)
            .motor_model(JointAxis::AngX, MotorModel::ForceBased)
            .motor_model(JointAxis::AngY, MotorModel::ForceBased)
            .motor_model(JointAxis::AngZ, MotorModel::ForceBased);
        commands
            .entity(section)
            .insert(ImpulseJoint::new(prev_section, rapier_joint));
        // commands.entity(prev_section).push_children(&[section]); // Is this needed?

        prev_section = section;
        sections.push(section);
        prev_section_height = joint_height;
    }

    let end_joint = FixedJointBuilder::new()
        .local_anchor1(Vec3::new(0.0, prev_section_height, 0.0))
        .local_anchor2(Vec3::new(0.0, 0.0, 0.0));
    let end_section = commands.spawn()
        .insert(RigidBody::Dynamic)
        .insert(draft_end.0)
        .insert(GlobalTransform::identity())
        .insert(ImpulseJoint::new(prev_section, end_joint))
        .insert(Collider::ball(draft_end.1))
        .insert(CollisionGroups::new(0b1000, 0b0100))
        .id();
    sections.push(end_section);

    return sections;
}

pub fn create_demo_plant(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut skinned_mesh_inverse_bindposes_assets: ResMut<Assets<SkinnedMeshInverseBindposes>>,
    asset_server: Res<AssetServer>,
) {
    let root = commands
        .spawn()
        .insert(RigidBody::Fixed)
        .insert(GlobalTransform::identity())
        .insert(Transform::from_xyz(0.0, 0.0, 0.0))
        .id();

    let joint_count = 4;

    let joints = create_stem_skeleton(
        &mut commands,
        joint_count,
        root,
        0.0,
        Quat::from_euler(EulerRot::YZX, 0.0, 0.0, 0.0),
        (0.6, 0.4),
    );

    create_stem_skeleton(
        &mut commands,
        3,
        joints[1],
        0.0,
        Quat::from_euler(EulerRot::YZX, 0.0, 0.5, 0.0),
        (0.5, 0.2),
    );
    create_stem_skeleton(
        &mut commands,
        3,
        joints[2],
        0.0,
        Quat::from_euler(EulerRot::YZX, 0.0, -0.2, 0.0),
        (0.42, 0.14),
    );

    create_mesh_stem(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 16.0, 0.0),
        0.6,
        0.4,
        joints,
        &mut commands,
        &mut meshes,
        &mut materials,
        &mut skinned_mesh_inverse_bindposes_assets,
        asset_server,
    );
}
