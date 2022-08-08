use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::schemas;

pub fn update_collider_cuboid(
    mut commands: Commands,
    query: Query<(Entity, &schemas::ColliderCuboid), Changed<schemas::ColliderCuboid>>,
) {
    for (entity, schemas::ColliderCuboid { hx, hy, hz }) in query.iter() {
        commands
            .entity(entity)
            .insert(Collider::cuboid(*hx, *hy, *hz));
    }
}

pub fn update_names(
    mut commands: Commands,
    query: Query<(Entity, &schemas::Name), Changed<schemas::Name>>,
) {
    for (entity, schemas::Name(name)) in query.iter() {
        commands.entity(entity).insert(Name::new(name.clone()));
    }
}

pub fn update_rigid_body(
    mut commands: Commands,
    query: Query<(Entity, &schemas::RigidBody), Changed<schemas::RigidBody>>,
) {
    for (entity, schemas::RigidBody(kind)) in query.iter() {
        let rigid_body = match kind.as_str() {
            "Dynamic" => RigidBody::Dynamic,
            "Fixed" => RigidBody::Fixed,
            "KinematicPositionBased" => RigidBody::KinematicPositionBased,
            "KinematicVelocityBased" => RigidBody::KinematicVelocityBased,
            _ => RigidBody::Fixed,
        };
        commands.entity(entity).insert(rigid_body);
    }
}

fn update_primitives(
    mut commands: Commands,
    primitives: Query<
        (
            Entity,
            &schemas::Primitive,
            Option<&mut Handle<Mesh>>,
            Option<&Transform>,
            Added<schemas::Primitive>,
        ),
        Changed<schemas::Primitive>,
    >,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    // let material = material.get_single();
    // if material.is_err() {
    //     return;
    // }
    // let material = material.unwrap();

    for (entity, primitive, mesh_handle, transform, is_added) in primitives.iter() {
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
            let transform = transform.map_or(Transform::default(), |transform| transform.clone());

            commands.entity(entity).insert_bundle(PbrBundle {
                mesh: mesh_handle,
                material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
                transform,
                ..default()
            });
            // .insert_bundle((
            //     mesh_handle,
            //     // material.handle.clone(),
            //     GlobalTransform::default(),
            //     Visibility::default(),
            //     Transform::default(),
            //     ComputedVisibility::default(),
            // ));
        } else {
            // commands.entity(entity).insert(mesh_handle);
        }
    }
}

pub fn update_impulse_joints(
    mut commands: Commands,
    query: Query<(Entity, &schemas::ImpulseJoint), Changed<schemas::ImpulseJoint>>,
    linkeds: Query<(Entity, &schemas::JointLink)>,
) {
    for (entity, schemas::ImpulseJoint { parent, joint }) in query.iter() {
        println!(">>>>>>>>>connect {:?} with {:?}", parent, entity);
        let parent = linkeds
            .iter()
            .find_map(|(entity, link)| if link == parent { Some(entity) } else { None });

        if let Some(parent) = parent {
            let data = RevoluteJointBuilder::new(joint.axis)
                .local_anchor1(joint.local_anchor1)
                .local_anchor2(joint.local_anchor2);
            commands
                .entity(entity)
                .insert(ImpulseJoint::new(parent, data));
        } else {
            println!(">>>>>>>>>no parent");
        }
    }
}

pub struct UpdatersPlugin;

impl Plugin for UpdatersPlugin {
    fn build(&self, app: &mut App) {
        app.add_system(update_collider_cuboid)
            .add_system(update_names)
            .add_system(update_rigid_body)
            .add_system(update_primitives)
            .add_system(update_impulse_joints);
    }
}
