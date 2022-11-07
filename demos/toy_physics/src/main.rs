use bevy::prelude::*;
use bevy_prototype_debug_lines::*;
use bevy_rapier3d::{na::UnitQuaternion, prelude::*};
use rand::prelude::random;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(DebugLinesPlugin::default())
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugRenderPlugin {
            mode: DebugRenderMode::default(), //| DebugRenderMode::CONTACTS,
            ..Default::default()
        })
        // .insert_resource(PhysicsHooksWithQueryObject(Box::new(MyPhysicsHooks {})))
        .add_plugin(impostor_editor_camera::EditorCameraPlugin)
        .add_startup_system(setup_stick)
        .add_startup_system(setup_physics)
        .add_system(shoot_at)
        // .add_system(print_ball_altitude)
        .add_system(display_contact_info)
        .add_stage_after(
            PhysicsStages::StepSimulation,
            "update_plants",
            SystemStage::single_threaded(),
        )
        .add_system_to_stage("update_plants", remember_velocity)
        .run();
}

#[derive(Component)]
struct Stick {}

#[derive(Component)]
struct Ball {}

fn setup_stick(mut commands: Commands) {
    println!("Spawn stick");
    commands
        .spawn()
        .insert(RigidBody::KinematicPositionBased)
        .insert(ActiveEvents::CONTACT_FORCE_EVENTS)
        .insert_bundle(TransformBundle::default())
        // .insert(Collider::ball(0.5))
        .with_children(|children| {
            children
                .spawn()
                .insert(Stick {})
                .insert(Collider::capsule_y(2.0, 0.6))
                .insert_bundle(TransformBundle::from_transform(
                    Transform::from_translation((0.0, 2.0, 0.0).into()),
                ));
        });
}

fn shoot_at(mut commands: Commands, time: Res<Time>) {
    let freq = 2.0;
    if time.seconds_since_startup() % freq - time.delta_seconds_f64() < 0.0 {
        let rad = random::<f32>() * std::f32::consts::TAU;
        let r = 8.0;
        let force = 12.0;
        let (x, z) = rad.sin_cos();
        commands
            .spawn()
            .insert(Ball {})
            .insert(Velocity::linear(Vec3::new(-x * force, 0.0, -z * force)))
            .insert(RigidBody::Dynamic)
            .insert(Collider::ball(0.5))
            .insert(Restitution::coefficient(0.7))
            .insert_bundle(TransformBundle::from(Transform::from_xyz(
                x * r,
                4.0,
                z * r,
            )));
    }
}

fn setup_physics(mut commands: Commands) {
    /* Create the ground. */
    // commands
    //     .spawn()
    //     .insert(Collider::cuboid(100.0, 0.1, 100.0))
    //     .insert_bundle(TransformBundle::from(Transform::from_xyz(0.0, -2.0, 0.0)));

    /* Create the bouncing ball. */
    commands
        .spawn()
        .insert(Ball {})
        .insert(Velocity::default())
        .insert(RigidBody::Dynamic)
        .insert(Collider::ball(0.5))
        .insert(Restitution::coefficient(0.7))
        .insert_bundle(TransformBundle::from(Transform::from_xyz(0.2, 7.0, 0.1)));
}

#[derive(Component)]
struct PrevVelocity(Velocity);

fn remember_velocity(mut commands: Commands, velocities: Query<(Entity, &Velocity)>) {
    // TODO use clone_from() if PrevVelocity already exist
    for (entity, velocity) in velocities.into_iter() {
        commands
            .entity(entity)
            .insert(PrevVelocity(velocity.clone()));
    }
}

fn display_events(
    mut collision_events: EventReader<CollisionEvent>,
    mut contact_force_events: EventReader<ContactForceEvent>,
    stick: Query<Entity, With<Stick>>,
) {
    if let Ok(stick_entity) = stick.get_single() {
        // for collision_event in collision_events.iter() {
        //     println!("Received collision event: {:?}", collision_event);
        // }

        // for contact_force_event in contact_force_events.iter() {
        //     let pair = if stick.contains(contact_force_event.collider1) {
        //         Some((contact_force_event.collider1, contact_force_event.collider2))
        //     }
        //     else if stick.contains(contact_force_event.collider2) {
        //         Some((contact_force_event.collider2, contact_force_event.collider1))
        //     }
        //     else {
        //         None
        //     };
        //     if let Some(plant, actor) = pair {

        //     }
        //     println!("Received contact force event: {:?}", contact_force_event);
        // }
    }
}

fn display_contact_info(
    rapier_context: Res<RapierContext>,
    mut lines: ResMut<DebugLines>,
    stick: Query<Entity, With<Stick>>,
    mut transforms: Query<(&mut Transform, &GlobalTransform)>,
    mut balls: Query<(Entity, &mut Velocity, &PrevVelocity, &GlobalTransform), With<Ball>>,
) {
    for (ball_entity, mut ball_velocity, PrevVelocity(prev_velocity), ball_gt) in balls.iter_mut() {
        if let Ok(stick_entity) = stick.get_single() {
            /* Find the contact pair, if it exists, between two colliders. */
            if let Some(contact_pair) = rapier_context.contact_pair(stick_entity, ball_entity) {
                // The contact pair exists meaning that the broad-phase identified a potential contact.
                if contact_pair.has_any_active_contacts() {
                    // The contact pair has active contacts, meaning that it
                    // contains contacts for which contact forces were computed.
                }

                if let Some((manifold, contact)) = contact_pair.find_deepest_contact() {
                    println!(": World-space contact normal: {}", manifold.normal());

                    if let Some(contact) = manifold.find_deepest_contact() {
                        let rb = if contact_pair.collider1() == stick_entity {
                            manifold.rigid_body1()
                        } else {
                            manifold.rigid_body2()
                        };

                        let rb = rb.unwrap();

                        if let Ok((mut body_local, body_global)) = transforms.get_mut(rb) {
                            for solver_contact in manifold.solver_contacts() {
                                // Keep in mind that all the solver contact data are expressed in world-space.
                                println!(
                                    "Found solver contact point: {:?}",
                                    solver_contact.point()
                                );
                                println!(
                                    "Found solver contact distance: {:?}",
                                    solver_contact.dist()
                                ); // Negative if there is a penetration.

                                let contact = solver_contact.point();
                                let anchor = body_global.translation();
                                lines.line(contact, anchor, 0.0);

                                let pushed_contact = contact - manifold.normal();
                                let a = contact - anchor;
                                let b = pushed_contact - anchor;
                                if let Some(rotation) =
                                    UnitQuaternion::rotation_between(&a.into(), &b.into())
                                {
                                    body_local.rotate(rotation.into());
                                }

                                lines.line_colored(contact, pushed_contact, 0.0, Color::RED);
                                lines.line_colored(
                                    ball_gt.translation(),
                                    ball_gt.translation() + ball_velocity.linvel,
                                    30.0,
                                    Color::INDIGO,
                                );
                                lines.line_colored(
                                    ball_gt.translation() + Vec3::X,
                                    ball_gt.translation() + Vec3::X + prev_velocity.linvel,
                                    30.0,
                                    Color::PINK,
                                );
                                // *ball_velocity = prev_velocity.clone();
                                let keep_prev = 0.7;
                                let keep_curr = 0.2;
                                ball_velocity.linvel = (ball_velocity.linvel * keep_curr)
                                    + (prev_velocity.linvel * keep_prev);
                                ball_velocity.angvel = (ball_velocity.angvel * keep_curr)
                                    + (prev_velocity.angvel * keep_prev);
                            }
                        }
                    }
                }
            }
        }
    }
}

// struct MyPhysicsHooks;
// impl PhysicsHooksWithQuery<NoUserData<'_>> for MyPhysicsHooks {
//     fn modify_solver_contacts(
//         &self,
//         context: ContactModificationContextView,
//         _user_data: &Query<NoUserData>
//     ) {
//         // This is a silly example of contact modifier that does silly things
//         // for illustration purpose:
//         // - Flip all the contact normals.
//         // - Delete the first contact.
//         // - Set the friction coefficients to 0.3
//         // - Set the restitution coefficients to 0.4
//         // - Set the tangent velocities to X * 10.0
//         *context.normal = -*context.normal;

//         if !context.solver_contacts.is_empty() {
//             context.solver_contacts.swap_remove(0);
//         }

//         for solver_contact in &mut *context.solver_contacts {
//             solver_contact.friction = 0.3;
//             solver_contact.restitution = 0.4;
//             solver_contact.tangent_velocity.x = 10.0;
//         }

//         // Use the persistent user-data to count the number of times
//         // contact modification was called for this contact manifold
//         // since its creation.
//         *context.user_data += 1;
//         println!("Contact manifold has been modified {} times since its creation.", *context.user_data);
//     }
// }
