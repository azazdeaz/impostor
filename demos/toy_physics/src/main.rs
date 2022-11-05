use bevy::prelude::*;
use bevy_rapier3d::{prelude::*, na::UnitQuaternion};
use bevy_prototype_debug_lines::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(DebugLinesPlugin::default())        
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugRenderPlugin {
            mode: DebugRenderMode::default() ,//| DebugRenderMode::CONTACTS,
            ..Default::default()
        })
        // .insert_resource(PhysicsHooksWithQueryObject(Box::new(MyPhysicsHooks {})))
        .add_plugin(impostor_editor_camera::EditorCameraPlugin)
        .add_startup_system(setup_stick)
        .add_startup_system(setup_physics)       
        // .add_system(print_ball_altitude)
        .add_system(display_contact_info)
        .run();
}

#[derive(Component)]
struct Stick {}

#[derive(Component)]
struct Ball {}


fn setup_stick(mut commands:Commands) {
    println!("Spawn stick");
    commands.spawn()
        .insert(Stick {})
        .insert(RigidBody::KinematicPositionBased)
        .insert(ActiveEvents::CONTACT_FORCE_EVENTS)
        .insert(Collider::capsule_y(2.0, 0.6))
        .insert_bundle(TransformBundle::default());
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
        .insert(RigidBody::Dynamic)
        .insert(Collider::ball(0.5))
        .insert(Restitution::coefficient(0.7))
        .insert_bundle(TransformBundle::from(Transform::from_xyz(0.01, 4.0, 0.01)));
}

fn print_ball_altitude(positions: Query<&Transform, With<RigidBody>>) {
    for transform in positions.iter() {
        println!("Ball altitude: {}", transform.translation.y);
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
    mut stick: Query<(Entity, &GlobalTransform, &mut Transform), With<Stick>>,
    ball: Query<Entity, With<Ball>>,
) {
    if let (Ok((stick_entity, stick_global, mut stick_transform)), Ok(ball_entity)) = (stick.get_single_mut(), ball.get_single()) {
        /* Find the contact pair, if it exists, between two colliders. */
        if let Some(contact_pair) = rapier_context.contact_pair(stick_entity, ball_entity) {
            // The contact pair exists meaning that the broad-phase identified a potential contact.
            if contact_pair.has_any_active_contacts() {
                // The contact pair has active contacts, meaning that it
                // contains contacts for which contact forces were computed.
            }

            

            // We may also read the contact manifolds to access the contact geometry.
            for manifold in contact_pair.manifolds() {
                // println!("Local-space contact normal: {}", manifold.local_n1());
                // println!("Local-space contact normal: {}", manifold.local_n2());
                println!(": World-space contact normal: {}", manifold.normal());

                

                if let Some(contact) = manifold.find_deepest_contact() {
                    let (local_p, ) = if manifold.rigid_body1().unwrap() == stick_entity {
                        (contact.local_p1(),)
                    }
                    else {
                        (contact.local_p2(),)
                    };
                    // let pos = 
                    println!(": Deepest: {:?} {:?}", contact.dist(), local_p);
                }
                

                // // Read the geometric contacts.
                // for contact_point in manifold.points() {
                //     // Keep in mind that all the geometric contact data are expressed in the local-space of the colliders.
                //     println!("Found local contact point 1: {:?}", contact_point.local_p1());
                //     println!("Found contact distance: {:?}", contact_point.dist()); // Negative if there is a penetration.
                //     println!("Found contact impulse: {}", contact_point.impulse());
                //     println!("Found friction impulse: {:?}", contact_point.tangent_impulse());
                // }

                // // Read the solver contacts.
                for solver_contact in manifold.solver_contacts() {
                    // Keep in mind that all the solver contact data are expressed in world-space.
                    println!("Found solver contact point: {:?}", solver_contact.point());
                    println!("Found solver contact distance: {:?}", solver_contact.dist()); // Negative if there is a penetration.
                    


                    let contact = solver_contact.point();
                    let anchor = stick_global.translation();
                    lines.line(contact, anchor, 0.0);

                    let pushed_contact = contact + manifold.normal();
                    let a = contact - anchor;
                    let b = pushed_contact - anchor;
                    if let Some(rotation)  = UnitQuaternion::rotation_between(&a.into(), &b.into()) {
                        stick_transform.rotate(rotation.into());
                    }

                    lines.line_colored(contact, contact + manifold.normal(), 0.0, Color::CRIMSON)
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