use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

#[derive(PartialEq, Eq, Clone, Copy, Component)]
enum CustomFilterTag {
    GroupA,
    GroupB,
}

#[derive(Component)]
struct Floor {}

// A custom filter that allows contacts only between rigid-bodies with the
// same user_data value.
// Note that using collision groups would be a more efficient way of doing
// this, but we use custom filters instead for demonstration purpose.
struct SameUserDataFilter;
impl<'a> PhysicsHooksWithQuery<&Floor> for SameUserDataFilter {
    // fn filter_contact_pair(
    //     &self,
    //     context: PairFilterContextView,
    //     tags: &Query<&'a CustomFilterTag>,
    // ) -> Option<SolverFlags> {
    //     if tags.get(context.collider1()).is_ok() 
    //         && tags.get(context.collider2()).is_ok()
    //     {
    //         if tags.get(context.collider1()).ok().copied()
    //         == tags.get(context.collider2()).ok().copied()
    //         {
    //             Some(SolverFlags::COMPUTE_IMPULSES)
    //         } else {
    //             None
    //         }
    //     } else {
    //         Some(SolverFlags::COMPUTE_IMPULSES)
    //     }
        
    // }

    fn modify_solver_contacts(
        &self, 
        context: ContactModificationContextView,
        mut user_data: &Query<&Floor>
    ) {
        // This is a silly example of contact modifier that does silly things
        // for illustration purpose:
        // - Flip all the contact normals.
        // - Delete the first contact.
        // - Set the friction coefficients to 0.3
        // - Set the restitution coefficients to 0.4
        // - Set the tangent velocities to X * 10.0
        *context.raw.normal = -*context.raw.normal;

        if !context.raw.solver_contacts.is_empty() {
            context.raw.solver_contacts.swap_remove(0);
        }

        for solver_contact in &mut *context.raw.solver_contacts {
            solver_contact.friction = 0.3;
            solver_contact.restitution = 0.4;
            solver_contact.tangent_velocity.x *= -10.0;
        }

        // Use the persistent user-data to count the number of times
        // contact modification was called for this contact manifold
        // since its creation.
        // *context.raw.user_data += 1;
        println!("Contact manifold has been modified {} times since its creation.", *context.raw.user_data);
    }
}

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(
            0xF9 as f32 / 255.0,
            0xF9 as f32 / 255.0,
            0xFF as f32 / 255.0,
        )))
        .insert_resource(Msaa::default())
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<&CustomFilterTag>::default())
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    commands.spawn_bundle(Camera3dBundle {
        transform: Transform::from_xyz(-30.0, 30.0, 100.0)
            .looking_at(Vec3::new(0.0, 10.0, 0.0), Vec3::Y),
        ..Default::default()
    });
}

pub fn setup_physics(mut commands: Commands) {
    /*
     * Ground
     */
    commands.insert_resource(PhysicsHooksWithQueryResource(Box::new(
        SameUserDataFilter {},
    )));

    let ground_size = 10.0;

    commands
        .spawn_bundle(TransformBundle::from(Transform::from_xyz(0.0, -10.0, 0.0)))
        .insert(RigidBody::KinematicPositionBased)
        .insert(Floor {})
        .insert(Collider::cuboid(ground_size, 1.2, ground_size))
        .insert(CustomFilterTag::GroupA);

    commands
        .spawn_bundle(TransformBundle::from(Transform::from_xyz(0.0, 0.0, 0.0)))
        .insert(Collider::cuboid(ground_size, 1.2, ground_size))
        .insert(CustomFilterTag::GroupB);

    /*
     * Create the cubes
     */
    let num = 4;
    let rad = 0.5;

    let shift = rad * 2.0;
    let centerx = shift * (num as f32) / 2.0;
    let centery = shift / 2.0;
    let mut group_id = 0;
    let tags = [CustomFilterTag::GroupA, CustomFilterTag::GroupB];
    let colors = [Color::hsl(220.0, 1.0, 0.3), Color::hsl(260.0, 1.0, 0.7)];

    for i in 0..num {
        for j in 0usize..num * 5 {
            let x = (i as f32 + j as f32 * 0.2) * shift - centerx;
            let y = j as f32 * shift + centery + 2.0;
            group_id += 1;

            commands
                .spawn_bundle(TransformBundle::from(Transform::from_xyz(x, y, 0.0)))
                .insert(RigidBody::Dynamic)
                .insert(Collider::cuboid(rad, rad, rad))
                .insert(ActiveHooks::FILTER_CONTACT_PAIRS | ActiveHooks::MODIFY_SOLVER_CONTACTS)
                .insert(tags[group_id % 2])
                .insert(ColliderDebugColor(colors[group_id % 2]));
        }
    }
}