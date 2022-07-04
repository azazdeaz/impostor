use bevy::{
    prelude::*,
};
use bevy_rapier3d::{
    prelude::*,
    rapier::prelude::{JointAxis, MotorModel},
};
use std::rc::Rc;
use std::{cell::RefCell, fmt};

fn main() {
    App::new()
        .insert_resource(Msaa { samples: 4 })
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugin(RapierDebugRenderPlugin::default())
        .add_startup_system(setup_plant)
        .add_startup_system(setup_scene)
        .run();
}

#[derive(Clone)]
struct LNode {
    next: Option<Rc<RefCell<LNode>>>,
    prev: Option<Rc<RefCell<LNode>>>,
    branches: Vec<Rc<RefCell<LNode>>>,
    data: StemSection,
}
impl Default for LNode {
    fn default() -> Self {
        LNode {
            next: None,
            prev: None,
            branches: Vec::new(),
            data: StemSection::default(),
        }
    }
}
impl fmt::Debug for LNode {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let next = if let Some(next) = self.next.as_ref() {
            format!(" > ${:?}", next.borrow())
        } else {
            " |".to_string()
        };
        write!(
            f,
            "Stem {:.3?}/{:.3?}{}",
            self.data.length, self.data.radius, next
        )
    }
}
// impl LNode {
//     fn step(&mut self) {
//         self.data.length += (1.0 - self.data.length) / 6.0;
//         println!("_3length {}", self.data.length);
//     }
// }

#[derive(Debug, Clone)]
struct StemSection {
    order: u32,
    length: f32,
    max_length: f32,
    radius: f32,
    direction: Quat,
}
impl Default for StemSection {
    fn default() -> Self {
        Self {
            order: 0,
            length: 1.0,
            max_length: 4.0,
            radius: 1.0,
            direction: Quat::default(),
        }
    }
}

fn step(ln: Rc<RefCell<LNode>>) {
    grow(&mut ln.borrow_mut());
    extend(&mut ln.borrow_mut());

    if ln.borrow().next.is_some() {
        step(Rc::clone(ln.borrow().next.as_ref().unwrap()))
    }
}

fn grow(ln: &mut LNode) {
    ln.data.length += (ln.data.max_length - ln.data.length) / 3.0;
    let prev_radius = if let Some(prev) = ln.prev.as_ref() {
        prev.borrow().data.radius
    } else {
        1.0
    };
    ln.data.radius += (prev_radius - ln.data.radius) / 12.0;
}

fn extend(ln: &mut LNode) {
    if ln.data.length > ln.data.max_length * 0.8 && ln.next.is_none() {
        ln.next = Some(Rc::new(RefCell::new(LNode {
            next: None,
            prev: None,
            branches: Vec::new(),
            data: StemSection {
                order: ln.data.order,
                length: 0.2,
                max_length: 4.0,
                radius: 0.001,
                direction: Quat::default(),
            },
        })));
    }
}

fn setup_plant(
    mut commands: Commands,
    // mut meshes: ResMut<Assets<Mesh>>,
    // mut materials: ResMut<Assets<StandardMaterial>>,
    // mut skinned_mesh_inverse_bindposes_assets: ResMut<Assets<SkinnedMeshInverseBindposes>>,
    // asset_server: Res<AssetServer>,
) {
    let root_joint = commands
        .spawn()
        .insert(RigidBody::Fixed)
        .insert(GlobalTransform::identity())
        .insert(Transform::from_xyz(0.0, 0.0, 0.0))
        .id();

    let plant = Rc::new(RefCell::new(LNode {
        data: StemSection {
            order: 0,
            length: 0.2,
            max_length: 4.0,
            radius: 0.001,
            direction: Quat::default(),
        },
        ..Default::default()
    }));

    println!(">> {:?}", plant.borrow());
    for _ in 0..16 {
        step(Rc::clone(&plant));
        println!(">> {:?}", plant.borrow());
    }
    build_joints(&mut commands, &plant.borrow(), root_joint);
}

fn build_joints(commands: &mut Commands, ln: &LNode, prev_joint: Entity) {
    let transform = Transform::from_xyz(0.0, ln.data.length, 0.0);
    let (rot_y, rot_z, rot_x) = (0.0, 0.0, 0.0); //transform.rotation.to_euler(EulerRot::YZX);
    println!("rot_x {} rot_y {} rot_z {}", rot_x, rot_y, rot_z);
    let section = commands
        .spawn()
        .insert(RigidBody::Dynamic)
        .insert(transform)
        .insert(GlobalTransform::identity())
        .with_children(|children| {
            children
                .spawn()
                .insert(Collider::capsule_y(ln.data.length / 2.0, ln.data.radius))
                .insert(CollisionGroups::new(0b1000, 0b0100))
                .insert(Transform::from_xyz(0.0, -ln.data.length / 2.0, 0.0));
        })
        .id();

    let rapier_joint = SphericalJointBuilder::new()
        .local_anchor1(Vec3::new(0.0, 0.0, 0.0))
        .local_anchor2(Vec3::new(0.0, -ln.data.length, 0.0))
        .motor_position(JointAxis::AngX, rot_x, 2000.0 * ln.data.radius, 10000.0)
        .motor_position(JointAxis::AngY, rot_y, 2000.0 * ln.data.radius, 10000.0)
        .motor_position(JointAxis::AngZ, rot_z, 2000.0 * ln.data.radius, 10000.0)
        .motor_model(JointAxis::AngX, MotorModel::ForceBased)
        .motor_model(JointAxis::AngY, MotorModel::ForceBased)
        .motor_model(JointAxis::AngZ, MotorModel::ForceBased);
    commands
        .entity(section)
        .insert(ImpulseJoint::new(prev_joint, rapier_joint));
    // commands.entity(prev_section).push_children(&[section]); // Is this needed?

    if ln.next.is_some() {
        let next = Rc::clone(ln.next.as_ref().unwrap());
        build_joints(commands, &next.borrow_mut(), section);
    }
}

fn setup_scene(mut commands: Commands) {
    /* Create the ground. */
    commands
        .spawn()
        .insert(Collider::cuboid(10.0, 0.1, 10.0))
        .insert(Transform::from_xyz(0.0, -2.0, 0.0));

    /* Create the bouncing ball. */
    commands
        .spawn()
        .insert(RigidBody::Dynamic)
        .insert(Collider::ball(1.0))
        .insert(Restitution::coefficient(0.7))
        .insert(Transform::from_xyz(0.1, (6.0 as f32) * 4.0 + 4.0, 0.1));

    commands.spawn_bundle(PerspectiveCameraBundle {
        transform: Transform::from_xyz(-13.0 / 1.0, 24.0 / 1.0, 17.0 / 1.0)
            .looking_at(Vec3::new(0.0, 8.0, 0.0), Vec3::Y),
        ..default()
    });
}
