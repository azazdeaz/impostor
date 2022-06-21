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
use std::{cell::RefCell, fmt};
use std::rc::Rc;

fn main() {
    // App::new()
    //     .insert_resource(Msaa { samples: 4 })
    //     .add_plugins(DefaultPlugins)
    //     .add_plugin(RapierPhysicsPlugin::<NoUserData>::default())
    //     .add_plugin(RapierDebugRenderPlugin::default())
    //     // .add_startup_system(setup_scene)
    //     .run();

    let mut plant = Rc::new(RefCell::new(LNode {
        data: StemSection { order: 0, length: 0.2, radius: 0.001, direction: Quat::default() },
        ..Default::default()
    }));

    println!(">> {:?}", plant.borrow());
    for _ in 0..12 {
    step(Rc::clone(&plant));
    println!(">> {:?}", plant.borrow());
    }
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
        LNode { next: None, prev: None, branches: Vec::new(), data: StemSection::default() }
    }
}
impl fmt::Debug for LNode {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let next = if let Some(next) = self.next.as_ref() {
            format!(" > ${:?}", next.borrow())
        }
        else {
            " |".to_string()
        };
        write!(f, "Stem {:.3?}/{:.3?}{}", self.data.length, self.data.radius, next)
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
    radius: f32,
    direction: Quat,
}
impl Default for StemSection {
    fn default() -> Self {
        Self { order: 0, length: 1.0, radius: 1.0, direction: Quat::default() }
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
    ln.data.length += (1.0 - ln.data.length) / 3.0;
    let prev_radius = if let Some(prev) = ln.prev { prev.borrow().data.radius} else {1.0};
    ln.data.radius += (prev_radius - ln.data.radius) / 12.0;
}


fn extend(ln: &mut LNode) {
    if ln.data.length > 0.8 && ln.next.is_none() {
        ln.next = Some(Rc::new(RefCell::new(LNode{
            next: None,
            prev: None,
            branches: Vec::new(),
            data: StemSection { order: ln.data.order, length: 0.2, radius: 0.001, direction: Quat::default() }
        })));
    }
}

fn setup_plant(
    mut commands: Commands,
    // mut meshes: ResMut<Assets<Mesh>>,
    // mut materials: ResMut<Assets<StandardMaterial>>,
    // mut skinned_mesh_inverse_bindposes_assets: ResMut<Assets<SkinnedMeshInverseBindposes>>,
    // asset_server: Res<AssetServer>,
)  {
    let root_joint = commands
        .spawn()
        .insert(RigidBody::Fixed)
        .insert(GlobalTransform::identity())
        .insert(Transform::from_xyz(0.0, 0.0, 0.0))
        .id();
}

fn build_joints(commands: &mut Commands,ln: &LNode, prev_joint: Entity) {
    let transform = Transform::from_xyz(0.0, ln.data.length, 0.0);
    let (rot_y, rot_z, rot_x) = transform.rotation.to_euler(EulerRot::YZX);
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
        let prev_section_height = if let Some(prev) = ln.prev { prev.borrow_mut().data.length} else {0.0};

        let rapier_joint = SphericalJointBuilder::new()
            .local_anchor1(Vec3::new(0.0, 0.0, 0.0))
            .local_anchor2(Vec3::new(0.0, ln.data.length, 0.0))
            .motor_position(JointAxis::AngX, rot_x, 90000.0 * ln.data.radius, 10000.0)
            .motor_position(JointAxis::AngY, rot_y, 90000.0 * ln.data.radius, 10000.0)
            .motor_position(JointAxis::AngZ, rot_z, 90000.0 * ln.data.radius, 10000.0)
            .motor_model(JointAxis::AngX, MotorModel::ForceBased)
            .motor_model(JointAxis::AngY, MotorModel::ForceBased)
            .motor_model(JointAxis::AngZ, MotorModel::ForceBased);
        commands
            .entity(section)
            .insert(ImpulseJoint::new(prev_joint, rapier_joint));
        // commands.entity(prev_section).push_children(&[section]); // Is this needed?

        if ln.next.is_some() {
            let next = Rc::clone(&ln.next.unwrap());
            // build_joints(commands, &next.borrow_mut(), section);
        }
    
}



