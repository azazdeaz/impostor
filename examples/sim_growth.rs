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
        data: StemSection { order: 0, length: 0.2, direction: Quat::default() },
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
        write!(f, "Stem {}{}", self.data.length, next)
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
    direction: Quat,
}
impl Default for StemSection {
    fn default() -> Self {
        Self { order: 0, length: 1.0, direction: Quat::default() }
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
    println!("__length {}", ln.data.length);
}


fn extend(ln: &mut LNode) {
    if ln.data.length > 0.8 && ln.next.is_none() {
        ln.next = Some(Rc::new(RefCell::new(LNode{
            next: None,
            prev: None,
            branches: Vec::new(),
            data: StemSection { order: ln.data.order, length: 0.2, direction: Quat::default() }
        })));
    }
}



