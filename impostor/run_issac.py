import os
import random
from dataclasses import dataclass

import numpy as np

from impostor.components.core import AxeNext, AxePrev, Vascular
from impostor.components.rigid_transformation import RigidTransformation
from impostor.plant import Entity, Plant
from impostor.run_plant import test_grow
from impostor.systems.core import add_transforms_system

os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCylinder, FixedCylinder
from pxr import Gf, UsdPhysics

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
world.reset()

plant, root = test_grow(15)
add_transforms_system(plant, root)

@dataclass
class PrimPath:
    path: str

def add_prims_system(plant, entity: Entity):
    comps = plant.get_components(entity)
    if Vascular in comps:
        stem = comps.get_by_type(Vascular)
        rigid_transform = comps.get_by_type(RigidTransformation)
        if rigid_transform is not None:
            Cls = FixedCylinder if AxePrev not in comps else DynamicCylinder
            prim = Cls(
                prim_path=f"/plant/stem_{entity}",
                position=rigid_transform.translation,
                orientation=rigid_transform.rotation.as_quat(),
                visible=True,
                height=stem.length,
                radius=stem.radius,
                color=np.array([random.uniform(0.5, 1), random.uniform(0.5, 1), random.uniform(0.5, 1)]),
            )
            comps.add(PrimPath(prim.prim_path))
            # result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cylinder")
            # prim = world.stage.GetPrimAtPath(path)
            # utils.setRigidBody(prim, "convexHull", False)
            # prim.GetAttribute("xformOp:translate").Set((0,0,rigid_transform.translation[2]))
            # prim.GetAttribute("xformOp:scale").Set((stem.radius, stem.radius, stem.length))
            
            if AxeNext in comps:
                add_prims_system(plant, comps.get_by_type(AxeNext).next)
        else:
            raise ValueError("Entity does not have a rigid transformation component")
    else:
        raise ValueError("Entity does not have a stem component")
    
def add_2d_joints_system(plant: Plant, entity: Entity):
    comps = plant.get_components(entity)
    if AxeNext in comps:
        next_axe = comps.get_by_type(AxeNext)
        next_comps = plant.get_components(next_axe.next)

        stem = comps.get_by_type(Vascular)
        next_stem = next_comps.get_by_type(Vascular)
        
        joint_prim_path = f"{comps.get_by_type(PrimPath).path}_joint"
        revoluteJoint = UsdPhysics.RevoluteJoint.Define(world.stage, joint_prim_path)

        revoluteJoint.CreateAxisAttr("X")
        revoluteJoint.CreateLowerLimitAttr(-30.0)
        revoluteJoint.CreateUpperLimitAttr(30)

        revoluteJoint.CreateBody0Rel().SetTargets([comps.get_by_type(PrimPath).path])
        revoluteJoint.CreateBody1Rel().SetTargets([next_comps.get_by_type(PrimPath).path])

        revoluteJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, stem.length / 2.0))
        revoluteJoint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

        revoluteJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, -next_stem.length / 2.0))
        revoluteJoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

        # add angular drive
        angularDriveAPI = UsdPhysics.DriveAPI.Apply(world.stage.GetPrimAtPath(joint_prim_path), "angular")
        angularDriveAPI.CreateTypeAttr("force")
        angularDriveAPI.CreateTargetPositionAttr(0.0)
        angularDriveAPI.CreateDampingAttr(1e10)
        angularDriveAPI.CreateStiffnessAttr(1e10)

        add_joints_system(plant, next_axe.next)

def add_joints_system(plant: Plant, entity: Entity):
    comps = plant.get_components(entity)
    if AxeNext in comps:
        next_axe = comps.get_by_type(AxeNext)
        next_comps = plant.get_components(next_axe.next)

        stem = comps.get_by_type(Vascular)
        next_stem = next_comps.get_by_type(Vascular)
        
        joint_prim_path = f"{comps.get_by_type(PrimPath).path}_joint"
        joint = UsdPhysics.SphericalJoint.Define(world.stage, joint_prim_path)

        joint.CreateAxisAttr("Z")
        joint.CreateConeAngle0LimitAttr(30.0)
        joint.CreateConeAngle1LimitAttr(30.0)

        joint.CreateBody0Rel().SetTargets([comps.get_by_type(PrimPath).path])
        joint.CreateBody1Rel().SetTargets([next_comps.get_by_type(PrimPath).path])

        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, stem.length / 2.0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))

        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, -next_stem.length / 2.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

        # /plant/stem_0_joint.physxLimit:cone:damping


        def add_drive(axis):
            angularDriveAPI = UsdPhysics.DriveAPI.Apply(world.stage.GetPrimAtPath(joint_prim_path), axis)
            angularDriveAPI.CreateTypeAttr("acceleration")
            angularDriveAPI.CreateTargetPositionAttr(0.0)
            angularDriveAPI.CreateDampingAttr(10000000.0)
            angularDriveAPI.CreateStiffnessAttr(100000000.0)
        add_drive("rotX")
        add_drive("rotY")

        add_joints_system(plant, next_axe.next)


add_prims_system(plant, root)
add_2d_joints_system(plant, root)

# result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cylinder")
# prim = world.stage.GetPrimAtPath(path)
# utils.setRigidBody(prim, "convexHull", False)
# prim.GetAttribute("xformOp:translate").Set((0, 0, 1.5))
# prim.GetAttribute("xformOp:scale").Set((0.2, 0.2, 0.5))

while simulation_app.is_running():
    world.step(render=True)
    if world.is_playing():
        pass
simulation_app.close()