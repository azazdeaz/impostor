from dataclasses import asdict
from typing import List, Optional

import rerun as rr
from scipy.spatial.transform._rotation import Rotation

import impostor.messages as messages
from impostor.components.core import AxeNext, AxePrev, Branch, Branches, Stem
from impostor.components.rigid_transformation import (
    ApexTransformation,
    RigidTransformation,
)
from impostor.plant import Entity, Plant


def add_transforms_system(
    plant: Plant, entity: Entity, base_transform: RigidTransformation | None = None
):
    comps = plant.get_components(entity)
    if base_transform is None:
        base_transform = RigidTransformation.from_x_translation(0.2)

    comps.add(base_transform)

    rr.log(f"nodes/{entity}", rr.Transform3D(
        translation=base_transform.translation, #rotation=base_transform.rotation.as_quat()
        quaternion=base_transform.rotation.as_quat()
    ))

    if Stem in comps:
        stem = comps.get_by_type(Stem)

        if stem.length <= 0:
            return

        if Branches in comps:
            for branch_entity in comps.get_by_type(Branches).branches:
                branch = plant.get_components(branch_entity).get_by_type(Branch)
                rotation = Rotation.from_euler("zyx", [branch.azimuth, branch.inclination, 0])
                branch_transform = base_transform.combine(RigidTransformation.from_rotation(rotation))
                add_transforms_system(plant, branch_entity, branch_transform)

        next_transform = base_transform.combine(
            RigidTransformation.from_z_translation(stem.length).rotate(
                stem.rotation
            )
        )
        if AxeNext in comps:
            add_transforms_system(
                plant, comps.get_by_type(AxeNext).next, next_transform
            )
            # Remove apex transform if it has one
            if ApexTransformation in comps:
                comps.remove(ApexTransformation)
        else:
            comps.add(ApexTransformation(next_transform))
    else:
        raise ValueError("Entity does not have a stem component")


def create_stem_mesh_data(
    plant: Plant, entity: Entity, rings: Optional[List[messages.StemRing]] = None
) -> List[messages.StemRing]:
    if rings is None:
        rings = []
    comps = plant.get_components(entity)
    for comp in comps:
        values = {}
        for key, value in asdict(comp).items():
            try:
                rr.any_value.AnyBatchValue(key, value)
                values[f"comp.{comp.__class__.__name__}.{key}"] = value
            except Exception as _:
                pass

        rr.log(f"nodes/{entity}", rr.AnyValues(**values))
        
    

    if Stem in comps and RigidTransformation in comps:
        stem = comps.get_by_type(Stem)
        transform = comps.get_by_type(RigidTransformation)
        ring = messages.StemRing(pose=transform.to_pose_message(), radius=stem.radius)
        rings.append(ring)
        if AxeNext in comps:
            return create_stem_mesh_data(plant, comps.get_by_type(AxeNext).next, rings)
    
    if ApexTransformation in comps:
        apex_transform = comps.get_by_type(ApexTransformation)
        ring = messages.StemRing(pose=apex_transform.transfrom.to_pose_message(), radius=0.001)
        rings.append(ring)

    return rings

def create_axes_mesh_data(
    plant: Plant, entity: Entity
) -> List[List[messages.StemRing]]:
    roots = collect_all_axis_roots(plant, entity)
    return [create_stem_mesh_data(plant, root) for root in roots]



def collect_all_axis_roots(plant: Plant, entity: Entity, roots: List[Entity] | None = None) -> List[Entity]:
    if roots is None:
        roots = []
    
    if plant.get_components(entity).get_by_type(AxePrev) is None:
        roots = roots + [entity]
    
    if AxeNext in plant.get_components(entity):
        roots = collect_all_axis_roots(plant, plant.get_components(entity).get_by_type(AxeNext).next, roots)

    if Branches in plant.get_components(entity):
        for branch in plant.get_components(entity).get_by_type(Branches).branches:
            roots = collect_all_axis_roots(plant, branch, roots)

    return roots
    
    