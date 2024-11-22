from typing import List, Optional

import impostor.messages as messages
from impostor.components.core import AxeNext, Stem, AxePrev, Branches, Branch
from impostor.components.rigid_transformation import RigidTransformation
from impostor.plant import Entity, Plant
from scipy.spatial.transform._rotation import Rotation


def add_transforms_system(
    plant: Plant, entity: Entity, transform: RigidTransformation | None = None
):
    comps = plant.get_components(entity)
    if transform is None:
        transform = RigidTransformation.from_x_translation(0.2)

    if Stem in comps:
        stem = comps.get_by_type(Stem)
        transform = transform.combine(
            RigidTransformation.from_z_translation(stem.length / 2.0).with_rotation(
                stem.rotation
            )
        )
        comps.add(transform)

        if Branches in comps:
            for branch_entity in comps.get_by_type(Branches).branches:
                branch = plant.get_components(branch_entity).get_by_type(Branch)
                rotation = Rotation.from_euler("zyx", [branch.azimuth, branch.inclination, 0])
                branch_transform = transform.combine(RigidTransformation.from_rotation(rotation))
                plant.get_components(branch_entity).add(branch_transform)
                add_transforms_system(plant, branch_entity, branch_transform)

        if AxeNext in comps:
            next_transform = RigidTransformation()
            next_transform = transform.combine(
                RigidTransformation.from_z_translation(stem.length / 2.0)
            )
            add_transforms_system(
                plant, comps.get_by_type(AxeNext).next, next_transform
            )
        
            transform = next_transform
    else:
        raise ValueError("Entity does not have a stem component")


def create_stem_mesh_data(
    plant: Plant, entity: Entity, rings: Optional[List[messages.StemRing]] = None
) -> List[messages.StemRing]:
    if rings is None:
        rings = []
    comps = plant.get_components(entity)
    if Stem in comps and RigidTransformation in comps:
        stem = comps.get_by_type(Stem)
        transform = comps.get_by_type(RigidTransformation)
        ring = messages.StemRing(pose=transform.to_pose_message(), radius=stem.radius)
        rings.append(ring)
        if AxeNext in comps:
            return create_stem_mesh_data(plant, comps.get_by_type(AxeNext).next, rings)

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
    
    