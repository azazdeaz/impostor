from typing import List, Optional

import impostor.messages as messages
from impostor.components.core import AxeNext, Stem
from impostor.components.rigid_transformation import RigidTransformation
from impostor.plant import Entity, Plant


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

        if AxeNext in comps:
            next_transform = RigidTransformation()
            next_transform = transform = transform.combine(
                RigidTransformation.from_z_translation(stem.length / 2.0)
            )
            add_transforms_system(
                plant, comps.get_by_type(AxeNext).next, next_transform
            )
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
