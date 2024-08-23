from impostor.components.core import AxeNext, AxePrev, Root, Stem
from impostor.components.rigid_transformation import RigidTransformation
from impostor.plant import Entity, Plant, Query

import numpy as np


def start_root(plant: Plant):
    q = plant.query()
    print(q)
    print(Query)
    root = q.without_component(AxePrev).single()
    if root is None:
        root = plant.create_entity(Root(), Stem())
    return root


def grow_system(plant: Plant, entity: Entity):
    comps = plant.get_components(entity)
    if Stem in comps:
        stem = comps.get_by_type(Stem)
        if stem.length < 1:
            stem.length += 0.4
        else:
            if AxeNext in comps:
                grow_system(plant, comps.get_by_type(AxeNext).next)
            else:
                next = plant.create_entity(Stem(), AxePrev(entity))
                comps.add(AxeNext(next))
        stem = comps.get_by_type(Stem)
        stem.radius *= 1.001


def add_transforms_system(plant: Plant, entity: Entity, transform: RigidTransformation | None = None):
    comps = plant.get_components(entity)
    if transform is None:
        transform = RigidTransformation.from_x_translation(0.2)
        
    if Stem in comps:
        stem = comps.get_by_type(Stem)
        transform = transform.combine(RigidTransformation.from_z_translation(stem.length / 2.0))
        comps.add(transform)

        if AxeNext in comps:
            next_transform = RigidTransformation()
            next_transform = transform = transform.combine(RigidTransformation.from_z_translation(stem.length / 2.0))
            add_transforms_system(plant, comps.get_by_type(AxeNext).next, next_transform)
    else:
        raise ValueError("Entity does not have a stem component")
