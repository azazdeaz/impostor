from impostor.components.core import AxeNext, AxePrev, Root, Stem
from impostor.components.rigid_transformation import RigidTransformation
from impostor.plant import Entity, Plant, Query
from scipy.spatial.transform._rotation import Rotation

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
                rotation = Rotation.from_euler("xyz", [0, np.pi / 8, 0])
                next = plant.create_entity(Stem(rotation=rotation), AxePrev(entity))
                comps.add(AxeNext(next))
        stem = comps.get_by_type(Stem)
        stem.radius *= 1.001