import numpy as np
from scipy.spatial.transform._rotation import Rotation

from impostor.components.core import AxeNext, AxePrev, Branch, Branches, Root, Stem
from impostor.components.rigid_transformation import RigidTransformation
from impostor.plant import Entity, Plant, Query
from impostor.utils import NormalDistribution


def start_root(plant: Plant):
    q = plant.query()
    root = q.without_component(AxePrev).single()
    if root is None:
        root = plant.create_entity(Root(), Stem())
    return root


def grow_system(plant: Plant, entity: Entity):
    max_stem_length = 0.1
    growth_rate = 0.04
    comps = plant.get_components(entity)
    if Stem in comps:
        stem = comps.get_by_type(Stem)
        if stem.length < max_stem_length:   
            stem.length += growth_rate
        else:
            # Grow until there is a next entity
            if AxeNext in comps:
                grow_system(plant, comps.get_by_type(AxeNext).next)
            else:
                rotation = Rotation.from_euler("xyz", [0, np.pi / 12 * stem.length, 0])
                next = plant.create_entity(Stem(rotation=rotation, length=0.01, radius=0.02), AxePrev(entity))
                comps.add(AxeNext(next))

            if Branches in comps:
                for branch in comps.get_by_type(Branches).branches:
                    grow_system(plant, branch)

        stem = comps.get_by_type(Stem)
        stem.radius *= 1.001


def branch_system(
    plant: Plant,
    entity: Entity,
    internode_spacing: NormalDistribution,
    length_without_branches=0.0,
):
    comps = plant.get_components(entity)
    if Stem in comps:
        stem = comps.get_by_type(Stem)
        length_without_branches += stem.length

        spacing = internode_spacing.sample()
        if Branches in comps:
            length_without_branches = 0.0
        elif length_without_branches >= spacing:
            length_without_branches -= spacing

            branch = plant.create_entity(Stem(radius=stem.radius * 0.8), Branch(inclination=np.pi / 4))
            comps.add(Branches([branch]))

        if AxeNext in comps:
            branch_system(
                plant,
                comps.get_by_type(AxeNext).next,
                internode_spacing,
                length_without_branches,
            )
