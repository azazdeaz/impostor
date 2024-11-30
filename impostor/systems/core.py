from dataclasses import dataclass
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
                next = plant.create_entity(
                    Stem(rotation=rotation, length=0.01, radius=0.02), AxePrev(entity)
                )
                comps.add(AxeNext(next))

            if Branches in comps:
                for branch in comps.get_by_type(Branches).branches:
                    grow_system(plant, branch)

        stem = comps.get_by_type(Stem)
        stem.radius *= 1.001


@dataclass
class BranchingSystem:
    internode_spacing: NormalDistribution

    def execute(self, plant: Plant):
        print("BranchingSystem")
        apices = (
            plant.query()
            .without_component(AxeNext)
            .with_component(Stem)
            .with_component(AxePrev)
            .entities
        )
        for apex in apices:
            spacing = self.internode_spacing.sample()
            if self.length_without_branches(plant, apex) >= spacing:
                comps = plant.get_components(apex)
                stem = comps.get_by_type(Stem)
                branch = plant.create_entity(
                    Stem(radius=stem.radius * 0.8), Branch(inclination=np.pi / 4)
                )
                comps.add(Branches([branch]))

    def length_without_branches(self, plant: Plant, entity: Entity, sum=0.0):
        comps = plant.get_components(entity)
        if Branches in comps:
            return sum

        if Stem in comps:
            stem = comps.get_by_type(Stem)
            sum += stem.length
            if AxePrev in comps:
                return self.length_without_branches(
                    plant, comps.get_by_type(AxePrev).prev, sum
                )
        return sum
