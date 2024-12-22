import random
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform._rotation import Rotation

import impostor.components as comp
from impostor.components import (
    AxeNext,
    AxePrev,
    Branches,
    RigidTransformation,
    Vascular,
)
from impostor.plant import Entity, Plant
from impostor.utils import NormalDistribution


def start_root(plant: Plant):
    q = plant.query()
    root = q.without_component(AxePrev).single()
    if root is None:
        root = plant.create_entity(comp.Root(), comp.Vascular())
        meristem = plant.create_entity(comp.GrowthTip(), comp.AxePrev(root))
        plant.add_components(root, comp.AxeNext(meristem), comp.RigidTransformation())
        plant.create_entity(comp.Spring(root, meristem))
    return root


def grow_system(plant: Plant):
    max_stem_length = 0.1
    growth_rate = 0.04

    for tip in plant.query().with_component(comp.GrowthTip).entities():
        if not AxePrev in plant.get_components(tip):
            raise ValueError("Growth tip must have an AxePrev component")

        prev = plant.get_components(tip).get_by_type(AxePrev).prev

        prev_comps = plant.get_components(prev)
        if Vascular in prev_comps:
            stem = prev_comps.get_by_type(comp.Vascular)
            if stem.length < max_stem_length:
                stem.length += growth_rate
            else:
                # If the stem is at max length, convert the tip into a new stem and create a new tip
                new_tip = plant.create_entity(comp.GrowthTip(), comp.AxePrev(tip))
                plant.remove_components(tip, comp.GrowthTip)
                plant.add_components(
                    tip, comp.AxeNext(new_tip), comp.Vascular(length=0.01, radius=0.02)
                )
                plant.create_entity(comp.Spring(tip, new_tip))


@dataclass
class RelaxSpringSystem:
    max_iterations: int = 10
    max_distance_step: float = 10.1
    distance_relaxed_treshold: float = 0.0001

    def execute(self, plant: Plant):
        spring_entities = plant.query().with_component(comp.Spring)._entities
        if len(spring_entities) == 0:
            return

        # Entity to all its connected spring entities
        connection_map = {}

        # Build the connection map
        for spring_entity, spring_comps in spring_entities.items():
            spring = spring_comps.get_by_type(comp.Spring)
            if spring.entity_a not in connection_map:
                connection_map[spring.entity_a] = set()
            connection_map[spring.entity_a].add(spring_entity)
            if spring.entity_b not in connection_map:
                connection_map[spring.entity_b] = set()
            connection_map[spring.entity_b].add(spring_entity)

        self.update_spring_targets(plant)

        root_entity = plant.query().with_component(comp.Root).single()

        if root_entity not in connection_map:
            raise ValueError("Root entity is not connected to any spring entity")

        relaxeds = set()

        def relax_all_from(spring_entity: Entity):
            relaxeds.add(spring_entity)
            spring = plant.get_components(spring_entity).get_by_type(comp.Spring)
            self.relax_spring(plant, spring_entity)

            for spring_entity in (
                connection_map[spring.entity_a] | connection_map[spring.entity_b]
            ):
                if spring_entity not in relaxeds:
                    relax_all_from(spring_entity)

        for _ in range(self.max_iterations):
            relaxeds.clear()
            for spring_entity in connection_map[root_entity]:
                relax_all_from(spring_entity)

            if len(relaxeds) != len(spring_entities):
                print(connection_map)
                raise ValueError(
                    f"Only {len(relaxeds)} of {len(spring_entities)} springs relaxed. Graph is not connected."
                )

    def update_spring_targets(self, plant: Plant):
        spring_entities = plant.query().with_component(comp.Spring)._entities

        # Update the target length and angle of the springs
        for spring_entity, spring_comps in spring_entities.items():
            spring = spring_comps.get_by_type(comp.Spring)
            comps_a = plant.get_components(spring.entity_a)
            comps_b = plant.get_components(spring.entity_b)

            if (
                comp.AxePrev in comps_b
                and comps_b.get_by_type(comp.AxePrev).prev == spring.entity_a
            ):
                if comp.Vascular in comps_a:
                    stem_a = comps_a.get_by_type(comp.Vascular)
                    spring.length = stem_a.length
                    spring.angle = Rotation.identity()

            if comp.Branch in comps_b:
                branch = comps_b.get_by_type(comp.Branch)
                spring.length = 0
                spring.angle = branch.as_rotation()

            plant.add_components(spring_entity, spring)

    def relax_spring(self, plant: Plant, spring_entity):
        spring = plant.get_components(spring_entity).get_by_type(comp.Spring)
        comps_a = plant.get_components(spring.entity_a)
        comps_b = plant.get_components(spring.entity_b)

        transform_a = comps_a.get_by_type(RigidTransformation)
        transform_a_b = RigidTransformation.from_rotation(spring.angle).combine(
            RigidTransformation.from_z_translation(spring.length)
        )

        comps_b.add(transform_a.combine(transform_a_b))
        return  # TODO fix physics based growing

        if RigidTransformation not in comps_b:
            comps_b.add(transform_a.combine(transform_a_b))

        transform_b = comps_b.get_by_type(RigidTransformation)

        distance = np.linalg.norm(transform_a.translation - transform_b.translation)
        distance_stress = spring.length - distance

        if distance == 0:
            distance = 0.0001
            direction = transform_a.rotation.apply([0, 0, 1])
        else:
            direction = transform_a_b.translation / distance

        step = distance_stress
        if step > self.max_distance_step:
            step = self.max_distance_step
        step = step * direction

        full_weight = spring.weight_a + spring.weight_b
        transform_a.translation -= step * spring.weight_b / full_weight
        transform_b.translation += step * spring.weight_a / full_weight


@dataclass
class BranchingSystem:
    internode_spacing: NormalDistribution

    def execute(self, plant: Plant):
        apices = plant.query().with_component(comp.GrowthTip)._entities
        for apex in apices:
            spacing = self.internode_spacing.sample()
            if self.length_without_branches(plant, apex) >= spacing:
                last_stem = plant.get_components(apex).get_by_type(comp.AxePrev).prev
                comps = plant.get_components(last_stem)
                stem = comps.get_by_type(comp.Vascular)
                branches = comp.Branches()
                branch_comp1 = comp.Branch(
                    inclination=np.pi / 4, azimuth=random.random() * np.pi * 2
                )
                branch_comp2 = comp.Branch(
                    inclination=np.pi / 4, azimuth=branch_comp1.azimuth + np.pi
                )
                for branch_comp in [branch_comp1, branch_comp2]:
                    branch = plant.create_entity(
                        comp.Vascular(radius=stem.radius * 0.8),
                        branch_comp,
                    )

                    growth_tip = plant.create_entity(
                        comp.GrowthTip(), comp.AxePrev(branch)
                    )
                    plant.add_components(branch, comp.AxeNext(growth_tip))
                    plant.create_entity(
                        comp.Spring(
                            last_stem,
                            branch,
                            angle=branch_comp.as_rotation(),
                        )
                    )
                    plant.create_entity(comp.Spring(branch, growth_tip))
                    branches.branches.append(branch)
                comps.add(branches)

    def length_without_branches(self, plant: Plant, entity: Entity, sum=0.0):
        comps = plant.get_components(entity)
        if Branches in comps:
            return sum

        if Vascular in comps:
            stem = comps.get_by_type(comp.Vascular)
            sum += stem.length

        if AxePrev in comps:
            return self.length_without_branches(
                plant, comps.get_by_type(AxePrev).prev, sum
            )
        return sum
