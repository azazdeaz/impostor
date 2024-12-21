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
        plant.add_components(root, comp.AxeNext(meristem))
        plant.create_entity(comp.Spring(root, meristem))
    return root


def grow_system(plant: Plant):
    max_stem_length = 0.1
    growth_rate = 0.04

    for tip in plant.query().with_component(comp.GrowthTip).entities():
        print(f"Growing tip: {tip}")
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
                plant.add_components(tip, comp.AxeNext(new_tip), comp.Vascular(length=0.01, radius=0.02))
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
        # relaxeds = set()

        for _ in range(self.max_iterations):
            # if len(relaxeds) == len(spring_entities):
            #     break
            spring_entities_sample = random.sample(spring_entities.items(), len(spring_entities))
            for _, spring_comps in spring_entities_sample:
                # if entity in relaxeds:
                #     continue

                spring = spring_comps.get_by_type(comp.Spring)
                comps_a = plant.get_components(spring.entity_a)
                comps_b = plant.get_components(spring.entity_b)

                if Vascular in comps_a:
                    stem_a = comps_a.get_by_type(Vascular)
                    if not RigidTransformation in comps_a:
                        comps_a.add(RigidTransformation())
                    transform_a = comps_a.get_by_type(RigidTransformation)
                    transform_a_b = RigidTransformation.from_rotation(transform_a.rotation).combine(
                        RigidTransformation.from_z_translation(stem_a.length)
                    )

                    if not RigidTransformation in comps_b:
                        comps_b.add(transform_a.combine(transform_a_b))
                    
                    transform_b = comps_b.get_by_type(RigidTransformation)

                    distance = np.linalg.norm(transform_a.translation - transform_b.translation)
                    distance_stress =  stem_a.length - distance
                    
                    if distance == 0:
                        distance = 0.0001
                        direction = transform_a.rotation.apply([0, 0, 1])
                    else:
                        direction = transform_a_b.translation / distance
                    
                    step = distance_stress
                    if step > self.max_distance_step:
                        step = self.max_distance_step
                    spring.length += step
                    step = step * direction
                    
                    full_weight = spring.weight_a + spring.weight_b
                    transform_a.translation -= step / 2 * spring.weight_b / full_weight
                    transform_b.translation += step / 2 * spring.weight_a / full_weight                    

                    # if distance_stress <= self.distance_relaxed_treshold:
                    #     relaxeds.add(entity)
                    #     continue



                

@dataclass
class BranchingSystem:
    internode_spacing: NormalDistribution

    def execute(self, plant: Plant):
        apices = (
            plant.query()
            .without_component(AxeNext)
            .with_component(Vascular)
            .with_component(AxePrev)
            ._entities
        )
        for apex in apices:
            spacing = self.internode_spacing.sample()
            if self.length_without_branches(plant, apex) >= spacing:
                comps = plant.get_components(apex)
                stem = comps.get_by_type(comp.Vascular)
                branch = plant.create_entity(
                    comp.Vascular(radius=stem.radius * 0.8), comp.Branch(inclination=np.pi / 4)
                )
                growth_tip = plant.create_entity(comp.GrowthTip(), comp.AxePrev(branch))
                plant.add_components(branch, comp.AxeNext(growth_tip))
                comps.add(Branches([branch]))
                plant.create_entity(comp.Spring(apex, branch, angle=Rotation.from_euler("xyz", [0, np.pi / 4, 0])))

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
