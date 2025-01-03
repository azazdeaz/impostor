import random
from dataclasses import dataclass

import numpy as np

import impostor.components as comp
from impostor.plant import Entity, Plant
from impostor.utils import NormalDistribution


@dataclass
class BranchingSystem:
    internode_spacing: NormalDistribution
    max_branch_order: int = 2

    def execute(self, plant: Plant):
        apices = plant.query().with_component(comp.GrowthTip)._entities
        for apex in apices:
            apex_comps = plant.get_components(apex)
            branch_order = apex_comps.get_by_type(comp.GrowthTip).branch_order
            if branch_order >= self.max_branch_order:
                continue

            spacing = self.internode_spacing.sample()
            if self.length_without_branches(plant, apex) >= spacing:
                last_stem = apex_comps.get_by_type(comp.AxePrev).prev
                comps = plant.get_components(last_stem)
                stem = comps.get_by_type(comp.Vascular)
                branches = comp.Attachments()
                orientation1 = comp.AttachmentOrientation(
                    inclination=np.pi / 4, azimuth=random.random() * np.pi * 2
                )
                orientation2 = comp.AttachmentOrientation(
                    inclination=np.pi / 4, azimuth=orientation1.azimuth + np.pi
                )
                for orientation in [orientation1, orientation2]:
                    branch = plant.create_entity(
                        comp.Vascular(radius=stem.radius * 0.8),
                        comp.BranchAttachment(),
                        orientation,
                    )

                    growth_tip = plant.create_entity(
                        comp.GrowthTip(branch_order=branch_order + 1),
                        comp.AxePrev(branch),
                    )
                    plant.add_components(branch, comp.AxeNext(growth_tip))
                    plant.create_entity(
                        comp.Spring(
                            last_stem,
                            branch,
                            angle=orientation.as_rotation(),
                        )
                    )
                    plant.create_entity(comp.Spring(branch, growth_tip))
                    branches.attachments.append(branch)
                comps.add(branches)

    def length_without_branches(self, plant: Plant, entity: Entity, sum=0.0):
        comps = plant.get_components(entity)
        if comp.Attachments in comps:
            return sum

        if comp.Vascular in comps:
            stem = comps.get_by_type(comp.Vascular)
            sum += stem.length

        if comp.AxePrev in comps:
            return self.length_without_branches(
                plant, comps.get_by_type(comp.AxePrev).prev, sum
            )
        return sum
