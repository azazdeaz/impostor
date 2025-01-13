import random
from dataclasses import dataclass

import numpy as np

import impostor.components as comp
from impostor.plant import Entity, Plant
from impostor.utils import NormalDistribution, Curve
from scipy.spatial.transform import Rotation


@dataclass
class StartLeafSystem:
    branch_order: int = 0
    at_stem_length: float = NormalDistribution

    def execute(self, plant: Plant):
        for apex, components in plant.query().with_component(comp.GrowthTip).items():
            # Check if the branch order matches
            if components.get_by_type(comp.GrowthTip).branch_order != self.branch_order:
                continue
            
            # Check if the stem is long enough
            length = self.stem_length(plant, apex)
            if length < self.at_stem_length.sample():
                continue

            curve = Curve(
                [(0, 0), (0.14, 0.6), (0.7, 0.7), (0.86, 0.4), (0.98, 0.1), (1, 0.0)]
            )
            plant.create_entity(comp.LeafMeta(apex, vein_length_multiplier=curve))
            plant.remove_components(apex, comp.GrowthTip)

    def stem_length(self, plant: Plant, entity: Entity, sum=0.0):
        comps = plant.get_components(entity)
        if comp.BranchAttachment in comps:
            return sum

        if comp.Vascular in comps:
            stem = comps.get_by_type(comp.Vascular)
            sum += stem.length

        if comp.AxePrev in comps:
            return self.stem_length(plant, comps.get_by_type(comp.AxePrev).prev, sum)
        return sum


@dataclass
class GrowLeafSystem:
    def execute(self, plant: Plant):
        for _, components in plant.query().with_component(comp.LeafMeta).items():
            meta = components.get_by_type(comp.LeafMeta)
            meta.growth_stage = np.minimum(1.0, meta.growth_stage + 0.04)

            # Initialize the leaf
            if meta.base_entity is None:
                self.initialize_leaf(plant, meta)

            # Update the lengths of the leaf components
            self.update_lengths(plant, meta)

    def update_lengths(self, plant: Plant, meta: comp.LeafMeta):
        for midrib_entity in meta.midrib_entities:
            vascular = plant.get_components(midrib_entity).get_by_type(comp.Vascular)
            growth_plan = plant.get_components(midrib_entity).get_by_type(
                comp.GrowthPlan
            )
            vascular.length = growth_plan.get_length_at(meta.growth_stage)
            vascular.rotation = growth_plan.get_rotation_at(meta.growth_stage)

        for base_entity in meta.lateral_vein_entities:
            vascular = plant.get_components(base_entity).get_by_type(comp.Vascular)
            growth_plan = plant.get_components(base_entity).get_by_type(comp.GrowthPlan)
            vascular.length = growth_plan.get_length_at(meta.growth_stage)
            vascular.rotation = growth_plan.get_rotation_at(meta.growth_stage)

    def initialize_leaf(self, plant: Plant, meta: comp.LeafMeta):
        print("Creating leaf")
        # Create the attachment point for the leaf
        meta.base_entity = plant.create_entity(
            comp.LeafAttachment(),
            comp.Vascular(radius=0.005, type=comp.VascularType.MIDRIB),
        )

        # Register the attachment point with the parent entity
        base_entity_attachments = plant.get_components(
            meta.attachment_parent_entity
        ).get_or_create_by_type(comp.Attachments)
        base_entity_attachments.attachments.append(meta.base_entity)

        # Setup physics between the attachment point and the parent entity
        plant.create_entity(
            comp.Spring(
                meta.attachment_parent_entity,
                meta.base_entity,
                fixed_angle_stiffness=True,
                angle_stiffness=1.0,
            )
        )

        # Initialize the midrib
        for i in range(meta.midrib_entitiy_count):
            prev_entity = meta.base_entity if i == 0 else meta.midrib_entities[i - 1]
            new_entity = plant.create_entity(
                comp.Vascular(
                    radius=0.005,
                    type=comp.VascularType.MIDRIB,
                ),
                comp.GrowthPlan(
                    length_end=meta.midrib_length / meta.midrib_entitiy_count,
                    rotation_start=Rotation.from_euler("xyz", [12, 0, 0], degrees=True),
                    rotation_end=Rotation.from_euler("xyz", [1, 4, 0], degrees=True),
                ),
                comp.AxePrev(prev_entity),
            )
            meta.midrib_entities.append(new_entity)
            plant.add_components(prev_entity, comp.AxeNext(new_entity))
            plant.create_entity(
                comp.Spring(prev_entity, new_entity, fixed_angle_stiffness=True)
            )

        # Initialize the lateral veins
        for i in range(meta.lateral_vein_count):
            entity_length = meta.lateral_vein_length / meta.lateral_vein_entitiy_count

            if meta.vein_length_multiplier is not None:
                print(f"Using vein length multiplier at {i}")
                print(f"Length before: {entity_length}")
                t = i / meta.lateral_vein_count
                entity_length *= meta.vein_length_multiplier.evaluate(t)
                print(f"entity_length after: {entity_length}")

            for is_left in [True, False]:
                pos_on_midrib = (i - 1) / meta.lateral_vein_count
                index_on_midrib = int(pos_on_midrib * meta.midrib_entitiy_count)
                midrib_attachment_entity = meta.midrib_entities[index_on_midrib]
                attachments = plant.get_components(
                    midrib_attachment_entity
                ).get_or_create_by_type(comp.Attachments)

                azimuth = -np.pi * 0.4 if is_left else np.pi * 0.4
                base_entity = plant.create_entity(
                    comp.Vascular(
                        radius=0.005,
                        type=comp.VascularType.VEIN,
                    ),
                    comp.AttachmentOrientation(inclination=-np.pi / 4, azimuth=azimuth),
                    comp.GrowthPlan(
                        length_end=entity_length,
                    ),
                    comp.VeinAttachment(),
                )

                attachments.attachments.append(base_entity)

                plant.create_entity(
                    comp.Spring(
                        midrib_attachment_entity,
                        base_entity,
                        fixed_angle_stiffness=True,
                        angle_stiffness=1.0,
                    )
                )

                meta.lateral_vein_entities.append(base_entity)
                if is_left:
                    meta.lateral_vein_bases_left.append(base_entity)
                else:
                    meta.lateral_vein_bases_right.append(base_entity)

                prev_entity = base_entity
                for _ in range(meta.lateral_vein_entitiy_count):
                    new_entity = plant.create_entity(
                        comp.Vascular(
                            radius=0.005,
                            type=comp.VascularType.VEIN,
                        ),
                        comp.GrowthPlan(
                            length_end=entity_length,
                            rotation_start=Rotation.from_euler(
                                "xyz", [12, 0, 0], degrees=True
                            ),
                            rotation_end=Rotation.from_euler(
                                "xyz", [1, 0, 0], degrees=True
                            ),
                        ),
                        comp.AxePrev(prev_entity),
                    )
                    plant.add_components(prev_entity, comp.AxeNext(new_entity))
                    plant.create_entity(
                        comp.Spring(
                            prev_entity,
                            new_entity,
                            fixed_angle_stiffness=True,
                            angle_stiffness=1.0,
                        )
                    )
                    meta.lateral_vein_entities.append(new_entity)
                    prev_entity = new_entity
