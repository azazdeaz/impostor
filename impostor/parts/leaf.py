from dataclasses import dataclass, field
from typing import Iterable

import numpy as np
import rerun as rr
from rerun.any_value import AnyBatchValue
from scipy.spatial.transform import Rotation

from impostor.components import Entity
import impostor.components as comp
import impostor.parts as parts
from impostor.parts.core import BasePart
from impostor.plant import Plant
from impostor.utils import Curve



    
@dataclass
class Leaf(BasePart):
    attachment_parent_entity: Entity

    midrib_entitiy_count: int = 12
    midrib_length: float = 0.6
    lateral_vein_count: int = 5
    lateral_vein_length: float = 0.32
    lateral_vein_entitiy_count: int = 5
    vein_length_multiplier: Curve | None = None
    attacment_orientation: comp.AttachmentOrientation = field(default_factory=comp.AttachmentOrientation)

    base_entity: Entity | None = None
    midrib_entities: list[Entity] = field(default_factory=list)
    lateral_vein_entities: list[Entity] = field(default_factory=list)
    lateral_vein_bases_left: list[Entity] = field(default_factory=list)
    lateral_vein_bases_right: list[Entity] = field(default_factory=list)

    def step(self, plant: Plant, entity: Entity):
        if self.base_entity is None:
            self.base_entity = entity
            self.initialize(plant)

    def initialize(self, plant: Plant):
        # Create the attachment point for the leaf
        plant.add_components(
            self.base_entity,
            comp.LeafAttachment(),
            parts.Vascular(radius=0.005, type=parts.VascularType.MIDRIB),
            parts.Mass(),
            self.attacment_orientation,
        )
        print(f"Creating Leaf {self.base_entity}, {self.attacment_orientation}")

        # Register the attachment point with the parent entity
        base_entity_attachments = plant.get_components(
            self.attachment_parent_entity
        ).get_or_create_by_type(comp.Attachments)
        base_entity_attachments.attachments.append(self.base_entity)

        # Setup physics between the attachment point and the parent entity
        plant.create_entity(
            comp.Spring(
                self.attachment_parent_entity,
                self.base_entity,
                # fixed_angle_stiffness=True,
                # angle_stiffness=1.0,
            )
        )

        # Initialize the midrib
        for i in range(self.midrib_entitiy_count):
            prev_entity = self.base_entity if i == 0 else self.midrib_entities[i - 1]
            new_entity = plant.create_entity(
                parts.Vascular(
                    radius=0.005,
                    type=parts.VascularType.MIDRIB,
                ),
                parts.Mass(),
                parts.GrowthPlan(
                    length_end=self.midrib_length / self.midrib_entitiy_count,
                    rotation_start=Rotation.from_euler("xyz", [12, 0, 0], degrees=True),
                    rotation_end=Rotation.from_euler("xyz", [1, 0, 0], degrees=True),
                    radius_end=0.005,
                ),
                comp.AxePrev(prev_entity),
            )
            self.midrib_entities.append(new_entity)
            plant.add_components(prev_entity, comp.AxeNext(new_entity))
            plant.create_entity(
                comp.Spring(prev_entity, new_entity, fixed_angle_stiffness=True)
            )

        # Initialize the lateral veins
        for i in range(self.lateral_vein_count):
            entity_length = self.lateral_vein_length / self.lateral_vein_entitiy_count

            if self.vein_length_multiplier is not None:
                t = i / self.lateral_vein_count
                entity_length *= self.vein_length_multiplier.evaluate(t)

            pos_on_midrib = i / self.lateral_vein_count
            index_on_midrib = int(pos_on_midrib * self.midrib_entitiy_count)
            midrib_attachment_entity = self.midrib_entities[index_on_midrib]
            attachments = plant.get_components(
                midrib_attachment_entity
            ).get_or_create_by_type(comp.Attachments)

            print(
                f"Creating lateral vein {i} with length {entity_length} on midrib #{index_on_midrib} {midrib_attachment_entity}"
            )

            for is_left in [True, False]:
                inclanation = np.deg2rad(60 - 25 * pos_on_midrib)
                if is_left:
                    inclanation = -inclanation
                base_entity = plant.create_entity(
                    parts.Vascular(     
                        radius=0.005,
                        type=parts.VascularType.VEIN,
                    ),
                    parts.Mass(),
                    comp.AttachmentOrientation(inclination=inclanation, azimuth=np.deg2rad(0)),
                    parts.GrowthPlan(
                        length_end=entity_length,
                        radius_end=0.004,
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

                self.lateral_vein_entities.append(base_entity)
                if is_left:
                    self.lateral_vein_bases_left.append(base_entity)
                else:
                    self.lateral_vein_bases_right.append(base_entity)

                prev_entity = base_entity
                for _ in range(self.lateral_vein_entitiy_count):
                    new_entity = plant.create_entity(
                        parts.Vascular(
                            radius=0.005,
                            type=parts.VascularType.VEIN,
                        ),
                        parts.GrowthPlan(
                            length_end=entity_length,
                            rotation_start=Rotation.from_euler(
                                "xyz", [12, 0, 0], degrees=True
                            ),
                            rotation_end=Rotation.from_euler(
                                "xyz", [1, 0, 0], degrees=True
                            ),
                            radius_end=0.003,
                        ),
                        comp.AxePrev(prev_entity),
                    )
                    plant.add_components(prev_entity, comp.AxeNext(new_entity))
                    plant.create_entity(
                        comp.Spring(
                            prev_entity,
                            new_entity,
                            # fixed_angle_stiffness=True,
                            # angle_stiffness=1.0,
                        )
                    )
                    self.lateral_vein_entities.append(new_entity)
                    prev_entity = new_entity