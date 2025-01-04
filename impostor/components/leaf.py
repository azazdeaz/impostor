from dataclasses import dataclass
from impostor.components import Entity
from impostor.plant import Plant
import impostor.components as comp
import numpy as np


@dataclass
class LeafMeta:
    base_entity: Entity
    midrib_entities: list[Entity]
    lateral_vein_bases_left: list[Entity]
    lateral_vein_bases_right: list[Entity]

    def __init__(self, plant: Plant, attachment_parent_entity: Entity):
        self.base_entity = plant.create_entity(
            comp.LeafAttachment(),
            comp.Vascular(radius=0.005, type=comp.VascularType.MIDRIB),
        )
        self.midrib_entities = []
        self.lateral_vein_bases_left = []
        self.lateral_vein_bases_right = []

        base_entity_attachments = plant.get_components(
            attachment_parent_entity
        ).get_or_create_by_type(comp.Attachments)
        base_entity_attachments.attachments.append(self.base_entity)
        plant.create_entity(
            comp.Spring(
                attachment_parent_entity,
                self.base_entity,
                fixed_angle_stiffness=True,
                angle_stiffness=1.0,
            )
        )

        print(
            f"LeafMeta created with attachment_parent_entity: {attachment_parent_entity}"
        )

        base_entity_attachments.attachments.append(self.base_entity)
        plant.create_entity(
            comp.Spring(
                attachment_parent_entity,
                self.base_entity,
                fixed_angle_stiffness=True,
                angle_stiffness=1.0,
            )
        )

        print(
            f"LeafMeta created with attachment_parent_entity: {attachment_parent_entity}"
        )

        midrib_entitiy_count = 12
        midrib_length = 0.12
        lateral_vein_count = 5
        lateral_vein_length = 0.05
        lateral_vein_entitiy_count = 5

        for i in range(midrib_entitiy_count):
            prev_entity = self.base_entity if i == 0 else self.midrib_entities[i - 1]
            new_entity = plant.create_entity(
                comp.Vascular(
                    radius=0.005,
                    type=comp.VascularType.MIDRIB,
                    length=midrib_length / midrib_entitiy_count,
                ),
                comp.AxePrev(prev_entity),
            )
            self.midrib_entities.append(new_entity)
            plant.add_components(prev_entity, comp.AxeNext(new_entity))
            plant.create_entity(
                comp.Spring(prev_entity, new_entity, fixed_angle_stiffness=True)
            )

        for i in range(lateral_vein_count):
            for is_left in [True, False]:
                pos_on_midrib = (i - 1) / lateral_vein_count
                index_on_midrib = int(pos_on_midrib * midrib_entitiy_count)
                midrib_attachment_entity = self.midrib_entities[index_on_midrib]
                attachments = plant.get_components(
                    midrib_attachment_entity
                ).get_or_create_by_type(comp.Attachments)

                azimuth = -np.pi / 4 if is_left else np.pi / 4
                base_entity = plant.create_entity(
                    comp.Vascular(
                        radius=0.005,
                        type=comp.VascularType.VEIN,
                        length=lateral_vein_length / lateral_vein_count,
                    ),
                    comp.VeinAttachment(),
                    comp.AttachmentOrientation(
                        inclination=-np.pi / 40, azimuth=azimuth
                    ),
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

                if is_left:
                    self.lateral_vein_bases_left.append(base_entity)
                else:
                    self.lateral_vein_bases_right.append(base_entity)

                prev_entity = base_entity
                for _ in range(lateral_vein_entitiy_count):
                    new_entity = plant.create_entity(
                        comp.Vascular(
                            radius=0.005,
                            type=comp.VascularType.VEIN,
                            length=lateral_vein_length / lateral_vein_entitiy_count,
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
                    prev_entity = new_entity

        print("LeafMeta created")
        print(f"midrib_entities: {self.midrib_entities}")
