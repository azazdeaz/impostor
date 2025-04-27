from dataclasses import dataclass, field
from typing import Iterable

import numpy as np
import rerun as rr
from rerun.any_value import AnyBatchValue
from scipy.spatial.transform import Rotation

from impostor.components import Entity
import impostor.components as comp
from impostor.parts.core import BasePart
import impostor.parts as parts
from impostor.plant import Plant
from impostor.utils import Curve


@dataclass
class StrawberryStem(rr.AsComponents, BasePart):
    _initialized: bool = False
    petiole_length: float = 1.15
    petiole_entity_count: int = 9
    leaf_size: float = 1.0

    _petiole_entities: list[Entity] = field(default_factory=list)

    def step(self, plant: Plant, parent_entity: Entity):
        if self._initialized:
            return

        self._initialized = True

        initialize_strawberry_stem(
            plant,
            parent_entity,
            petiole_length=self.petiole_length,
            petiole_entity_count=self.petiole_entity_count,
            leaf_size=self.leaf_size,
        )

    def as_component_batches(self) -> Iterable[rr.ComponentBatchLike]:
        return [
            AnyBatchValue("comps.StrawberryStem.petiole_length", self.petiole_length),
            AnyBatchValue(
                "comps.StrawberryStem.petiole_entity_count", self.petiole_entity_count
            ),
        ]


def initialize_strawberry_stem(
    plant: Plant,
    parent_entity: Entity,
    petiole_length: float = 1.15,
    petiole_entity_count: int = 9,
    leaf_size: float = 1.0,
) -> Entity:
    # Create the petiole
    _petiole_entities = [parent_entity]
    for i in range(petiole_entity_count - 1):
        _petiole_entities.append(plant.create_entity())

    for i, petiole in enumerate(_petiole_entities):
        length = petiole_length / petiole_entity_count
        radius = 0.008 - 0.006 * i / petiole_entity_count
        plant.add_components(
            petiole,
            parts.GrowthPlan(
                length_end=length,
                radius_end=radius,
                rotation_end=Rotation.from_euler("xyz", [7, 0, 0], degrees=True),
                rotation_start=Rotation.from_euler("xyz", [3, 0, 0], degrees=True),
            ),
            parts.GrowthConfig(
                role=parts.GrowthConfig.Role.STEM,
                divisions=petiole_entity_count,
                petiole_no=i,
            ),
            parts.Vascular(length=0.001, radius=0.001),
            parts.Collider(),
            parts.Mass(),
        )

        if i < petiole_entity_count - 1:
            next_petiole = _petiole_entities[i + 1]
            plant.add_components(petiole, comp.AxeNext(next_petiole))
            plant.create_entity(parts.Spring(petiole, next_petiole))

        if i > 0:
            prev_petiole = _petiole_entities[i - 1]
            plant.add_components(petiole, comp.AxePrev(prev_petiole))

    # Add leaf
    curve = Curve(
        [(0, 0), (0.14, 0.6), (0.5, 0.7), (0.72, 0.4), (0.92, 0.12), (1, 0.0)]
    )
    plant.create_entity(
        parts.Leaf(
            attachment_parent_entity=_petiole_entities[-1],
            # attacment_orientation=comp.AttachmentOrientation(inclination=np.deg2rad(-30), azimuth=np.deg2rad(90)),
            vein_length_multiplier=curve,
            midrib_length=0.6 * leaf_size,
            lateral_vein_length=0.32 * leaf_size,
        )
    )
    plant.create_entity(
        parts.Leaf(
            attachment_parent_entity=_petiole_entities[-1],
            attacment_orientation=comp.AttachmentOrientation(
                inclination=np.deg2rad(-90), azimuth=np.deg2rad(0)
            ),
            vein_length_multiplier=curve,
            midrib_length=0.6 * leaf_size * 0.9,
            lateral_vein_length=0.32 * leaf_size * 0.9,
        )
    )
    plant.create_entity(
        parts.Leaf(
            attachment_parent_entity=_petiole_entities[-1],
            attacment_orientation=comp.AttachmentOrientation(
                inclination=np.deg2rad(90), azimuth=np.deg2rad(0)
            ),
            vein_length_multiplier=curve,
            midrib_length=0.6 * leaf_size * 0.9,
            lateral_vein_length=0.32 * leaf_size * 0.9,
        )
    )
