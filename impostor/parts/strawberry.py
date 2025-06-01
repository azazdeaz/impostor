from dataclasses import dataclass, field
from typing import Iterable

import numpy as np
import rerun as rr
from rerun.any_value import AnyBatchValue
from scipy.spatial.transform import Rotation, Slerp

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

        # Create the petiole
        self._petiole_entities = [parent_entity]
        for i in range(self.petiole_entity_count - 1):
            self._petiole_entities.append(plant.create_entity())
        
        for i, petiole in enumerate(self._petiole_entities):
            length = self.petiole_length / self.petiole_entity_count
            radius = 0.008 - 0.006 * i / self.petiole_entity_count
            plant.add_components(
                petiole,
                parts.GrowthPlan(length_end=length, radius_end=radius),
                parts.Vascular(length=0.001, radius=0.001),
                parts.Collider(),
                parts.Mass(),
            )

            if i < self.petiole_entity_count - 1:
                next_petiole = self._petiole_entities[i + 1]
                plant.add_components(petiole, comp.AxeNext(next_petiole))
                plant.create_entity(
                    parts.Spring(petiole, next_petiole)
                )
            
            if i > 0:
                prev_petiole = self._petiole_entities[i - 1]
                plant.add_components(petiole, comp.AxePrev(prev_petiole))

        # Add leaf
        curve = Curve(
            [(0, 0), (0.14, 0.6), (0.5, 0.7), (0.72, 0.4), (0.92, 0.12), (1, 0.0)]
        )
        print(f"!!!!!!!!!!!!!!!!!!!!!!Creating leaf with curve: {curve}")
        plant.create_entity(parts.Leaf(
            attachment_parent_entity=self._petiole_entities[-1],

            # attacment_orientation=comp.AttachmentOrientation(inclination=np.deg2rad(-30), azimuth=np.deg2rad(90)),
            vein_length_multiplier=curve,
            midrib_length=0.6 * self.leaf_size,
            lateral_vein_length=0.32 * self.leaf_size,
        ))
        plant.create_entity(parts.Leaf(
            attachment_parent_entity=self._petiole_entities[-1],
            attacment_orientation=comp.AttachmentOrientation(inclination=np.deg2rad(-90), azimuth=np.deg2rad(0)),
            vein_length_multiplier=curve,
            midrib_length=0.6 * self.leaf_size * 0.9,
            lateral_vein_length=0.32 * self.leaf_size * 0.9,
        ))
        plant.create_entity(parts.Leaf(
            attachment_parent_entity=self._petiole_entities[-1],
            attacment_orientation=comp.AttachmentOrientation(inclination=np.deg2rad(90), azimuth=np.deg2rad(0)),
            vein_length_multiplier=curve,
            midrib_length=0.6 * self.leaf_size * 0.9,
            lateral_vein_length=0.32 * self.leaf_size * 0.9,
        ))

    def as_component_batches(self) -> Iterable[rr.ComponentBatchLike]:
        return [
            AnyBatchValue("comps.StrawberryStem.petiole_length", self.petiole_length),
            AnyBatchValue("comps.StrawberryStem.petiole_entity_count", self.petiole_entity_count),  
        ]
