from dataclasses import dataclass
from typing import  List, Tuple

import numpy as np

from impostor.components import Entity
from .core import BasePart
from impostor.plant import Plant
import impostor.parts as parts


@dataclass
class Collider(BasePart):
    compute_from_vascular: bool = True
    radius: float = 0.0001
    
    def step(self, plant: Plant, entity: Entity):
        if self.compute_from_vascular:
            vascular = plant.get_components(entity).get_by_type(parts.Vascular)
            self.radius = vascular.radius * 1.2


@dataclass
class CollisionSolver(BasePart):
    max_move: float = 0.01

    def step(self, plant: Plant, entity: Entity):
        colliders: List[Tuple[Entity, parts.Collider, parts.RigidTransformation]] = [
            (entity, comps.get_by_type(parts.Collider), comps.get_by_type(parts.RigidTransformation))
            for entity, comps in plant.query().with_components(parts.Collider, parts.RigidTransformation).items()
        ]
        
        for a_entity, a_coll, a_trans in colliders:
            for b_entity, b_coll, b_trans in colliders:
                if a_entity == b_entity:
                    continue
                travel = a_trans.translation - b_trans.translation
                distance = np.linalg.norm(travel)

                # Skip checking collisions for attached entities
                if distance == 0:
                    continue
                
                min_distance = a_coll.radius + b_coll.radius
                if distance < min_distance:
                    # Move the entities apart
                    move = (min_distance - distance) / 2
                    move_dir = travel / distance
                    move = move_dir * move
                    a_trans.translation += move
                    b_trans.translation -= move
                