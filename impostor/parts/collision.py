from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from impostor.components import Entity
from .core import BasePart
from impostor.plant import Plant
import impostor.parts as parts

@dataclass
class Force:
    """Represents forces and torques applied during collision.

    Attributes:
        linear (np.ndarray): Linear force vector (N) in world coordinates.
        angular (np.ndarray): Angular torque as a 3D vector where:
            - Direction represents the axis of rotation (right-hand rule)
            - Magnitude represents the torque strength (N⋅m)
            Example: [1, 0, 0] represents a torque around the X axis
    """
    linear: np.ndarray
    angular: np.ndarray  # Axis-angle representation

@dataclass
class Collider(BasePart):
    compute_from_vascular: bool = True
    radius: float = 0.0001

    def step(self, plant: Plant, entity: Entity):
        if self.compute_from_vascular:
            vascular = plant.get_components(entity).get_by_type(parts.Vascular)
            self.radius = vascular.radius * 1.2
    
    @staticmethod
    def get_collision_forces(plant: Plant, a: Entity, b: Entity) -> Optional[Tuple[Force, Force]]:
        plant_a = plant.get_components(a)
        plant_b = plant.get_components(b)

        transform_a = plant_a.get_by_type(parts.RigidTransformation)
        transform_b = plant_b.get_by_type(parts.RigidTransformation)
        collider_a = plant_a.get_by_type(Collider)
        collider_b = plant_b.get_by_type(Collider)
        mass_a = plant_a.get_by_type(parts.Mass)
        mass_b = plant_b.get_by_type(parts.Mass)

        if transform_a is None or transform_b is None:
            return None
        
        travel = transform_a.translation - transform_b.translation
        distance = np.linalg.norm(travel)
        min_distance = collider_a.radius + collider_b.radius

        if distance == 0:
            return None
        
        if distance < min_distance:
            m_a = mass_a.mass if mass_a is not None else 0.00001
            m_b = mass_b.mass if mass_b is not None else 0.00001
            mass = m_a + m_b
            if mass > 0:
                a_weight = m_b / mass
                b_weight = m_a / mass
            else:
                a_weight = 0.5
                b_weight = 0.5

            move = min_distance - distance
            move_dir = travel / distance
            move = move_dir * move
            force_a = Force(linear=move * a_weight, angular=np.zeros(3))
            force_b = Force(linear=-move * b_weight, angular=np.zeros(3))
            return force_a, force_b



@dataclass
class CollisionSolver(BasePart):
    max_move: float = 0.002
    iterations: int = 10

    def step(self, plant: Plant, entity: Entity):
        colliders: List[
            Tuple[
                Entity, parts.Collider, parts.RigidTransformation, Optional[parts.Mass]
            ]
        ] = [
            (
                entity,
                comps.get_by_type(parts.Collider),
                comps.get_by_type(parts.RigidTransformation),
                comps.get_by_type(parts.Mass),
            )
            for entity, comps in plant.query()
            .with_components(parts.Collider, parts.RigidTransformation)
            .items()
        ]

        for _ in range(self.iterations):
            for a_entity, a_coll, a_trans, a_mass in colliders:
                for b_entity, b_coll, b_trans, b_mass in colliders:
                    a_m = a_mass.mass if a_mass is not None else 0.00001
                    b_m = b_mass.mass if b_mass is not None else 0.00001
                    mass = a_m + b_m
                    if mass > 0:
                        a_weight = b_m / mass
                        b_weight = a_m / mass
                    else:
                        a_weight = 0.5
                        b_weight = 0.5

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
                        move = min_distance - distance
                        move_dir = travel / distance
                        move = move_dir * move
                        a_trans.translation += move * a_weight
                        b_trans.translation -= move * b_weight
