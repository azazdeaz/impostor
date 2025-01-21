from dataclasses import dataclass, field
from typing import Iterable

import numpy as np
import rerun as rr
from rerun.any_value import AnyBatchValue
from scipy.spatial.transform._rotation import Rotation

from impostor.components import Entity, RigidTransformation
from impostor.parts.core import BasePart
from impostor.plant import Plant
import impostor.components as comp
import impostor.parts as parts


@dataclass
class Spring(rr.AsComponents, BasePart):
    entity_a: Entity
    entity_b: Entity
    weight_a: float = 1.0
    weight_b: float = 0.0
    length: float = 0.0
    angle: Rotation = field(default_factory=Rotation.identity)
    angle_rest: Rotation = field(default_factory=Rotation.identity)
    angle_stiffness: float = 0.0 # 0.0 is no stiffness, 1.0 is full stiffness
    fixed_angle_stiffness: bool = False # If false, angle_stiffness is computed from mass and radius

    def step(self, plant: Plant, entity: Entity):
        comps_a = plant.get_components(self.entity_a)
        comps_b = plant.get_components(self.entity_b)

        # If it's connecting consecutive parts of a stem
        if (
            comp.AxePrev in comps_b
            and comps_b.get_by_type(comp.AxePrev).prev == self.entity_a
            and parts.Vascular in comps_a
        ):
            stem_a = comps_a.get_by_type(parts.Vascular)
            self.length = stem_a.length
            self.angle_rest = stem_a.rotation
            self.angle = self.angle_rest
        else:
            print(f"Spring {entity} {self.entity_a} -> {self.entity_b} is not connecting consecutive parts of a stem")

        # If it's connecting a branch to a stem
        if comp.AttachmentOrientation in comps_b:
            branch = comps_b.get_by_type(comp.AttachmentOrientation)
            self.length = 0
            self.angle_rest = branch.as_rotation()
            self.angle = self.angle_rest

        # If the angle stiffness is not fixed, compute it from the mass
        if not self.fixed_angle_stiffness and comp.Mass in comps_a:
            mass_a = comps_a.get_by_type(comp.Mass)
            # For now, the mass determines how bendable the spring is
            self.angle_stiffness = (mass_a.mass / 1.1) ** 2
            

    def to_transform(self) -> RigidTransformation:
        return RigidTransformation.from_rotation(self.angle).combine(RigidTransformation.from_z_translation(self.length))
    
    def as_component_batches(self) -> Iterable[rr.ComponentBatchLike]:
        return [
            AnyBatchValue("comps.Spring.entity_a", self.entity_a),
            AnyBatchValue("comps.Spring.entity_b", self.entity_b),
            AnyBatchValue("comps.Spring.length", self.length),
            AnyBatchValue("comps.Spring.angle", rr.components.Vector3D(self.angle.as_euler("xyz"))),
            AnyBatchValue("comps.Spring.angle_rest", rr.components.Vector3D(self.angle_rest.as_euler("xyz"))),
            AnyBatchValue("comps.Spring.angle_stiffness", self.angle_stiffness),
        ]
    
@dataclass
class SpringGraphSolver(BasePart):
    max_iterations: int = 10
    max_distance_step: float = 10.1
    distance_relaxed_treshold: float = 0.0001

    def step(self, plant: Plant, entity: Entity):
        spring_entities = plant.query().with_component(Spring)._entities
        if len(spring_entities) == 0:
            return

        # Entity to all its connected spring entities
        connection_map = {}

        # Build the connection map
        for spring_entity, spring_comps in spring_entities.items():
            spring = spring_comps.get_by_type(Spring)
            if spring.entity_a not in connection_map:
                connection_map[spring.entity_a] = set()
            connection_map[spring.entity_a].add(spring_entity)
            if spring.entity_b not in connection_map:
                connection_map[spring.entity_b] = set()
            connection_map[spring.entity_b].add(spring_entity)

        root_entity = plant.query().with_component(comp.Root).single()

        if root_entity not in connection_map:
            raise ValueError("Root entity is not connected to any spring entity")

        relaxeds = set()

        lines = []

        sideways = comp.AttachmentOrientation(inclination=np.pi / 2).as_rotation()
        sideways = RigidTransformation.from_rotation(sideways)

        def relax_all_from(spring_entity: Entity):
            relaxeds.add(spring_entity)
            spring = plant.get_components(spring_entity).get_by_type(Spring)
            self.relax_spring(plant, spring_entity)

            ta = plant.get_components(spring.entity_a).get_by_type(RigidTransformation)
            tb = plant.get_components(spring.entity_b).get_by_type(RigidTransformation)
            lines.append(
                [
                    ta.combine(sideways).transform_point(np.array([0.01, 0.01, 0])),
                    ta.translation,
                    tb.translation,
                    tb.combine(sideways).transform_point(np.array([0.01, 0.01, 0])),
                ]
            )

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

        rr.log(
            "springs",
            rr.LineStrips3D(
                lines,
                radii=0.0003,
            ),
        )

    def relax_spring(self, plant: Plant, spring_entity):
        spring = plant.get_components(spring_entity).get_by_type(Spring)
        comps_a = plant.get_components(spring.entity_a)
        comps_b = plant.get_components(spring.entity_b)

        transform_a = comps_a.get_by_type(RigidTransformation)

        # Apply gravity
        if spring.length > 0 and comp.MassAbove in comps_a:
            # Get the direction where this node is pointing
            pointing = (transform_a.rotation * spring.angle_rest).apply([0, 0, 1])
            # Make it flat on the horizontal plane
            pointing[2] = 0
            # Check if the magnitude of the pointing direction is not zero
            magnitude = np.linalg.norm(pointing)
            if magnitude != 0 and spring.angle_stiffness > 0:
                pointing = pointing / magnitude
                # The rotation axis is perpendicular to the pointing direction on the horizontal plane
                rotation_axis = np.array([-pointing[1], pointing[0], 0])
                bend = 5.0 * (1 - np.clip(spring.angle_stiffness, 0.0, 1))
                rotation = Rotation.from_rotvec(rotation_axis * np.deg2rad(bend))
                spring.angle = rotation * spring.angle_rest

        transform_a_b = RigidTransformation.from_rotation(spring.angle).combine(
            RigidTransformation.from_z_translation(spring.length)
        )

        comps_b.add(transform_a.combine(transform_a_b))
    
