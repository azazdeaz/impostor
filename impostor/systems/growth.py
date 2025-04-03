from dataclasses import dataclass

import numpy as np
import rerun as rr
from scipy.spatial.transform._rotation import Rotation

import impostor.components as comp
from impostor.components import (
    AxePrev,
    RigidTransformation,
    Vascular,
)
from impostor.plant import Entity, Plant


def start_root(plant: Plant):
    q = plant.query()
    root = q.without_component(AxePrev).single()
    if root is None:
        root = plant.create_entity(
            comp.Root(),
            comp.Vascular(
                rotation=Rotation.from_euler("xyz", [0, 3, 0], degrees=True),
                radius=0.01,
            ),
        )
        meristem = plant.create_entity(comp.GrowthTip(), comp.AxePrev(root))
        plant.add_components(
            root,
            comp.AxeNext(meristem),
            comp.RigidTransformation.from_rotation(
                Rotation.from_euler("xyz", [0, 0, 0], degrees=True)
            ),
        )
        plant.create_entity(
            comp.Spring(
                root,
                meristem,
                # angle=Rotation.from_euler("xyz", [0, 30, 0], degrees=True),
            )
        )
    return root


def grow_system(plant: Plant):
    max_stem_length = 0.1
    growth_rate = 0.04

    for tip in plant.query().with_component(comp.GrowthTip).entities():
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
                tip_comp = plant.get_components(tip).get_by_type(comp.GrowthTip)
                plant.remove_components(tip, tip_comp)
                new_tip = plant.create_entity(tip_comp, comp.AxePrev(tip))
                plant.add_components(
                    tip, comp.AxeNext(new_tip), comp.Vascular(length=0.01, radius=0.003)
                )
                plant.create_entity(
                    comp.Spring(
                        tip,
                        new_tip,
                        angle=Rotation.from_euler("xyz", [0, 3, 0], degrees=True),
                        angle_stiffness=0.1,
                    )
                )


@dataclass
class RelaxSpringSystem:
    max_iterations: int = 10
    max_distance_step: float = 10.1
    distance_relaxed_treshold: float = 0.0001

    def execute(self, plant: Plant):
        spring_entities = plant.query().with_component(comp.Spring)._entities
        if len(spring_entities) == 0:
            return

        # Entity to all its connected spring entities
        connection_map = {}

        # Build the connection map
        for spring_entity, spring_comps in spring_entities.items():
            spring = spring_comps.get_by_type(comp.Spring)
            if spring.entity_a not in connection_map:
                connection_map[spring.entity_a] = set()
            connection_map[spring.entity_a].add(spring_entity)
            if spring.entity_b not in connection_map:
                connection_map[spring.entity_b] = set()
            connection_map[spring.entity_b].add(spring_entity)

        self.update_spring_targets(plant)

        root_entity = plant.query().with_component(comp.Root).single()

        if root_entity not in connection_map:
            raise ValueError("Root entity is not connected to any spring entity")

        relaxeds = set()

        lines = []

        sideways = comp.AttachmentOrientation(inclination=np.pi / 2).as_rotation()
        sideways = RigidTransformation.from_rotation(sideways)


        def relax_all_from(spring_entity: Entity):
            relaxeds.add(spring_entity)
            spring = plant.get_components(spring_entity).get_by_type(comp.Spring)
            self.relax_spring(plant, spring_entity)

            ta = plant.get_components(spring.entity_a).get_by_type(RigidTransformation)
            tb = plant.get_components(spring.entity_b).get_by_type(RigidTransformation)
            lines.append(
                [
                    ta.combine(sideways).transform_point(np.asarray([0.01, 0.01, 0])),
                    ta.translation,
                    tb.translation,
                    tb.combine(sideways).transform_point(np.asarray([0.01, 0.01, 0])),
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

    def update_spring_targets(self, plant: Plant):
        spring_entities = plant.query().with_component(comp.Spring)._entities

        # Update the target length and angle of the springs
        for spring_entity, spring_comps in spring_entities.items():
            spring = spring_comps.get_by_type(comp.Spring)
            comps_a = plant.get_components(spring.entity_a)
            comps_b = plant.get_components(spring.entity_b)

            if (
                comp.AxePrev in comps_b
                and comps_b.get_by_type(comp.AxePrev).prev == spring.entity_a
            ):
                if comp.Vascular in comps_a:
                    stem_a = comps_a.get_by_type(comp.Vascular)
                    spring.length = stem_a.length
                    spring.angle_rest = stem_a.rotation
                    spring.angle = spring.angle_rest

            if comp.AttachmentOrientation in comps_b:
                branch = comps_b.get_by_type(comp.AttachmentOrientation)
                spring.length = 0
                spring.angle_rest = branch.as_rotation()
                spring.angle = spring.angle_rest

            if not spring.fixed_angle_stiffness and comp.Mass in comps_a:
                mass_a = comps_a.get_by_type(comp.Mass)
                # For now, the mass determines how bendable the spring is
                spring.angle_stiffness = (mass_a.mass / 1.1) ** 2

            plant.add_components(spring_entity, spring)

    def relax_spring(self, plant: Plant, spring_entity):
        spring = plant.get_components(spring_entity).get_by_type(comp.Spring)
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
                rotation_axis = np.asarray([-pointing[1], pointing[0], 0])
                bend = 5.0 * (1 - np.clip(spring.angle_stiffness, 0.0, 1))
                rotation = Rotation.from_rotvec(rotation_axis * np.deg2rad(bend))
                spring.angle = rotation * spring.angle_rest

        transform_a_b = RigidTransformation.from_rotation(spring.angle).combine(
            RigidTransformation.from_z_translation(spring.length)
        )

        comps_b.add(transform_a.combine(transform_a_b))
