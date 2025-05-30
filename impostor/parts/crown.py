from dataclasses import dataclass
from impostor.parts import BasePart
from impostor.plant import Plant
import impostor.components as comp
import impostor.parts as parts
import numpy as np


@dataclass
class Crown(BasePart):
    age = 0
    current_angle = 0
    current_inclanation = np.deg2rad(35)
    current_petiole_length = 0.4
    current_leaf_size = 0.8
    angle_step = np.deg2rad(75)
    step_per_sprout = 12
    stop_sprouting = 60

    base_entity = None

    def step(self, plant: Plant, entity):
        if self.base_entity is None:
            self.base_entity = plant.create_entity(
                comp.Root(),
                parts.RigidTransformation(),
                parts.MassAbove(),
                comp.Attachments(),
            )

        print(f"{self.age} % {self.step_per_sprout}")
        if self.age <= self.stop_sprouting and self.age % self.step_per_sprout == 0:
            print("start petiole")
            self.start_petiole(plant)

        self.age += 1
        print(f"Crown is {self.age} steps old")

    def start_petiole(self, plant: Plant):
        rotation = comp.Rotation.from_euler(
            "xyz", [self.current_inclanation, 0, self.current_angle]
        )
        print(f"Start petiole in {rotation.as_euler('xyz')}")

        self.current_angle += self.angle_step
        self.current_inclanation *= 0.5
        self.current_petiole_length += 0.06
        self.current_leaf_size *= 0.9

        first_entity = plant.create_entity(
            parts.Vascular(rotation=rotation),
        )
        plant.get_components(self.base_entity).get_by_type(
            comp.Attachments
        ).attachments.append(first_entity)
        strawberry_stem = plant.create_entity(
            parts.StrawberryStem(
                petiole_length=self.current_petiole_length,
                leaf_size=self.current_leaf_size,
            ),
            comp.AxePrev(first_entity),
        )
        plant.add_components(
            first_entity,
            comp.AxeNext(strawberry_stem),
            parts.RigidTransformation.from_rotation(
                comp.Rotation.from_euler("xyz", [0, 0, 0], degrees=True)
            ),
        )
