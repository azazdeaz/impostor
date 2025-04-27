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
            initialize_petiole(
                plant,
                self.base_entity,
                self.current_petiole_length,
                self.current_leaf_size,
                self.current_inclanation,
                self.current_angle,
            )

        self.age += 1
        print(f"Crown is {self.age} steps old")


def initialize_petiole(
    plant: Plant,
    base_entity: comp.Entity,
    petiole_length: float,
    leaf_size: float,
    inclanation: float,
    angle: float,
) -> None:
    rotation = comp.Rotation.from_euler("xyz", [inclanation, 0, angle])
    print(f"Start petiole in {rotation.as_euler('xyz')}")

    first_entity = plant.create_entity(
        parts.Vascular(rotation=rotation),
    )
    plant.get_components(base_entity).get_by_type(comp.Attachments).attachments.append(
        first_entity
    )
    strawberry_stem = plant.create_entity(
        parts.StrawberryStem(
            petiole_length=petiole_length,
            leaf_size=leaf_size,
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
