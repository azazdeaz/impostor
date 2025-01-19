from dataclasses import dataclass
from impostor.parts import BasePart
from impostor.plant import Plant
import impostor.components as comp
import numpy as np

@dataclass
class Crown(BasePart):
    age = 0
    current_angle = 0
    current_inclanation = np.deg2rad(35)
    angle_step = np.deg2rad(75)
    step_per_sprout = 12
    stop_sprouting = 60

    base_entity = None

    def step(self, plant: Plant):
        if self.base_entity is None:
            self.base_entity = plant.create_entity(comp.Root(), comp.RigidTransformation())

        print(f"{self.age} % {self.step_per_sprout}")
        if self.age <= self.stop_sprouting and self.age % self.step_per_sprout == 0:
            print("start petiole")
            self.start_petiole(plant)

        self.age += 1
        print(f"Crown is {self.age} steps old")

    def start_petiole(self, plant: Plant):
        rotation = comp.Rotation.from_euler("xyz", [self.current_inclanation, 0, self.current_angle])
        print(f"Start petiole in {rotation.as_euler('xyz')}")

        self.current_angle += self.angle_step
        self.current_inclanation *= 0.7
        
        first_entity = plant.create_entity(
            comp.Vascular(
                rotation=rotation
            ),
        )
        plant.create_entity(comp.Spring(self.base_entity, first_entity))
        meristem = plant.create_entity(comp.GrowthTip(), comp.AxePrev(first_entity))
        plant.add_components(
            first_entity,
            comp.AxeNext(meristem),
            comp.RigidTransformation.from_rotation(
                comp.Rotation.from_euler("xyz", [0, 0, 0], degrees=True)
            ),
        )
        plant.create_entity(
            comp.Spring(
                first_entity,
                meristem,
            )
        )