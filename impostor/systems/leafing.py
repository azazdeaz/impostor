import random
from dataclasses import dataclass

import numpy as np

import impostor.components as comp
from impostor.plant import Entity, Plant
from impostor.utils import NormalDistribution



@dataclass
class LeafingSystem:
    def execute(self, plant: Plant):
        apices = plant.query().with_component(comp.GrowthTip)._entities
        for apex in apices:
            plant.create_entity(comp.LeafMeta(plant, apex))
