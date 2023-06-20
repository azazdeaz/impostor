from typing import List
import esper
import numpy as np
from scipy.spatial.transform import Rotation # type: ignore

from dataclasses import dataclass, field



@dataclass
class RootAxis:
    """Marks the base of the plant. This is the lowest level part. It is connected to the ground."""
    pass

@dataclass
class ApicalPart:
    """The next part in the stem. This is the part closer to the tip."""
    entity: int

@dataclass
class BasalPart:
    """The previous part in the stem. This is the part closer to the root."""
    entity: int

@dataclass
class Length:
    """Length of a stem section."""
    length: float = 0.0

@dataclass
class Radius:
    """The radius of a stem section at its base."""
    radius: float = 0.0  # Small correction from your original draft

@dataclass
class Rotation:
    """Rotation of a stem section at its base as a quaternion."""
    quaternion = np.array([0, 0, 0, 1])

@dataclass
class Node:
    """Branches starting from this part"""
    branch_parts: List[int] = field(default_factory=list)



@dataclass
class Plant:
    def __init__(self):
        self.world = esper.World()
        self.base = self.world.create_entity(Length())

    def iterate_up(self, start_part, start_level=0, visit_branches=True):
        """Iterates through the parts in the apical direction (towards the tip) starting from the provided part."""
        entity = start_part
        level = start_level
        while True:
            yield entity, level

            next = self.world.try_component(entity, ApicalPart)
            node = self.world.try_component(entity, Node)

            if node:
                for branch_part in node.branch_parts:
                    if visit_branches:
                        yield from self.iterate_up(branch_part, level+1)

            if next:
                entity = next.entity
            else:
                break

    def grow(self, branch_level, max_length = 0.2, increment = 0.067):
        for entity, level in self.iterate_up(self.base):
            if level == branch_level:
                length = self.world.component_for_entity(entity, Length)
                if length.length < max_length - increment:
                    length.length += increment
                else:
                    remainder = length.length + increment - max_length
                    length.length = max_length
                    if not self.world.has_component(entity, ApicalPart):
                        next = self.world.create_entity()
                        self.world.add_component(entity, ApicalPart(next))
                        self.world.add_component(next, BasalPart(entity))
                        self.world.add_component(next, Length(remainder))

    def print_state(self):
        for entity, level in self.iterate_up(self.base):
            length = self.world.component_for_entity(entity, Length)
            print(f"Entity {entity} at level {level} with length {length.length}")

# Grow a test plant
plant = Plant()
for i in range(30):
    print(f"\nGrowing iteration {i}")
    plant.grow(0)
    plant.print_state()



                    
            


