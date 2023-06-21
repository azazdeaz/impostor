from typing import List
import esper
import numpy as np
import quaternionic
import rerun as rr


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
    quaternion: quaternionic.array = field(default_factory=lambda: quaternionic.array([1, 0, 0, 0]))

@dataclass
class Node:
    """Branches starting from this part"""
    branch_parts: List[int] = field(default_factory=list)
    leaf_parts: List[int] = field(default_factory=list)

@dataclass
class Leaf:
    """Represents a simple leaf."""
    size: float = 0.0


@dataclass
class PartIteration:
    part_id: int
    parent_part_id: int | None
    level: int
    is_tip: bool = False

@dataclass
class GlobalTransform:
    """The global transform of a part."""
    position: np.ndarray = field(default_factory=lambda: np.array([0, 0, 0]))
    rotation: np.ndarray = field(default_factory=lambda: quaternionic.array([1, 0, 0, 0]))

@dataclass
class PhyllotacticAxis:
    """The axis from which leaves or branches start at this node, used to calculate rotation."""
    first_leaf_rotation: float = 0.0


@dataclass
class Plant:
    def __init__(self):
        self.world = esper.World()
        self.base = self.world.create_entity(Length())
        # rr.init("plant", spawn=True)

    def upsert_component(self, entity, component):
        """Adds or updates a component for an entity."""
        if self.world.has_component(entity, type(component)):
            self.world.remove_component(entity, type(component))
        self.world.add_component(entity, component)

    def iterate_up(self, start_part, parent_part_id=None, start_level=0, visit_branches=True):
        """Iterates through the parts in the apical direction (towards the tip) starting from the provided part."""
        entity = start_part
        level = start_level
        while True:

            next = self.world.try_component(entity, ApicalPart)
            node = self.world.try_component(entity, Node)

            yield PartIteration(entity, parent_part_id, level, is_tip=next is None)

            if node:
                for branch_part in node.branch_parts:
                    if visit_branches:
                        yield from self.iterate_up(branch_part, entity, level+1)

            if next:
                entity = next.entity
            else:
                break

    def iterate_down(self, start_part):
        """Iterates through the parts in the basal direction (towards the root) starting from the provided part."""
        entity = start_part
        while True:
            yield entity
            next = self.world.try_component(entity, BasalPart)
            if next:
                entity = next.entity
            else:
                break

    def grow(self, branch_level: int, max_length = 0.2, increment = 0.067):
        for iteration in self.iterate_up(self.base):
            if iteration.level == branch_level:
                length = self.world.component_for_entity(iteration.part_id, Length)
                if length.length < max_length - increment:
                    length.length += increment
                else:
                    remainder = length.length + increment - max_length
                    length.length = max_length
                    if not self.world.has_component(iteration.part_id, ApicalPart):
                        next = self.world.create_entity()
                        self.world.add_component(iteration.part_id, ApicalPart(next))
                        self.world.add_component(next, BasalPart(iteration.part_id))
                        self.world.add_component(next, Length(remainder))

    def branch_out(self, branch_level: int, node_distance: float):
        # collect all the tips at the branch level
        tips = []
        for iteration in self.iterate_up(self.base):
            if iteration.level == branch_level and iteration.is_tip:
                tips.append(iteration.part_id)
        # if last-node <> tip distance is less than node_distance, branch out
        for tip in tips:
            distance = 0.0
            node_part = None
            for part in self.iterate_down(tip):
                distance += self.world.component_for_entity(part, Length).length
                is_node = self.world.has_component(part, Node)
                if is_node:
                    node_part = part
                    break
            print(f"distance: {distance} node_distance: {node_distance} node_part: {node_part}")
            if distance < node_distance:
                prev_axis = self.world.component_for_entity(node_part, PhyllotacticAxis) if node_part else PhyllotacticAxis()
                new_axis = PhyllotacticAxis(prev_axis.first_leaf_rotation + np.pi/2)

                def add_branch(y_rotation):
                    self.world.add_component(tip, new_axis)
                    self.world.add_component(tip, Node())
                    rotation = quaternionic.array.from_euler_angles(0, y_rotation, 0)
                    self.world.create_entity(Length(0.1), BasalPart(tip), Rotation(rotation))
                
                add_branch(new_axis.first_leaf_rotation)
                add_branch(new_axis.first_leaf_rotation + np.pi)

                



    def log_rerun(self):
        for iteration in self.iterate_up(self.base):
            pass


    def print_state(self):
        for iteration in self.iterate_up(self.base):
            length = self.world.component_for_entity(iteration.part_id, Length)
            print(f"Entity {iteration.part_id} at level {iteration.level} with length {length.length}")

# Grow a test plant
plant = Plant()
for i in range(30):
    print(f"\nGrowing iteration {i}")
    plant.grow(0)
    plant.branch_out(0, 0.52)
    plant.print_state()



                    
            


