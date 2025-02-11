from dataclasses import dataclass
from typing import Iterable
from impostor.parts.core import BasePart
from impostor.plant import Plant
import impostor.components as comp
import impostor.parts as parts
from impostor.plant import Entity
import rerun as rr
import numpy as np
from rerun.any_value import AnyBatchValue

@dataclass
class Mass(rr.AsComponents, BasePart):
    mass: float = 1.0
    
    base_radius: float = 0.06
    reduction_factor: float = 0.02

    def step(self, plant: Plant, entity: Entity):
        comps = plant.get_components(entity)
        if parts.Vascular in comps:
            vascular = comps.get_by_type(parts.Vascular)
            mass = comps.get_or_create_by_type(parts.Mass)
            mass.mass = np.pi * vascular.radius**2 * vascular.length * 1000

    def as_component_batches(self) -> Iterable[rr.ComponentBatchLike]:
        return [AnyBatchValue("comps.Mass.mass", self.mass)]



@dataclass
class MassAbove(rr.AsComponents, BasePart):
    mass: float = 0.0
    
    def step(self, plant: Plant, root_entity: Entity):
        """
        Iteratively computes the MassAbove values
        """
        # only run for the root entity
        if comp.Root not in plant.get_components(root_entity):
            return

        dependency_map: dict[Entity, list[Entity]] = {}
        full_mass_map: dict[Entity, float] = {}
        self_mass_map: dict[Entity, float] = {}

        # Build dependency map
        queue = [root_entity]
        while len(queue) > 0:
            entity = queue.pop(0)
            comps = plant.get_components(entity)
            dependency_map[entity] = []

            if comp.AxeNext in comps:
                dependency_map[entity] = [comps.get_by_type(comp.AxeNext).next]
            if comp.Attachments in comps:
                dependency_map[entity] += comps.get_by_type(comp.Attachments).attachments
            if parts.Mass in comps:
                self_mass_map[entity] = comps.get_by_type(parts.Mass).mass
            
            # add unregistered dependencies to the queue
            for dependency in dependency_map[entity]:
                if dependency not in dependency_map:
                    queue.append(dependency)

        # This shouldn't need to be higher than the max branch order
        recusion_limit = 100

        # Compute mass above map
        while len(dependency_map) > len(full_mass_map):
            recusion_limit -= 1
            if recusion_limit < 0:
                raise RecursionError(
                    "Recursion limit reached while solving mass dependencies"
                )

            # We added the nodes from the root up, so solving the dependencies in reverse will be more efficient
            for root_entity, dependencies in reversed(dependency_map.items()):
                if all(dep in full_mass_map for dep in dependencies):
                    full_mass_map[root_entity] = sum(
                        full_mass_map[dep] for dep in dependencies
                    ) + self_mass_map.get(root_entity, 0)

        # Update mass above components
        for root_entity, full_mass in full_mass_map.items():
            mass_above = full_mass - self_mass_map.get(root_entity, 0)
            comps = plant.get_components(root_entity)
            comps.add(parts.MassAbove(mass=mass_above))
    

    def as_component_batches(self) -> Iterable[rr.ComponentBatchLike]:
        return [AnyBatchValue("comps.MassAbove.mass", self.mass)]