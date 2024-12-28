from dataclasses import dataclass

import impostor.components as comp
from impostor.plant import Entity, Plant


@dataclass
class UpdateMassAboveSystem:
    def execute(self, plant: Plant):
        dependency_map: dict[Entity, list[Entity]] = {}
        full_mass_map: dict[Entity, float] = {}
        self_mass_map: dict[Entity, float] = {}

        # Build dependency map
        for entity in plant.entities.keys():
            comps = plant.get_components(entity)
            dependency_map[entity] = []

            if comp.AxeNext in comps:
                dependency_map[entity] = [comps.get_by_type(comp.AxeNext).next]
            if comp.Branches in comps:
                dependency_map[entity] += comps.get_by_type(comp.Branches).branches
            if comp.Mass in comps:
                self_mass_map[entity] = comps.get_by_type(comp.Mass).mass

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
            for entity, dependencies in reversed(dependency_map.items()):
                if all(dep in full_mass_map for dep in dependencies):
                    full_mass_map[entity] = sum(
                        full_mass_map[dep] for dep in dependencies
                    ) + self_mass_map.get(entity, 0)

        # Update mass above components
        for entity, full_mass in full_mass_map.items():
            mass_above = full_mass - self_mass_map.get(entity, 0)
            comps = plant.get_components(entity)
            comps.add(comp.MassAbove(mass=mass_above))
