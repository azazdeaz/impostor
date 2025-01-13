from dataclasses import dataclass

import numpy as np

import impostor.components as comp
from impostor.plant import Entity, Plant


@dataclass
class SecondaryGrowthSystem:
    def execute(self, plant: Plant):
        def update_thickness(entity: Entity, branch_order=0):
            # Set the max radius based on the branch order
            max_radius = self.get_max_radius(branch_order)
            comps = plant.get_components(entity)
            if comp.Vascular in comps:
                vascular = comps.get_by_type(comp.Vascular)
                if vascular.type == comp.VascularType.STEM:
                    vascular.radius += (max_radius - vascular.radius) * 0.01

                mass = comps.get_or_create_by_type(comp.Mass)
                mass.mass = np.pi * vascular.radius**2 * vascular.length * 1000

            # Continue updating the thickness for the next nodes
            if comp.AxeNext in comps:
                update_thickness(comps.get_by_type(comp.AxeNext).next, branch_order)
            if comp.Attachments in comps:
                for branch in comps.get_by_type(comp.Attachments).attachments:
                    update_thickness(branch, branch_order + 1)

        for root in plant.query().with_component(comp.Root).entities():
            update_thickness(root)

    def get_max_radius(
        self,
        branch_order: int,
        base_radius: float = 0.06,
        reduction_factor: float = 0.02,
    ) -> float:
        """
        Calculate the maximum radius for a branch based on its order.

        Parameters:
        - branch_order (int): The order of the branch, where 0 is the main trunk and higher numbers are subsequent branches.
        - base_radius (float): The starting radius for the main trunk.
        - reduction_factor (float): The amount by which the radius is reduced for each branch order.

        Returns:
        - float: The maximum radius for the given branch order.
        """
        return max(base_radius - reduction_factor * branch_order, 0.01)
