from xml.dom.minidom import Entity
from impostor.plant import Plant

class BasePart:
    def step(self, plant: Plant, entity: Entity):
        pass

def step_parts(plant: Plant):
    for entity, comps in list(plant.entities.items()):
        for comp in list(comps):
            # Check if the component has a step method
            if hasattr(comp, "step"):
                comp.step(plant, entity)
