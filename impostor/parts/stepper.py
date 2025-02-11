from xml.dom.minidom import Entity
from impostor.plant import Plant
from dataclasses import dataclass, field
from impostor.parts import BasePart

@dataclass
class PartStepperSystem():
    exec_order: list[type[BasePart]] = field(default_factory=list)

    def step_parts(self, plant: Plant):
        for part_type in self.exec_order:
            for entity, comps in list(plant.entities.items()):
                for comp in list(comps):
                    if isinstance(comp, part_type):
                        comp.step(plant, entity)
        
        # Step the parts that are not in the exec_order
        for entity, comps in list(plant.entities.items()):
            for comp in list(comps):
                if hasattr(comp, "step") and type(comp) not in self.exec_order:
                    comp.step(plant, entity)
