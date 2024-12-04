from dataclasses import dataclass
import impostor.components as comp
from impostor.plant import Plant


@dataclass
class StemMeshSystem:
    def execute(self, plant: Plant):
        starts = plant.query().with_component(comp.Vascular).without_component(
            comp.AxePrev
        ).filter(
            lambda comps: comps.get_by_type(comp.Vascular).type
            == comp.VascularType.STEM
        )._entities
    

