from impostor.plant import Plant

class BasePart:
    def step(plant: Plant):
        pass

def step_parts(plant: Plant):
    for comps in list(plant.entities.values()):
        for comp in list(comps):
            # Check if the component has a step method
            if hasattr(comp, "step"):
                comp.step(plant)
