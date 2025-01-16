from impostor.plant import Plant
import impostor.systems as syst
from impostor.utils import NormalDistribution

import rerun as rr
import time

class Profile:
    def __init__(self, label: str):
        self.label = label
        self.start_time = time.time()
    
    def end(self):
        print(f"{self.label}: {time.time() - self.start_time}")

def test_grow(iterations=120):
    plant = Plant()
    root_entity = syst.start_root(plant)
    branch_system = syst.BranchingSystem(
        internode_spacing=NormalDistribution(0.6, 0.1), max_branch_order=1
    )
    relax_spring_system = syst.RelaxSpringSystem()
    secondary_growth_system = syst.SecondaryGrowthSystem()
    update_mass_above_system = syst.UpdateMassAboveSystem()
    start_leaf_system = syst.StartLeafSystem(
        at_stem_length=NormalDistribution(0.5, 0.1), branch_order=1
    )
    grow_leaf_system = syst.GrowLeafSystem()

    for i in range(iterations):
        print(f"Frame {i}")
        rr.set_time_sequence("frame_idx", i)
        syst.grow_system(plant)
        branch_system.execute(plant)
        secondary_growth_system.execute(plant)
        start_leaf_system.execute(plant)
        grow_leaf_system.execute(plant)
        update_mass_above_system.execute(plant)
        relax_spring_system.execute(plant)
        syst.rr_log_components(plant)
        # syst.rr_log_graph(plant)
        syst.rr_log_transforms_system(plant)
        mesh = syst.create_plant_mesh(plant)
        mesh.rr_log()

    return plant, root_entity


if __name__ == "__main__":
    # create a recording id so the Rust process can log to the same Rerun recording
    recording_id = str(int(time.time()))
    rr.init("impostor", spawn=True, recording_id=recording_id)

    plant, root = test_grow(40)
