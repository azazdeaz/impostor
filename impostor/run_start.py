from impostor.plant import Plant
import impostor.systems as syst
from impostor.utils import NormalDistribution

import rerun as rr 
import time


def test_grow(iterations=120):
    plant = Plant()
    root_entity = syst.start_root(plant)
    branch_system = syst.BranchingSystem(internode_spacing=NormalDistribution(1.6, 0.1))
    relax_spring_system = syst.RelaxSpringSystem()
    secondary_growth_system = syst.SecondaryGrowthSystem()
    update_mass_above_system = syst.UpdateMassAboveSystem()

    for i in range(iterations):
        rr.set_time_sequence("frame_idx", i)
        syst.grow_system(plant)
        branch_system.execute(plant)    
        secondary_growth_system.execute(plant)

        if i == iterations - 1:
            syst.LeafingSystem().execute(plant)

        update_mass_above_system.execute(plant)
        relax_spring_system.execute(plant)
        syst.rr_log_components(plant)
        syst.rr_log_graph(plant)
        syst.rr_log_transforms_system(plant)
        mesh = syst.create_plant_mesh(plant)
        mesh.rr_log()

    return plant, root_entity


if __name__ == "__main__":
    # create a recording id so the Rust process can log to the same Rerun recording
    recording_id = str(int(time.time()))
    rr.init("impostor", spawn=True, recording_id=recording_id)
        
    plant, root = test_grow(80)
    