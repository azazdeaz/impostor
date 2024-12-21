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

    for _ in range(iterations):
        syst.grow_system(plant)
        relax_spring_system.execute(plant)
        branch_system.execute(plant)    

    return plant, root_entity


if __name__ == "__main__":
    # create a recording id so the Rust process can log to the same Rerun recording
    recording_id = str(int(time.time()))
    rr.init("impostor", spawn=True, recording_id=recording_id)
    for i in range(1):
        rr.set_time_sequence("frame_idx", i)
        plant, root = test_grow(i + 20)
        syst.log_transforms_system(plant)
        mesh = syst.create_plant_mesh(plant)
        mesh.rr_log()
        syst.rr_log_components(plant)
    