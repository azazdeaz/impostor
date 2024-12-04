import impostor_core

from impostor.plant import Plant
from impostor.systems import add_transforms_system, grow_system, start_root, BranchingSystem
import impostor.systems as syst
import impostor.messages
from impostor.utils import NormalDistribution

import rerun as rr 
import time


def test_grow(iterations=2):
    plant = Plant()
    root_entity = start_root(plant)
    branch_system = BranchingSystem(internode_spacing=NormalDistribution(1.6, 0.1))

    for _ in range(iterations):
        grow_system(plant, root_entity)
        branch_system.execute(plant)    

    return plant, root_entity


if __name__ == "__main__":
    # create a recording id so the Rust process can log to the same Rerun recording
    recording_id = str(int(time.time()))
    rr.init("impostor", spawn=True, recording_id=recording_id)
    plant, root = test_grow()
    add_transforms_system(plant, root)
    mesh = syst.create_plant_mesh(plant)
    mesh.rr_log()
    