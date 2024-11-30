from impostor.systems.mesh import create_axes_mesh_data
import impostor_core

from impostor.plant import Plant
from impostor.systems import add_transforms_system, grow_system, start_root, branch_system
import impostor.messages
from impostor.utils import NormalDistribution

import rerun as rr 
import time


def test_grow(iterations=100):
    plant = Plant()
    root_entity = start_root(plant)
    for _ in range(iterations):
        grow_system(plant, root_entity)
        branch_system(plant, root_entity, NormalDistribution(1.6, 0.1))    

    return plant, root_entity


if __name__ == "__main__":
    # create a recording id so the Rust process can log to the same Rerun recording
    recording_id = str(int(time.time()))
    rr.init("impostor", spawn=True, recording_id=recording_id)
    
    plant, root = test_grow()
    add_transforms_system(plant, root)
    axes = create_axes_mesh_data(plant, root)
    stems = [impostor.messages.Stem(rings=rings) for rings in axes]
    plant = impostor.messages.Plant(stems=stems)

    plant_msg = plant.bincode_serialize()
    print(impostor_core.test_mesh(plant_msg, recording_id))