from impostor.plant import Plant
import impostor.systems as syst
# from impostor.utils import NormalDistribution
from impostor import parts

import rerun as rr
import time


def test_grow(iterations=120):
    plant = Plant()
    plant.create_entity(parts.Crown())
    plant.create_entity(parts.SpringGraphSolver())

    # relax_spring_system = syst.RelaxSpringSystem()
    # secondary_growth_system = syst.SecondaryGrowthSystem(
    #     base_radius=0.012, reduction_factor=0.005
    # )
    # update_mass_above_system = syst.UpdateMassAboveSystem()
    # start_leaf_system = syst.StartLeafSystem(
    #     at_stem_length=NormalDistribution(0.6, 0.02), branch_order=0, is_trifoliate=True
    # )
    # grow_leaf_system = syst.GrowLeafSystem()

    for i in range(iterations):
        print(f"Frame {i}")
        parts.step_parts(plant)
        # rr.set_time_sequence("frame_idx", i)
        # syst.grow_system(plant)
        # secondary_growth_system.execute(plant)
        # start_leaf_system.execute(plant)
        # grow_leaf_system.execute(plant)
        # update_mass_above_system.execute(plant)
        # relax_spring_system.execute(plant)
        syst.rr_log_components(plant)
        # # syst.rr_log_graph(plant)
        syst.rr_log_transforms_system(plant)
        # mesh = syst.create_plant_mesh(plant)
        # mesh.rr_log()


if __name__ == "__main__":
    # create a recording id so the Rust process can log to the same Rerun recording
    recording_id = str(int(time.time()))
    rr.init("impostor", spawn=True, recording_id=recording_id)

    test_grow(40)
