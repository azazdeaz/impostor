from impostor.systems.mesh import create_axes_mesh_data
import impostor_core

from impostor.plant import Plant
from impostor.systems import add_transforms_system, grow_system, start_root, branch_system
import impostor.messages
from impostor.utils import NormalDistribution


def test_grow(iterations=27):
    plant = Plant()
    root_entity = start_root(plant)
    for _ in range(iterations):
        grow_system(plant, root_entity)
        branch_system(plant, root_entity, NormalDistribution(1.6, 0.1))    
    for entity, components in plant.entities.items():
        print(entity, components.print())

    return plant, root_entity


if __name__ == "__main__":
    plant, root = test_grow()
    add_transforms_system(plant, root)
    axes = create_axes_mesh_data(plant, root)
    stems = [impostor.messages.Stem(rings=rings) for rings in axes]
    plant = impostor.messages.Plant(stems=stems)

    print("\n\nAXES")
    for index, axis in enumerate(axes):
        print(f"Index: {index}, Axis: {axis}")

    plant_msg = plant.bincode_serialize()
    print(impostor_core.test_mesh(plant_msg))