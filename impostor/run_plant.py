from impostor.plant import Plant
from impostor.systems import add_transforms_system, grow_system, start_root, branch_system
from impostor.utils import NormalDistribution


def test_grow(iterations=10):
    plant = Plant()
    root_entity = start_root(plant)
    for _ in range(iterations):
        grow_system(plant, root_entity)
    for entity, components in plant.entities.items():
        print(entity, components.print())

    return plant, root_entity


if __name__ == "__main__":
    plant, root = test_grow()
    add_transforms_system(plant, root)
