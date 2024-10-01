from impostor.plant import Plant
from impostor.systems.core import grow_system, start_root


def test_grow(iterations=10):
    plant = Plant()
    root_entity = start_root(plant)
    for _ in range(iterations):
        grow_system(plant, root_entity)
    for entity, components in plant.entities.items():
        print(entity, components.print())

    return plant, root_entity


if __name__ == "__main__":
    test_grow()
