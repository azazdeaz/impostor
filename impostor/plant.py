from typing import Dict

from impostor.type_set import TypeSet

class Plant:
    entities: Dict[int, TypeSet] = {}

if __name__ == "__main__":
    plant = Plant()
    print(plant.entities)