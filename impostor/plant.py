from dataclasses import dataclass, field
from typing import Dict, NewType
from scipy.spatial.transform._rotation import Rotation

from impostor.type_set import TypeSet

Entity = NewType("Entity", int)

class Plant:
    entities: Dict[Entity, TypeSet] = {}

    def query(self):
        return Query(self.entities)

    def create_entity(self, *components):
        entity = Entity(len(self.entities))
        self.entities[entity] = TypeSet()
        for component in components:
            self.entities[entity].add(component)
        return entity

class Query:
    def __new__(self, entities: list[Entity]):
        self.entities = entities
        return self

    def with_component(self, Component):
        self.entities.filter(lambda entity: Component in entity)
        return self

    def without_component(self, component_cls):
        self.entities.filter(lambda entity: component_cls not in entity)
        return self

    def single(self):
        return self.entities[0]

@dataclass
class Stem:
    length: float = 0.0
    radius: float = 0.0
    rotation: Rotation = Rotation.identity()

@dataclass
class AxeNext:
    next: Entity

@dataclass
class AxePrev:
    prev: Entity

@dataclass
class Root:
    pass

@dataclass
class Branches:
    branches: list[Entity] = field(default_factory=list)
    directions: list[Rotation] = field(default_factory=list)

def start_root(plant: Plant):
    q = plant.query()
    root = q.without_component(component_cls=AxePrev).single()
    if root is None:
        plant.create_entity([Root(), Stem()])


if __name__ == "__main__":
    plant = Plant()
    start_root(plant)
    print(plant.entities)