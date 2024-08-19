from dataclasses import dataclass, field
from typing import Dict, NewType
from scipy.spatial.transform._rotation import Rotation
from enum import Enum

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
            print("add", component)
            self.entities[entity].add(component)
        return entity

    def get_components(self, entity: Entity):
        return self.entities.get(entity)


class Query:
    def __init__(self, entities: Dict[Entity, TypeSet]):
        self.entities = entities

    def with_component(self, component_cls):
        for entity, components in self.entities.items():
            if component_cls not in components:
                del self.entities[entity]
        return self

    def without_component(self, component_cls):
        for entity, components in self.entities.items():
            if component_cls in components:
                del self.entities[entity]
        return self

    def single(self):
        try:
            return list(self.entities.keys())[0]
        except IndexError:
            None


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


# @dataclass
# class Branches:
#     branches: list[Entity] = field(default_factory=list)
#     directions: list[Rotation] = field(default_factory=list)

def start_root(plant: Plant):
    q = plant.query()
    print(q)
    print(Query)
    root = q.without_component(AxePrev).single()
    if root is None:
        root = plant.create_entity(Root(), Stem())
    return root


def grow_system(plant: Plant, entity: Entity):
    comps = plant.get_components(entity)
    if Stem in comps:
        stem = comps.get_by_type(Stem)
        if stem.length < 1:
            stem.length += 0.4
        else:
            if AxeNext in comps:
                grow_system(plant, comps.get_by_type(AxeNext).next)
            else:
                next = plant.create_entity(Stem(), AxePrev(entity))
                comps.add(AxeNext(next))
        stem = comps.get_by_type(Stem)
        stem.radius *= 1.001


if __name__ == "__main__":
    plant = Plant()
    root_entity = start_root(plant)
    for _ in range(10):
        grow_system(plant, root_entity)
    for entity, components in plant.entities.items():
        print(entity, components.print())
