from typing import Dict, NewType

from enum import Enum
import numpy as np

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

