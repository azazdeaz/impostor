from copy import copy
from typing import Callable, Dict, Iterable, NewType, Tuple

from impostor.type_set import TypeSet

Entity = NewType("Entity", int)


class Plant:
    entities: Dict[Entity, TypeSet] = {}

    def query(self):
        return Query(copy(self.entities))

    def create_entity(self, *components):
        entity = Entity(len(self.entities))
        self.entities[entity] = TypeSet()
        for component in components:
            self.entities[entity].add(component)
        return entity

    def get_components(self, entity: Entity):
        return self.entities.get(entity)
     
    def add_components(self, entity: Entity, *components):
        for component in components:
            self.entities[entity].add(component)
    
    def remove_components(self, entity: Entity, *components):
        for component in components:
            self.entities[entity].remove(component)

class Query:
    def __init__(self, entities: Dict[Entity, TypeSet]):
        self._entities = entities

    def with_component(self, component_cls) -> "Query":
        remove = [
            entity
            for entity, components in self._entities.items()
            if component_cls not in components
        ]
        for entity in remove:
            del self._entities[entity]
        return self
    
    def with_components(self, *component_clss) -> "Query":
        for component_cls in component_clss:
            self.with_component(component_cls)
        return self

    def without_component(self, component_cls) -> "Query":
        remove = [
            entity
            for entity, components in self._entities.items()
            if component_cls in components
        ]
        for entity in remove:
            del self._entities[entity]
        return self

    def filter(self, func: Callable[[TypeSet], bool]) -> "Query":
        remove = [
            entity
            for entity, components in self._entities.items()
            if not func(components)
        ]
        for entity in remove:
            del self._entities[entity]
        return self

    def single(self) -> Entity:
        try:
            return list(self._entities.keys())[0]
        except IndexError:
            None
    
    def entities(self) -> Iterable[Entity]:
        return self._entities.keys()
    
    def values(self) -> Iterable[TypeSet]:
        return self._entities.values()
    
    def items(self) -> Iterable[Tuple[Entity, TypeSet]]:
        return self._entities.items()
