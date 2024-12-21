from typing import Tuple, Type, TypeVar
import yaml
from dataclasses import dataclass, asdict
from pprint import pformat

T = TypeVar('T')

class TypeSet:
    def __init__(self):
        self._type_map = {}

    def add(self, *items):
        for item in items:
            self._type_map[type(item)] = item

    def remove(self, item):
        # Remove the item by type
        if isinstance(item, type):
            self._type_map.pop(item, None)
        else:
            self._type_map.pop(type(item), None)

    def get_by_type(self, type_key: Type[T]) -> T:
        # Return the instance of this type if exists
        return self._type_map.get(type_key)
    
    def get_or_create_by_type(self, type_key: Type[T]) -> T:
        # Return the instance of this type if exists, otherwise create it
        if type_key not in self._type_map:
            self._type_map[type_key] = type_key()
        return self._type_map[type_key]

    def __contains__(self, item_cls):
        # Check if an instance of the item's type is in the set
        return item_cls in self._type_map.keys()

    def __iter__(self):
        # Allow iteration over instances
        return iter(self._type_map.values())

    def __len__(self):
        # Return the number of unique types
        return len(self._type_map)

    def print(self):
        return str(self._type_map.values())
    
    def pprint(self):
        """Log all components as yaml"""
        return "\n\n".join([pformat(item) for item in self._type_map.values()])

if __name__ == "__main__":
    # Example usage
    type_set = TypeSet()

    class Foo:
        pass

    class Bar:
        pass

    type_set.add(10)
    type_set.add("hello")
    type_set.add(3.14)
    type_set.add(Foo())

    print(int in type_set, "=> True")  # Outputs: True
    print(Foo in type_set, "=> True")  # Outputs: True
    print(Bar in type_set, "=> False")  # Outputs: True

    print(type_set.get_by_type(int))  # Outputs: 10
    print(type_set.get_by_type(str))  # Outputs: hello
    print(type_set.get_by_type(float))  # Outputs: 3.14
    print(type_set.get_by_type(list))  # Outputs: None (not added to type_set)

    type_set.remove("hello")
    print(type_set.get_by_type(str))  # Outputs: None (already removed)