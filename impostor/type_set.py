class TypeSet:
    def __init__(self):
        self._type_map = {}

    def add(self, item):
        # Only add the item if the type is not already present
        if type(item) not in self._type_map:
            self._type_map[type(item)] = item

    def remove(self, item):
        # Remove the item by type
        self._type_map.pop(type(item), None)

    def get_by_type(self, type_key):
        # Return the instance of this type if exists
        return self._type_map.get(type_key)

    def __contains__(self, item):
        # Check if an instance of the item's type is in the set
        return type(item) in self._type_map

    def __iter__(self):
        # Allow iteration over instances
        return iter(self._type_map.values())

    def __len__(self):
        # Return the number of unique types
        return len(self._type_map)

if __name__ == "__main__":
    # Example usage
    type_set = TypeSet()

    type_set.add(10)
    type_set.add("hello")
    type_set.add(3.14)

    print(type_set.get_by_type(int))  # Outputs: 10
    print(type_set.get_by_type(str))  # Outputs: hello
    print(type_set.get_by_type(float))  # Outputs: 3.14
    print(type_set.get_by_type(list))  # Outputs: None (not added to type_set)

    type_set.remove("hello")
    print(type_set.get_by_type(str))  # Outputs: None (already removed)