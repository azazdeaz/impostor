import numpy as np

class Vec3:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.xyz = np.array([x, y, z], dtype=float)
    
    def __add__(self, other: 'Vec3 | float'):
        if isinstance(other, Vec3):
            return Vec3(*(self.xyz + other.xyz))
        else:
            return Vec3(*(self.xyz + other))