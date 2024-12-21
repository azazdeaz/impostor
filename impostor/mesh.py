import numpy as np

class PlantMesh:
    def __init__(self):
        self.vertices = []
        self.faces = []
        self.normals = []
        self.uvs = []

    def from_layers(self, layers: np.ndarray, closed_loops: False) -> 'PlantMesh':
        """
        Create a PlantMesh by creating faces between the layers of vertices
        Args:
            layers: list of vertex lists, each list representing a layer of the mesh
            closed_loops: if True, the layers will be connected at the ends (for stems)
        """

        mesh = PlantMesh()

        for i in range(len(layers) - 1):
            layer = layers[i]
            next_layer = layers[i + 1]

            nearest_on_next = []
            for vertex in layer:
                nearest = min(next_layer, key=lambda x: np.linalg.norm(x - vertex))
                nearest_on_next.append(nearest)

            
            # for j in range(len(layer)):
                
