import numpy as np

class Mesh2D:
    def __init__(self, vertices: np.ndarray, edges: np.ndarray):
        assert vertices.ndim == 2 and vertices.shape[1] == 2, "Vertices must be a 2D array with shape (n, 2)"
        assert edges.ndim == 2 and edges.shape[1] == 2, "Edges must be a 2D array with shape (m, 2)"
        assert np.all(edges < len(vertices)), "Edges must reference valid vertex indices"
        assert np.all(edges >= 0), "Edges must reference non-negative vertex indices"
        self.vertices = vertices.astype(np.float32)
        self.edges = edges.astype(np.int32)
    
    def __repr__(self):
        return f"Mesh2D(vertices={self.vertices.shape[0]}, edges={self.edges.shape[0]})"
    
    def __len__(self):
        return len(self.edges)