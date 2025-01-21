from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
import rerun as rr

import impostor.components as comp
import impostor.parts as parts
from impostor.components.core import AxeNext, Vascular
from impostor.components.rigid_transformation import (
    RigidTransformation,
)
from impostor.plant import Entity, Plant


@dataclass
class VertexLayer:
    vertices: np.ndarray

    @staticmethod
    def create_ring(transform: RigidTransformation, radius: float, segments: int):
        angle = 2 * np.pi / segments
        vertices = []
        for i in range(segments):
            x = radius * np.cos(i * angle)
            y = radius * np.sin(i * angle)
            vertices.append(transform.transform_point(np.array([x, y, 0])))
        return VertexLayer(np.array(vertices))

    @staticmethod
    def from_vertices(vertices: np.ndarray):
        return VertexLayer(vertices)

    def merge_close_vertices(self, threshold: float = 0.001):
        """Merge close vertices in the layer."""
        new_vertices = [self.vertices[0]]
        for vertex in self.vertices[1:]:
            if np.linalg.norm(vertex - new_vertices[-1]) > threshold:
                new_vertices.append(vertex)


def compute_face_normal(v1, v2, v3):
    """Compute the normal of a face given its vertices."""
    edge1 = v2 - v1
    edge2 = v3 - v1
    normal = np.cross(edge1, edge2)
    magnitude = np.linalg.norm(normal)
    if magnitude < 1e-6:
        pass # TODO
        # print("Warning: Face has a normal of length 0")
    else:
        normal = normal / np.linalg.norm(normal)  # Normalize the normal
    return normal


@dataclass
class PlantMesh:
    layers: List[VertexLayer] = field(default_factory=list)
    vertices: np.ndarray = field(default_factory=lambda: np.ndarray((0, 3)))
    faces: np.ndarray = field(default_factory=lambda: np.ndarray((0, 3), dtype=int))
    normals: np.ndarray = field(default_factory=lambda: np.ndarray((0, 3)))
    uvs: np.ndarray = field(default_factory=lambda: np.ndarray((0, 2)))
    is_closed: bool = False

    def add_layer(self, layer: VertexLayer):
        self.layers.append(layer)

    def build_mesh(self, winding_clockwise: bool = True):
        if len(self.layers) == 0:
            return self

        self.vertices = np.concatenate([layer.vertices for layer in self.layers])
        self.normals = np.zeros((len(self.vertices), 3))
        self.uvs = np.zeros((len(self.vertices), 2))

        one_start = 0
        faces = []
        adjacent_faces = {}
        face_normals = []

        for i in range(len(self.layers) - 1):
            two_start = one_start + len(self.layers[i].vertices)
            one_id = 0
            two_id = 0
            one_size = len(self.layers[i].vertices)
            two_size = len(self.layers[i + 1].vertices)

            id_margin = 0 if self.is_closed else 1
            while one_id < one_size - id_margin or two_id < two_size - id_margin:
                id1 = one_start + one_id % one_size
                id2 = two_start + two_id % two_size
                id1_next = one_start + (one_id + 1) % one_size
                id2_next = two_start + (two_id + 1) % two_size

                # the positions of the vertices on the layer from 0 to 1
                up_progress = one_id / one_size
                down_progress = two_id / two_size

                if up_progress > down_progress:
                    if winding_clockwise:
                        new_face = [id1, id2_next, id2]
                    else:
                        new_face = [id1, id2, id2_next]
                    two_id += 1
                else:
                    if winding_clockwise:
                        new_face = [id1, id1_next, id2]
                    else:
                        new_face = [id1, id2, id1_next]
                    one_id += 1

                # Add the face to the list of faces
                faces.append(new_face)
                face_idx = len(faces) - 1
                # Save wich faces are adjacent to each vertex
                for vertex_id in new_face:
                    if vertex_id not in adjacent_faces:
                        adjacent_faces[vertex_id] = []
                    adjacent_faces[vertex_id].append(face_idx)

                # Compute the normal of the face
                v1, v2, v3 = self.vertices[new_face]
                normal = compute_face_normal(v1, v2, v3)
                face_normals.append(normal)

            one_start = two_start

        self.faces = np.array(faces, dtype=int)

        # Compute the normal of a vertex given its adjacent faces
        for vertex_idx in range(len(self.vertices)):
            normals = []
            for face_idx in adjacent_faces[vertex_idx]:
                normals.append(face_normals[face_idx])

            # Average the normals
            vertex_normal = np.mean(normals, axis=0)
            # warn if the normal is zero
            if np.linalg.norm(vertex_normal) < 1e-6:
                pass # TODO
                # print(
                #     f"Warning: Vertex {vertex_idx} has a normal of length {np.linalg.norm(vertex_normal)}"
                # )
            else:
                vertex_normal = vertex_normal / np.linalg.norm(
                    vertex_normal
                )  # Normalize the normal
                self.normals[vertex_idx] = vertex_normal

        return self

    def merge(self, other: "PlantMesh"):
        if other.faces.size == 0:
            return
        vertices_start = len(self.vertices)
        self.vertices = np.concatenate([self.vertices, other.vertices])
        self.normals = np.concatenate([self.normals, other.normals])
        self.uvs = np.concatenate([self.uvs, other.uvs])
        try:
            self.faces = np.concatenate([self.faces, other.faces + vertices_start])
        except Exception as e:
            print(e)
            print("other.faces", other.faces)
            print("vertices_start", vertices_start)
            print("self.faces", self.faces)
            print("self.vertices", self.vertices)
            print("other.vertices", other.vertices)
            print("other.vertices + vertices_start", other.vertices + vertices_start)
            print("other.faces + vertices_start", other.faces + vertices_start)
            raise e
        self.faces = np.concatenate([self.faces, other.faces + vertices_start])

    def rr_log(self):
        # self.vertices = np.append(self.vertices, np.zeros((1, 3)), axis=0)
        rr.log(
            "mesh",
            rr.Mesh3D(
                triangle_indices=self.faces,
                vertex_positions=self.vertices,
                vertex_texcoords=self.uvs,
                vertex_normals=self.normals,
            ),
        )
        rr.log(
            "wireframe",
            rr.LineStrips3D(self.vertices[self.faces], radii=0.0005),
        )
        rr.log(
            "normals",
            rr.LineStrips3D(
                np.stack((self.vertices, self.vertices + self.normals * 0.05), axis=1),
                radii=0.0005,
                colors=[255, 255, 0],
            ),
        )


def create_stem_vertex_layers(
    plant: Plant, entity: Entity, rings: Optional[List[VertexLayer]] = None
) -> List[VertexLayer]:
    if rings is None:
        rings = []

    comps = plant.get_components(entity)

    resolution = 10

    if RigidTransformation in comps:
        radius = 0.01
        if Vascular in comps:
            radius = comps.get_by_type(Vascular).radius
        transform = comps.get_by_type(RigidTransformation)
        ring = VertexLayer.create_ring(transform, radius, resolution)
        rings.append(ring)
        if AxeNext in comps:
            return create_stem_vertex_layers(
                plant, comps.get_by_type(AxeNext).next, rings
            )

    return rings


def create_blade_mesh(plant: Plant, leaf_meta: parts.Leaf) -> PlantMesh:
    def create_half(is_left:bool):
        bases = leaf_meta.lateral_vein_bases_left if is_left else leaf_meta.lateral_vein_bases_right
        layers: List[VertexLayer] = []
        for vein_base in bases:
            entities = [vein_base]
            while comp.AxeNext in plant.get_components(entities[-1]):
                entities.append(
                    plant.get_components(entities[-1]).get_by_type(comp.AxeNext).next
                )
            poses = []
            for entity in entities:
                poses.append(
                    plant.get_components(entity)
                    .get_by_type(RigidTransformation)
                    .translation
                )
            layers.append(VertexLayer.from_vertices(np.array(poses)))
        # Add one more layer to close the mesh at the tip of the leaf
        tip = leaf_meta.midrib_entities[-1]
        tip_pos = plant.get_components(tip).get_by_type(RigidTransformation).translation
        layers.append(VertexLayer.from_vertices(np.array([tip_pos])))

        for layer in layers:
            layer.merge_close_vertices()

        return PlantMesh(layers=layers, is_closed=False).build_mesh(winding_clockwise=is_left)

    blade = create_half(is_left=True)
    blade.merge(create_half(is_left=False))

    return blade


def create_plant_mesh(plant: Plant) -> PlantMesh:
    mesh = PlantMesh()

    roots = (
        plant.query()
        .with_component(comp.Vascular)
        .without_component(comp.AxePrev)
        .filter(
            lambda comps: comps.get_by_type(comp.Vascular).type
            == comp.VascularType.STEM
        )
        .entities()
    )

    for root in roots:
        rings = create_stem_vertex_layers(plant, root)
        mesh.merge(PlantMesh(layers=rings, is_closed=True).build_mesh())

    for leaf in plant.query().with_component(parts.Leaf).entities():
        leaf_meta = plant.get_components(leaf).get_by_type(parts.Leaf)
        leaf_mesh = create_blade_mesh(plant, leaf_meta)
        mesh.merge(leaf_mesh)

    return mesh
