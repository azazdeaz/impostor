from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
import rerun as rr

import impostor.components as comp
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


@dataclass
class PlantMesh:
    layers: List[VertexLayer] = field(default_factory=list)
    vertices: np.ndarray = field(default_factory=lambda: np.ndarray((0, 3)))
    faces: np.ndarray = field(default_factory=lambda: np.ndarray((0, 3)))
    normals: np.ndarray = field(default_factory=lambda: np.ndarray((0, 3)))
    uvs: np.ndarray = field(default_factory=lambda: np.ndarray((0, 2)))
    is_closed: bool = False

    def add_layer(self, layer: VertexLayer):
        self.layers.append(layer)

    def build_mesh(self):
        if len(self.layers) == 0:
            return self

        self.vertices = np.concatenate([layer.vertices for layer in self.layers])
        self.normals = np.zeros((len(self.vertices), 3))
        self.uvs = np.zeros((len(self.vertices), 2))

        one_start = 0
        faces = []
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
                    faces.append([id1, id2_next, id2])
                    two_id += 1
                else:
                    faces.append([id1, id1_next, id2])
                    one_id += 1

            one_start = two_start

        self.faces = np.array(faces)

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
        self.vertices = np.append(self.vertices, np.zeros((1, 3)), axis=0)
        rr.log(
            "mesh",
            rr.Mesh3D(
                triangle_indices=self.faces,
                vertex_positions=self.vertices,
                vertex_texcoords=self.uvs,
                vertex_normals=self.normals,
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


def create_blade_mesh(plant: Plant, leaf_meta: comp.LeafMeta) -> PlantMesh:
    def create_half(bases: List[Entity]):
        layers = []
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

        return PlantMesh(layers=layers, is_closed=False).build_mesh()

    blade = create_half(leaf_meta.lateral_vein_bases_left)
    blade.merge(create_half(leaf_meta.lateral_vein_bases_right))

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

    for leaf in plant.query().with_component(comp.LeafMeta).entities():
        leaf_meta = plant.get_components(leaf).get_by_type(comp.LeafMeta)
        leaf_mesh = create_blade_mesh(plant, leaf_meta)
        mesh.merge(leaf_mesh)

    return mesh
