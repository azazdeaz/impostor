from dataclasses import asdict, dataclass, field
from typing import List, Optional

import numpy as np
import rerun as rr
from scipy.spatial.transform._rotation import Rotation

import impostor.messages as messages
from impostor.components.core import AxeNext, AxePrev, Branch, Branches, Vascular
import impostor.components as comp
from impostor.components.rigid_transformation import (
    ApexTransformation,
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
        self.vertices = np.concatenate([layer.vertices for layer in self.layers])
        self.normals = np.zeros((len(self.vertices), 3))
        self.uvs = np.zeros((len(self.vertices), 2))

        one_start = 0
        faces = []
        for i in range(len(self.layers) - 1):
            two_start = one_start + len(self.layers[i].vertices)
            two_end = two_start + len(self.layers[i + 1].vertices)
            one_id = one_start
            two_id = two_start

            print(f"one_start: {one_start}, two_start: {two_start}, two_end: {two_end}")
            while one_id < two_start - 1 or two_id < two_end - 1:
                print(f"one_id: {one_id}, two_id: {two_id}")
                # distance from the current layer one vertex to the next layer two vertex
                up_distance = np.linalg.norm(
                    self.vertices[one_id] - self.vertices[two_id + 1]
                ) if two_id < two_end - 1 else np.inf
                # distance from the current layer two vertex to the next layer one vertex
                down_distance = np.linalg.norm(
                    self.vertices[two_id] - self.vertices[one_id + 1]
                ) if one_id < two_start - 1 else np.inf

                if up_distance < down_distance:
                    faces.append([one_id, two_id, two_id + 1])
                    two_id += 1
                else:
                    faces.append([one_id, two_id, one_id + 1])
                    one_id += 1
                print(f"Added face: {faces[-1]}")

            one_start = two_start
        
        self.faces = np.array(faces)

        return self

    def merge(self, other: "PlantMesh"):
        vertices_start = len(self.vertices)
        self.vertices = np.concatenate([self.vertices, other.vertices])
        self.normals = np.concatenate([self.normals, other.normals])
        self.uvs = np.concatenate([self.uvs, other.uvs])
        self.faces = np.concatenate([self.faces, other.faces + vertices_start])

    def rr_log(self):
        self.vertices = np.append(self.vertices, np.zeros((1, 3)), axis=0)
        print(self)
        rr.log(
            "mesh",
            rr.Mesh3D(
                triangle_indices=self.faces,
                vertex_positions=self.vertices,
                vertex_texcoords=self.uvs,
                vertex_normals=self.normals,
            ),
        )


def add_transforms_system(
    plant: Plant, entity: Entity, base_transform: RigidTransformation | None = None
):
    comps = plant.get_components(entity)
    if base_transform is None:
        base_transform = RigidTransformation.from_x_translation(0.2)

    comps.add(base_transform)

    rr.log(
        f"nodes/{entity}",
        rr.Transform3D(
            translation=base_transform.translation,  # rotation=base_transform.rotation.as_quat()
            quaternion=base_transform.rotation.as_quat(),
        ),
    )

    if Vascular in comps:
        stem = comps.get_by_type(Vascular)

        if stem.length <= 0:
            return

        if Branches in comps:
            for branch_entity in comps.get_by_type(Branches).branches:
                branch = plant.get_components(branch_entity).get_by_type(Branch)
                rotation = Rotation.from_euler(
                    "zyx", [branch.azimuth, branch.inclination, 0]
                )
                branch_transform = base_transform.combine(
                    RigidTransformation.from_rotation(rotation)
                )
                add_transforms_system(plant, branch_entity, branch_transform)

        next_transform = base_transform.combine(
            RigidTransformation.from_z_translation(stem.length).rotate(stem.rotation)
        )
        if AxeNext in comps:
            add_transforms_system(
                plant, comps.get_by_type(AxeNext).next, next_transform
            )
            # Remove apex transform if it has one
            if ApexTransformation in comps:
                comps.remove(ApexTransformation)
        else:
            comps.add(ApexTransformation(next_transform))
    else:
        raise ValueError("Entity does not have a stem component")


def create_stem_vertex_layers(
    plant: Plant, entity: Entity, rings: Optional[List[VertexLayer]] = None
) -> PlantMesh:
    if rings is None:
        rings = []

    comps = plant.get_components(entity)

    resolution = 3

    if Vascular in comps and RigidTransformation in comps:
        stem = comps.get_by_type(Vascular)
        transform = comps.get_by_type(RigidTransformation)
        ring = VertexLayer.create_ring(transform, stem.radius, resolution)
        rings.append(ring)
        if AxeNext in comps:
            return create_stem_vertex_layers(
                plant, comps.get_by_type(AxeNext).next, rings
            )

    if ApexTransformation in comps:
        apex_transform = comps.get_by_type(ApexTransformation)
        ring = VertexLayer.create_ring(apex_transform.transfrom, 0.01, resolution)
        rings.append(ring)

    return rings


def create_plant_mesh(plant: Plant) -> PlantMesh:
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

    print(f"Roots: {roots}")

    mesh = PlantMesh()

    for root in roots:
        print(f"Creating mesh for root {root}")
        rings = create_stem_vertex_layers(plant, root)
        mesh.merge(PlantMesh(layers=rings, is_closed=False).build_mesh())

    return mesh


def collect_all_axis_roots(
    plant: Plant, entity: Entity, roots: List[Entity] | None = None
) -> List[Entity]:
    if roots is None:
        roots = []

    if plant.get_components(entity).get_by_type(AxePrev) is None:
        roots = roots + [entity]

    if AxeNext in plant.get_components(entity):
        roots = collect_all_axis_roots(
            plant, plant.get_components(entity).get_by_type(AxeNext).next, roots
        )

    if Branches in plant.get_components(entity):
        for branch in plant.get_components(entity).get_by_type(Branches).branches:
            roots = collect_all_axis_roots(plant, branch, roots)

    return roots
