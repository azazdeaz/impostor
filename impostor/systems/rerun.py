from dataclasses import asdict

import rerun as rr
from rerun.components.color import Color
from scipy.spatial.transform._rotation import Rotation

import impostor.components as comp
import impostor.parts as parts
from impostor.plant import Entity, Plant
import numpy as np


class Palette:
    Red = Color([228, 26, 28])
    Blue = Color([55, 126, 184])
    Green = Color([77, 175, 74])
    Purple = Color([152, 78, 163])
    Orange = Color([255, 127, 0])
    Yellow = Color([255, 255, 51])
    Brown = Color([166, 86, 40])
    Pink = Color([247, 129, 191])
    Gray = Color([153, 153, 153])
    SkyBlue = Color([166, 206, 227])


def log_path(entity: Entity):
    return f"nodes/{entity}"


def rr_log_components(plant: Plant):
    """Log all components of all entities in the plant."""

    for entity in plant.query()._entities:
        comps = plant.get_components(entity)
        for component in comps:
            values = {}
            try:
                items = asdict(component).items()
            except Exception as e:
                raise ValueError(f"Failed to log component {component}:\n{e}")
            if hasattr(component, "as_component_batches"):
                rr.log(f"nodes/{entity}", component)
                continue

            if len(items) == 0:
                values[f"cmp.{component.__class__.__name__}"] = "✓"
            else:
                for key, value in items:
                    try:
                        # if the value is a list, log the first element
                        if isinstance(value, list):
                            value = value[0]
                        rr.any_value.AnyBatchValue(key, value)
                        values[f"cmp.{component.__class__.__name__}.{key}"] = value
                    except Exception as _:
                        pass

            rr.log(f"nodes/{entity}", rr.AnyValues(**values))


def rr_log_graph(plant: Plant):
    """Log the graph of the plant."""
    node_ids = []
    node_labels = []
    node_colors = []
    edges = []

    for entity in plant.query()._entities:
        if comp.AxeNext in plant.get_components(
            entity
        ) or comp.AxePrev in plant.get_components(entity):
            node_ids.append(log_path(entity))
            label = ""

            if comp.Root in plant.get_components(entity):
                label += "R"
            if parts.Vascular in plant.get_components(entity):
                label += "V"
            if comp.Attachments in plant.get_components(entity):
                label += "A"
            if comp.AttachmentOrientation in plant.get_components(entity):
                label += "O"
            if comp.GrowthTip in plant.get_components(entity):
                label += "T"

            node_labels.append(f"{entity} {label}")
            node_colors.append(Palette.Green)

            if comp.AxeNext in plant.get_components(entity):
                next_entity = (
                    plant.get_components(entity).get_by_type(comp.AxeNext).next
                )
                edges.append((log_path(entity), log_path(next_entity)))

        if parts.Spring in plant.get_components(entity):
            spring = plant.get_components(entity).get_by_type(parts.Spring)
            node_ids.append(log_path(entity))
            node_labels.append(f"{entity} S")
            node_colors.append(Palette.Blue)
            edges.append((log_path(entity), log_path(spring.entity_a)))
            edges.append((log_path(entity), log_path(spring.entity_b)))

    rr.log(
        "nodes",
        rr.GraphNodes(
            node_ids=node_ids,
            labels=node_labels,
            colors=node_colors,
        ),
        rr.GraphEdges(edges=edges, graph_type=rr.GraphType.Directed),
    )


def rr_log_collideres(plant: Plant):
    colliders = (
        plant.query()
        .with_components(parts.Collider, parts.RigidTransformation)
        .values()
    )
    centers = [
        comps.get_by_type(parts.RigidTransformation).translation for comps in colliders
    ]
    half_sizes = [[comps.get_by_type(parts.Collider).radius] * 3 for comps in colliders]

    rr.log(
        "colliders",
        rr.Ellipsoids3D(
            centers=centers,
            half_sizes=half_sizes,
            colors=[Palette.SkyBlue],
        ),
    )


def rr_log_transforms_system(plant: Plant):
    # Log all entities with a RigidTransformation component
    transforms = plant.query().with_component(parts.RigidTransformation).entities()
    for entity in transforms:
        comps = plant.get_components(entity)
        transform = comps.get_by_type(parts.RigidTransformation)

        # Draw a circle around the stem
        if parts.Vascular in comps:
            vascular = comps.get_by_type(parts.Vascular)
            r = vascular.radius
            # if comps.get_by_type(parts.Vascular).type == parts.VascularType.STEM:
            rr.log(
                log_path(entity),
                rr.LineStrips3D(
                    [
                        [
                            transform.transform_point(
                                np.array([np.cos(theta) * r, np.sin(theta) * r, 0])
                            )
                            for theta in np.linspace(0, 2 * np.pi, 10)
                        ],
                        [
                            transform.translation,
                            transform.combine(
                                parts.RigidTransformation.from_z_translation(r)
                            ).translation,
                        ],
                        [
                            transform.translation,
                            transform.combine(
                                parts.RigidTransformation.from_x_translation(r * 3)
                            ).translation,
                        ],
                    ],
                    colors=Palette.Green,
                ),
            )
        else:
            rr.log(
                log_path(entity),
                rr.Points3D(transform.translation, colors=Palette.Green),
            )

    # Log all springs as lines between their entities
    springs = plant.query().with_component(parts.Spring).entities()
    for entity in springs:
        spring = plant.get_components(entity).get_by_type(parts.Spring)
        transform_a = plant.get_components(spring.entity_a).get_by_type(
            parts.RigidTransformation
        )
        transform_b = plant.get_components(spring.entity_b).get_by_type(
            parts.RigidTransformation
        )

        if transform_a is None or transform_b is None:
            rr.log(
                log_path(entity),
                rr.Points3D([0.0, 0.0, 0.0], colors=Palette.Yellow),
            )
        else:
            rr.log(
                f"nodes/{entity}",
                rr.LineStrips3D(
                    [transform_a.translation, transform_b.translation],
                    colors=Palette.Blue,
                ),
            )

    # Log everithing else as a point so they will be visible in the 3D view
    for entity in plant.entities.keys():
        if entity not in transforms and entity not in springs:
            rr.log(
                log_path(entity),
                rr.Points3D([0.0, 0.0, 0.0], colors=Palette.Yellow),
            )
