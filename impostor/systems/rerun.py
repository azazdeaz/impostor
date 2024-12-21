from dataclasses import asdict

import rerun as rr
from rerun.components.color import Color
from scipy.spatial.transform._rotation import Rotation

import impostor.components as comp
from impostor.plant import Entity, Plant

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

def rr_log_components(plant: Plant):
    """Log all components of all entities in the plant."""

    for entity in plant.query()._entities:
        comps = plant.get_components(entity)
        for comp in comps:
            values = {}
            items = asdict(comp).items()
            if len(items) == 0:
                values[f"comp.{comp.__class__.__name__}"] = "Empty"
            else:
                for key, value in items:
                    try:
                        if isinstance(value, Rotation):
                            value = value.as_euler("xyz")
                        rr.any_value.AnyBatchValue(key, value)
                        values[f"comp.{comp.__class__.__name__}.{key}"] = value
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
        if comp.Vascular in plant.get_components(entity):
            node_ids.append(entity)
            node_labels.append(f"{entity} Vascular")
            node_colors.append(Palette.Green)

            if comp.AxeNext in plant.get_components(entity):
                next_entity = (
                    plant.get_components(entity).get_by_type(comp.AxeNext).next
                )
                edges.append((entity, next_entity))

        if comp.Spring in plant.get_components(entity):
            spring = plant.get_components(entity).get_by_type(comp.Spring)
            node_ids.append(entity)
            node_labels.append(f"{entity} Spring")
            node_colors.append(Palette.Blue)
            edges.append((spring.entity_a, entity))
            edges.append((spring.entity_b, entity))


    rr.log(
        "nodes",
        rr.GraphNodes(
            node_ids=node_ids,
            labels=node_labels,
            colors=node_colors,
        ),
        rr.GraphEdges(edges=edges),
    )

def rr_log_transforms_system(
    plant: Plant
):
    # Log all entities with a RigidTransformation component
    entities = plant.query().with_component(comp.RigidTransformation).entities()
    for entity in entities:
        transform = plant.get_components(entity).get_by_type(comp.RigidTransformation)
        rr.log(
            f"nodes/{entity}",
            rr.Transform3D(
                translation=transform.translation,
                quaternion=transform.rotation.as_quat(),
            ),
        )

    # Log all springs as lines between their entities
    springs = plant.query().with_component(comp.Spring).entities()
    for entity in springs:
        spring = plant.get_components(entity).get_by_type(comp.Spring)
        transform_a = plant.get_components(spring.entity_a).get_by_type(comp.RigidTransformation)
        transform_b = plant.get_components(spring.entity_b).get_by_type(comp.RigidTransformation)

        if transform_a is None or transform_b is None:
            continue

        rr.log(
            f"nodes/{entity}",
            rr.LineStrips3D(
                [transform_a.translation, transform_b.translation],
                colors=Palette.Blue,
            ),
        )