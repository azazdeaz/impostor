from dataclasses import asdict

import rerun as rr
from scipy.spatial.transform._rotation import Rotation

import impostor.components as comp
from impostor.plant import Entity, Plant


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
            node_labels.append(f"Vascular {entity}")
            node_colors.append([77, 175, 74])

            if comp.AxeNext in plant.get_components(entity):
                next_entity = (
                    plant.get_components(entity).get_by_type(comp.AxeNext).next
                )
                edges.append((entity, next_entity))

        if comp.Spring in plant.get_components(entity):
            spring = plant.get_components(entity).get_by_type(comp.Spring)
            node_ids.append(entity)
            node_labels.append(f"Spring {entity}")
            node_colors.append([55, 126, 184])
            edges.append((spring.entity_a, entity))
            edges.append((spring.entity_b, entity))


    rr.log(
        "plant",
        rr.GraphNodes(
            node_ids=node_ids,
            labels=node_labels,
            colors=node_colors,
        ),
        rr.GraphEdges(edges=edges),
    )
