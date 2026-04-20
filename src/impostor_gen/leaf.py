from typing import List

import numpy as np

from impostor_gen.material import Material
from impostor_gen.mesh.leaf_mesh_context import LeafMeshContext
from impostor_gen.svg_guide import SvgGuide

from .engine import (
    AgeingContext,
    BranchClose,
    BranchOpen,
    Diameter,
    F,
    MaterialKey,
    Pitch,
    Symbol,
    Yaw,
)


class LeafContext(AgeingContext):
    def __str__(self) -> str:
        return "Leaf"


def create_leaf(material: Material, svg_guide: SvgGuide, size_scale: float = 1.0) -> List[Symbol]:
    midrib_division = 8
    sec_vein_division = 4

    leaf: List[Symbol] = [
        LeafContext(),
        LeafMeshContext(),
        MaterialKey(key=material.key),
        Diameter(diameter=0.11 * size_scale),
    ]

    growth_end_age = 8

    lateral_lengths = svg_guide.get_mm("leaflet.secondary_veins.lengths", midrib_division)
    initial_lateral_yaws = svg_guide.get_degrees("leaflet.secondary_veins.start_angles", midrib_division)
    lateral_yaws = svg_guide.get_degrees("leaflet.secondary_veins.curvings", sec_vein_division)
    print("Lateral scales:", lateral_lengths)
    print("Initial lateral yaws:", initial_lateral_yaws)
    print("Lateral yaws:", lateral_yaws)
    def lateral_vein(section_idx: int, is_left: bool) -> List[Symbol]:
        symbols: List[Symbol] = [
            Yaw(
                angle=initial_lateral_yaws[section_idx]
                if is_left
                else -initial_lateral_yaws[section_idx]
            )
        ]
        length = lateral_lengths[section_idx]

        for i in range(sec_vein_division):
            symbols.append(Yaw(angle=lateral_yaws[i] if is_left else -lateral_yaws[i]))
            symbols.append(
                F(
                    value_se=(0.01 * size_scale, length),
                    age_se=(0, growth_end_age),
                    age_context_type=LeafContext,
                )
            )
            symbols.append(
                Pitch(
                    value_se=(48, -2),
                    age_se=(2, growth_end_age),
                    age_context_type=LeafContext,
                )
            )

        return symbols

    # Create midrib with secondary veins
    for mr in range(midrib_division):
        symbols: List[Symbol] = []

        midrib_length = 400
        length = midrib_length / midrib_division

        symbols.append(
            F(
                value_se=(0.01 * size_scale, length),
                age_se=(0, growth_end_age),
                age_context_type=LeafContext,
            )
        )

        symbols.append(
            Pitch(
                value_se=(48, np.random.uniform(-3, 5)),
                age_se=(2, growth_end_age),
                age_context_type=LeafContext,
            )
        )

        # Dont add lateral veins on the tip
        if mr < midrib_division - 1:
            symbols.append(BranchOpen())
            symbols.append(Diameter(diameter=0.1 * size_scale))
            symbols.append(Yaw(angle=-90))

            # Add secondary vein on left side
            symbols.extend(lateral_vein(mr, is_left=True))

            symbols.append(BranchClose())
            symbols.append(BranchOpen())
            symbols.append(Diameter(diameter=0.1 * size_scale))
            symbols.append(Yaw(angle=90))

            # Add secondary vein on right side
            symbols.extend(lateral_vein(mr, is_left=False))

            symbols.append(BranchClose())

        leaf.extend(symbols)

    return [BranchOpen()] + leaf + [BranchClose()]


def create_trifoliate_leaf(material: Material, svg_guide: SvgGuide, size_scale: float = 1.0) -> List[Symbol]:
    leaf_center = create_leaf(material, svg_guide, size_scale=size_scale)
    # leaf_left = create_leaf(material, svg_guide, size_scale=size_scale)
    # leaf_right = create_leaf(material, svg_guide, size_scale=size_scale)

    def insert_after_branch_open(
        symbols: List[Symbol], to_insert: List[Symbol]
    ) -> None:
        for i, symbol in enumerate(symbols):
            if isinstance(symbol, BranchOpen):
                symbols[i + 1 : i + 1] = to_insert
                return

    insert_after_branch_open(leaf_center, [Pitch(angle=20)])
    # insert_after_branch_open(leaf_left, [Yaw(angle=-85)])
    # insert_after_branch_open(leaf_right, [Yaw(angle=85)])

    return leaf_center
    return leaf_left + leaf_center + leaf_right
