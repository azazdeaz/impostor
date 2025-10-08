from typing import List

from impostor_gen.engine.core_symbols import UV
from impostor_gen.material import Material
from PIL import Image
import numpy as np

from .engine import (
    AgeingContext,
    BranchClose,
    BranchOpen,
    F,
    MaterialKey,
    Pitch,
    Symbol,
    Yaw,
)


class LeafContext(AgeingContext):
    def __str__(self) -> str:
        return "Leaf"


def create_leaf(material: Material) -> List[Symbol]:
    midrib_division = 8
    sec_vein_division = 4
    step_size = 0.5

    leaf: List[Symbol] = [LeafContext(), MaterialKey(key=material.key)]

    growth_end_age = 8

    # calculate lateral length based on the opacity map
    if material.texture_opacity_map is not None:
        alpha_mask = Image.open(material.texture_opacity_map)
        mask_width, mask_height = alpha_mask.size
        # Convert the image to gray scale
        gray_image = alpha_mask.convert("L")

        # Fold the image by taking the maximum of the right half and the mirrored left half
        right_half = gray_image.crop((mask_width // 2, 0, mask_width, mask_height))
        left_half = gray_image.crop((0, 0, mask_width // 2, mask_height))
        left_half_mirrored = left_half.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
        folded_image = np.maximum(np.array(right_half), np.array(left_half_mirrored))

        # Get the max x index where the alpha value is above a threshold
        threshold = 10
        mask = folded_image > threshold
        max_mask_widths = np.argmin(mask, axis=1)

        # For rows where the mask is all True, argmin returns 0. Correct this to the max width.
        all_true_rows = np.all(mask, axis=1)
        max_mask_widths[all_true_rows] = folded_image.shape[1] - 1

        # Split into sections and get the max width in each section
        section_mask_widths = np.array(
            [np.median(p) for p in np.array_split(max_mask_widths, midrib_division)]
        )
        # # Add a zero width at the base
        # section_mask_widths = np.concatenate(([0], section_mask_widths))

        # Calculate lateral scales based on the section widths
        lateral_scales = section_mask_widths / (folded_image.shape[1]) * 1.02
        lateral_scales = np.clip(lateral_scales, 0.2, 1.0)
    else:
        lateral_scales = np.ones(midrib_division)

    def lateral_vein(section_idx: int, is_left: bool) -> List[Symbol]:
        symbols: List[Symbol] = []
        scale = lateral_scales[section_idx]

        for i in range(sec_vein_division):
            progress_v = section_idx / (midrib_division - 1)
            progress_u = (i+1) / (sec_vein_division)
            u = -progress_u if is_left else progress_u
            u = 0.5 + u * scale / 2.0
            symbols.append(
                UV(u=u, v=progress_v)
            )
            symbols.append(
                    F(
                        value_se=(0.01, step_size * scale),
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

        length = 0 if mr == 0 else 0.5
        symbols.append(
            UV(u=0.5, v=mr / (midrib_division - 1))
        )
        symbols.append(
            F(
                value_se=(0.01, length),
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

        symbols.append(BranchOpen())
        symbols.append(Yaw(angle=-90))

        # Add secondary vein on left side
        symbols.extend(lateral_vein(mr, is_left=True))

        symbols.append(BranchClose())
        symbols.append(BranchOpen())
        symbols.append(Yaw(angle=90))

        # Add secondary vein on right side
        symbols.extend(lateral_vein(mr, is_left=False))

        symbols.append(BranchClose())

        leaf.extend(symbols)

    return [BranchOpen()] + leaf + [BranchClose()]


def create_trifoliate_leaf(material: Material) -> List[Symbol]:
    leaf_center = create_leaf(material)
    leaf_left = create_leaf(material)
    leaf_right = create_leaf(material)

    def insert_after_branch_open(
        symbols: List[Symbol], to_insert: List[Symbol]
    ) -> None:
        for i, symbol in enumerate(symbols):
            if isinstance(symbol, BranchOpen):
                symbols[i + 1 : i + 1] = to_insert
                return

    insert_after_branch_open(leaf_center, [Pitch(angle=20)])
    insert_after_branch_open(leaf_left, [Yaw(angle=-85)])
    insert_after_branch_open(leaf_right, [Yaw(angle=85)])

    return leaf_left + leaf_center + leaf_right
