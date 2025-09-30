from typing import List

from .branch_symbols import BranchClose, BranchOpen
from .context import Context, LeafContext
from .core_symbols import F, MaterialKey, Pitch, Yaw
from .rule import Rule, Writer
from .symbol import Symbol


class AgeLeaf(Rule):
    def apply(self, writer: Writer, context: Context):
        leaf = writer.peek(0)
        if not isinstance(leaf, LeafContext):
            return
        leaf.age += 1
        writer.write([leaf])


def create_leaf() -> List[Symbol]:
    midrib_division = 6
    sec_vein_division = 3
    step_size = 0.6

    leaf: List[Symbol] = [LeafContext(), MaterialKey(key="leaf")]

    # Create midrib with secondary veins
    for mr in range(midrib_division):
        symbols: List[Symbol] = []

        length = 0 if mr == 0 else 0.5
        symbols.append(
            F(
                value_se=(0.01, length),
                age_se=(0, 8),
                age_context_type=LeafContext,
            )
        )

        symbols.append(
            Pitch(value_se=(48, -2), age_se=(2, 8), age_context_type=LeafContext)
        )

        symbols.append(BranchOpen())
        symbols.append(Yaw(angle=-90))

        # Add secondary vein on left side
        for _ in range(sec_vein_division):
            symbols.append(
                F(
                    value_se=(0.01, step_size),
                    age_se=(0, 8),
                    age_context_type=LeafContext,
                )
            )
            symbols.append(
                Pitch(value_se=(48, -2), age_se=(2, 8), age_context_type=LeafContext)
            )

        symbols.append(BranchClose())
        symbols.append(BranchOpen())
        symbols.append(Yaw(angle=90))

        # Add secondary vein on right side
        for _ in range(sec_vein_division):  # Right side
            symbols.append(
                F(
                    value_se=(0.01, step_size),
                    age_se=(0, 8),
                    age_context_type=LeafContext,
                )
            )
            symbols.append(
                Pitch(value_se=(48, 6), age_se=(2, 8), age_context_type=LeafContext)
            )

        symbols.append(BranchClose())

        leaf.extend(symbols)

    return [BranchOpen()] + leaf + [BranchClose()]


def create_trifoliate_leaf() -> List[Symbol]:
    leaf_center = create_leaf()
    leaf_left = create_leaf()
    leaf_right = create_leaf()

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
