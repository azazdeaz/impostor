from typing import Any, List

from .branch_symbols import BranchClose, BranchOpen
from .context import Context, LeafContext
from .core_symbols import F, Pitch, Yaw
from .rule import Rule, Writer
from .symbol import Symbol


class AgeLeaf(Rule):
    def apply(self, writer: Writer, context: Context):
        leaf = writer.peek(0)
        if not isinstance(leaf, LeafContext):
            return
        leaf.age += 1
        writer.write([leaf])


class Bend(Pitch):
    start_angle: float = 0.0
    end_angle: float = 45.0
    start_age: int = 2
    end_age: int = 23

    def model_post_init(self, context: Any) -> None:
        self.update_with_age(0)

    def update_with_age(self, age: int) -> None:
        if age < self.start_age:
            self.angle = self.start_angle
        elif age > self.end_age:
            self.angle = self.end_angle
        else:
            t = (age - self.start_age) / (self.end_age - self.start_age)
            self.angle = self.start_angle + t * (self.end_angle - self.start_angle)


class BendLeaf(Rule):
    def apply(self, writer: Writer, context: Context):
        bend = writer.peek(0)
        if not isinstance(bend, Bend):
            return
        leaf_context = context.get(LeafContext)
        assert leaf_context is not None, "LeafContext not found in context"
        bend.update_with_age(leaf_context.age)
        writer.write([bend])


def create_leaf() -> List[Symbol]:
    midrib_division = 6
    sec_vein_division = 3
    step_size = 0.6

    leaf: List[Symbol] = [LeafContext()]

    # Create midrib with secondary veins
    for mr in range(midrib_division):
        symbols: List[Symbol] = []

        length = 0 if mr == 0 else 0.5
        symbols.append(F(length=length, start_value=0.01, end_value=length, start_age=0, end_age=8, age_context_type=LeafContext))
        symbols.append(Bend(start_age=2, end_age=8, start_angle=48, end_angle=6))

        symbols.append(BranchOpen())
        symbols.append(Yaw(angle=-90))

        # Add secondary vein on left side
        for _ in range(sec_vein_division):
            symbols.append(F(length=step_size, start_value=0.01, end_value=step_size, start_age=0, end_age=8, age_context_type=LeafContext))

        symbols.append(BranchClose())
        symbols.append(BranchOpen())
        symbols.append(Yaw(angle=90))

        # Add secondary vein on right side
        for _ in range(sec_vein_division):
            symbols.append(F(length=step_size, start_value=0.01, end_value=step_size, start_age=0, end_age=8, age_context_type=LeafContext))

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
