import rerun as rr

from impostor_gen.l_systems import (
    BasicRule,
    BranchClose,
    BranchOpen,
    F,
    LSystem,
    Pitch,
    Roll,
    Rule,
    Stem,
    Symbol,
    T,
    Tip,
    Writer,
    Yaw,
)
from impostor_gen.leaf import create_leaf
from impostor_gen.mesh_builder import generate_blueprints, log_mesh, log_transforms


class Crown(Symbol):
    age: int = 0
    shoot_period: int = 12  # How many iterations between new shoots
    max_age: int = 26  # Maximum age before the crown stops producing new shoots
    angle_step: float = 137.5  # Angle step in degrees for new shoots


class IterateCrown(Rule):
    def apply(self, writer: "Writer"):
        crown = writer.peek(0).model_copy()
        if not isinstance(crown, Crown):
            return

        age = crown.age
        crown.age += 1

        should_shoot = (age % crown.shoot_period == 0) and (age <= crown.max_age)
        if should_shoot:
            angle = (age // crown.shoot_period) * crown.angle_step
            writer.write(
                [
                    crown,
                    BranchOpen(),
                    Roll(angle=angle),
                    Pitch(angle=-20),
                    Stem(),
                    F(length=1.0, width=0.5),
                    Tip(),
                    *create_leaf(),
                    BranchClose(),
                ]
            )
        else:
            writer.write([crown])


class GrowStem(Rule):
    max_length: float = 12.0  # Limit the branching depth

    def apply(self, writer: "Writer"):
        tip = writer.peek(0)
        if not isinstance(tip, Tip):
            return

        length_without_branch = 0.0

        # Look backwards to see how much "F" length we have without a branch
        pointer = -1
        try:
            while True:
                symbol = writer.peek(pointer)
                if isinstance(symbol, F):
                    length_without_branch += symbol.length
                elif isinstance(symbol, BranchClose) or isinstance(symbol, Stem):
                    break
                pointer -= 1
        except IndexError:
            pass  # Reached the beginning of the world

        if length_without_branch < self.max_length:
            # Default "grow" behavior
            writer.write([Pitch(angle=-2), F(length=0.9, width=0.5), tip.model_copy()])


def widen_stem(symbol: F) -> list[Symbol]:
    f = symbol.model_copy()
    f.width *= 1.001
    return [f]


widen_stem_rule = BasicRule(left=F, right=widen_stem)


def main():
    # Define an L-system
    lsystem = LSystem(
        world=[Crown()],
        rules=[
            GrowStem(),
            IterateCrown(),
            widen_stem_rule
        ],
    )

    rr.init("rerun_example_my_data", spawn=True)

    # print(f"Initial: {','.join(str(s) for s in lsystem.world)}")
    for i in range(10):
        rr.set_time("frame_idx", sequence=i)
        lsystem.iterate()
        print(f"Iteration {i}, world size: {len(lsystem.world)}")
        # print(f"Iteration {i}: {','.join(str(s) for s in lsystem.world)}")
        blueprints = generate_blueprints(lsystem.world)
        log_transforms(blueprints)
        mesh = log_mesh(blueprints)
        if (i+1) % 10 == 0:
            mesh.to_trimesh().export(f"strawberry_{i:03}.glb")
        lsystem.log_graph()
        lsystem.log_as_markdown()
    


if __name__ == "__main__":
    main()
