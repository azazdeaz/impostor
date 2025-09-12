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



class Twist(Rule):
    def apply(self, writer: "Writer"):
        symbol = writer.peek(0)
        if isinstance(symbol, Yaw):
            writer.write([Yaw(angle=symbol.angle * 0.88)])

class Branch(Rule):
    max_order: int = 1 # Limit the branching depth

    def apply(self, writer: "Writer"):
        tip = writer.peek(0)
        if not isinstance(tip, Tip):
            return

        length_without_branch = 0.0
        min_length_for_branch = 3.5
        
        # Look backwards to see how much "F" length we have without a branch
        if tip.order < self.max_order:
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
                pass # Reached the beginning of the world
        
        if length_without_branch >= min_length_for_branch:
            writer.write([
                BranchOpen(),
                # Yaw(angle=-30), 
                Stem(),
                Yaw(angle=-30), 
                F(length=0.9), # Example: branch is shorter
                Tip(order=tip.order + 1),
                BranchClose(),
                BranchOpen(),
                Yaw(angle=30), 
                Stem(),
                Yaw(angle=30), 
                F(length=0.9), # Example: branch is shorter
                Tip(order=tip.order + 1),
                BranchClose(),
                # Continue the main stem
                F(length=0.9), 
                tip.model_copy(),
            ])
        else:
            # Default "grow" behavior
            writer.write([T(), F(length=0.9), tip.model_copy()])

def main():
    # Define an L-system
    lsystem = LSystem(
        world=[Stem(), F(), Tip()],
        rules=[
            # Twist(),
            Branch(),
        ]
    )


    rr.init("rerun_example_my_data", spawn=True)

    # print(f"Initial: {','.join(str(s) for s in lsystem.world)}")
    for i in range(12):
        rr.set_time("frame_idx", sequence=i)
        lsystem.iterate()
        print(f"Iteration {i}, world size: {len(lsystem.world)}")
        # print(f"Iteration {i}: {','.join(str(s) for s in lsystem.world)}")
        blueprints = lsystem.generate_blueprints()
        lsystem.log_transforms(blueprints)
        lsystem.log_mesh(blueprints)
        lsystem.log_graph()
        lsystem.log_as_markdown()
        

if __name__ == "__main__":
    main()