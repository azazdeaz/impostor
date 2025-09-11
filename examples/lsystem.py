from impostor_gen.l_systems import LSystem, Stem, F, Symbol, Writer, Yaw, Pitch, Roll, T, BasicRule, Rule, BranchClose, BranchOpen
import rerun as rr

class Tip(Symbol):
    def __str__(self) -> str:
        return "Tip"

class Twist(Rule):
    def apply(self, writer: "Writer"):
        symbol = writer.peek(0)
        if isinstance(symbol, Yaw):
            writer.write([Yaw(angle=symbol.angle * 0.88)])

class Branch(Rule):
    def apply(self, writer: "Writer"):
        if not isinstance(writer.peek(0), Tip):
            return

        length_without_branch = 0.0
        min_length_for_branch = 3.5
        
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
            # print("Branching!")
            writer.write([
                BranchOpen(),
                Yaw(angle=-30), 
                Stem(),
                Yaw(angle=-30), 
                F(length=length_without_branch * 0.6), # Example: branch is shorter
                Tip(),
                BranchClose(),
                # Continue the main stem
                F(length=0.9), 
                Tip(),
            ])
        else:
            # Default "grow" behavior
            # print(f"Not branching, only {leng th_without_branch} length since last branch.")
            writer.write([F(length=0.9), Tip()])

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
        extruded_mesh = lsystem.interpret()
        lsystem.log_graph()
        lsystem.log_as_markdown()
        rr.log("extruded_mesh", extruded_mesh.to_rerun())

if __name__ == "__main__":
    main()