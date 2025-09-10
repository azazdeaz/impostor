import numpy as np
from impostor_gen.trail import Trail
from impostor_gen.extrude import extrude_mesh2d_along_points
from impostor_gen.mesh2d import Mesh2D
from impostor_gen.l_systems import LSystem, Stem, F, Symbol, Writer, Yaw, Pitch, Roll, T, BasicRule, Rule
import rerun as rr

class Tip(Symbol):
    pass

class Twist(Rule):
    def apply(self, writer: "Writer"):
        symbol = writer.peek(0)
        if isinstance(symbol, Yaw):
            writer.write([Yaw(angle=symbol.angle * 0.88)])


def main():
    # Define an L-system
    lsystem = LSystem(
        world=[Stem(), F(), Tip()],
        rules=[
            BasicRule(left=Tip, right=[Yaw(angle=-50), F(length=0.9), Tip()]),
            Twist()
        ]
    )


    rr.init("rerun_example_my_data", spawn=True)


    for i in range(40):
        rr.set_time("frame_idx", sequence=i)
        lsystem.iterate()
        extruded_mesh = lsystem.interpret()
        rr.log("extruded_mesh", extruded_mesh.to_rerun())

if __name__ == "__main__":
    main()