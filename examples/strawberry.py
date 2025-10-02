from pathlib import Path

from pydantic import BaseModel
import rerun as rr

from impostor_gen.engine import (
    BranchClose,
    BranchOpen,
    Context,
    Diameter,
    F,
    LSystem,
    Pitch,
    Roll,
    Rule,
    Writer,
    InterpolateRule,
    AgeingRule,
    AgeingContext,
    StemContext,
    StemGrowthRule,
    StemTip,
)
from impostor_gen.engine.symbol import Symbol
from impostor_gen.leaf import create_trifoliate_leaf
from impostor_gen.material import Material, MaterialRegistry
from impostor_gen.mesh_builder import generate_blueprints, generate_mesh
from impostor_gen.mesh_utils import log_mesh
import numpy as np

class Crown(AgeingContext):
    shoot_period: int = 12  # How many iterations between new shoots
    max_shoots: int = 3  # Maximum age before the crown stops producing new shoots
    angle_step: float = 137.5  # Angle step in degrees for new shoots


class IterateCrown(Rule, BaseModel):
    shoots: int = 0

    def apply(self, writer: "Writer", context: "Context"):
        crown = writer.peek(0).model_copy()
        if not isinstance(crown, Crown):
            return
        
        age = crown.age

        shoot_count = np.minimum(1 + age // crown.shoot_period, crown.max_shoots)

        if shoot_count > self.shoots:
            self.shoots += 1

            roll = (age // crown.shoot_period) * crown.angle_step
            pitch = -50 + age // crown.shoot_period * 6.0
            writer.write(
                [
                    crown,
                    BranchOpen(),
                    Diameter(diameter=0.21),
                    F(length=0.0),
                    Roll(angle=roll),
                    Pitch(angle=pitch),
                    StemContext(target_length=5.2 + age * 0.02, growth_speed=0.2, section_length=0.8),
                    StemTip(),
                    *create_trifoliate_leaf(),
                    BranchClose(),
                ]
            )


class GrowStem(Rule):
    def apply(self, writer: "Writer", context: "Context"):
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

        if length_without_branch < tip.max_length:
            # Default "grow" behavior
            writer.write(
                [
                    Pitch(angle=-2),
                    Diameter(diameter=0.2),
                    F(length=0.9),
                    tip.model_copy(),
                ]
            )
class XY(Symbol):
    x: float = 0.0
    y: float = 0.0

def main():
    materials = MaterialRegistry()
    # materials.register(
    #     "leaf",
    #     Material(
    #         texture_base_color=Path("central_leaflet_color_cropped.png"),
    #         texture_normal_map=Path("central_leaflet_normal_cropped.png"),
    #         texture_opacity_map=Path("central_leaflet_mask_cropped.png"),
    #         texture_displacement_map=Path("central_leaflet_bump_cropped.png"),
    #     ),
    # )

    # Define an L-system
    lsystem = LSystem(
        world=[Crown(), XY()],
        rules=[
            InterpolateRule(),
            StemGrowthRule(),
            AgeingRule(),
            IterateCrown(),
        ],
    )

    rr.init("rerun_example_my_data", spawn=True)

    iterations = 100
    for i in range(iterations):
        rr.set_time("frame_idx", sequence=i)
        lsystem.iterate()
        print(f"Iteration {i}, world size: {len(lsystem.world)}")
        # if i % 5 == 0 or i == iterations - 1:
        blueprints = generate_blueprints(lsystem.world)
        # log_transforms(blueprints)
        meshes = generate_mesh(blueprints)
        log_mesh(meshes, materials)
        lsystem.log_graph()
        lsystem.log_as_markdown()

        if i == iterations - 1:
            meshes.to_usd(materials).Save()
            for j, mesh in enumerate(meshes.submeshes):
                Path(f"exports/strawberry_plant_{j}").mkdir(parents=True, exist_ok=True)
                mesh.to_trimesh(materials).export( # type: ignore
                    f"exports/strawberry_plant_{j}/model.glb"
                )  
                mesh.to_trimesh(materials).export( # type: ignore
                    f"exports/strawberry_plant_{j}/model.obj"
                )  


if __name__ == "__main__":
    main()
