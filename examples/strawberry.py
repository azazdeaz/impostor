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
    MaterialKey,
)
from impostor_gen.engine.symbol import Symbol
from impostor_gen.leaf import create_trifoliate_leaf
from impostor_gen.material import Material, MaterialRegistry
from impostor_gen.mesh_builder import generate_blueprints, generate_mesh
from impostor_gen.mesh_utils import log_mesh
from impostor_gen.usd_animation import UsdAnimation
import numpy as np

leaf_material = Material(
    key="leaf",
    texture_base_color=Path("textures/central_leaflet_color_cropped.png"),
    texture_normal_map=Path("textures/central_leaflet_normal_cropped.png"),
    texture_opacity_map=Path("textures/central_leaflet_mask_cropped.png"),
    texture_displacement_map=Path("textures/central_leaflet_bump_cropped.png"),
)
stem_material = Material(
    key="stem",
    texture_base_color=Path("textures/stem_color_cropped.png"),
    texture_normal_map=Path("textures/stem_normal_cropped.png"),
    # texture_opacity_map=Path("textures/stem_mask_cropped.png"),
    # texture_displacement_map=Path("textures/stem_bump_cropped.png"),
)


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
                    MaterialKey(key="stem"),
                    Diameter(diameter=0.21),
                    F(length=0.0),
                    Roll(angle=roll),
                    Pitch(angle=pitch),
                    StemContext(
                        target_length=5.2 + age * 0.02,
                        growth_speed=0.2,
                        section_length=0.8,
                    ),
                    StemTip(),
                    *create_trifoliate_leaf(leaf_material),
                    BranchClose(),
                ]
            )


class XY(Symbol):
    x: float = 0.0
    y: float = 0.0


def main():
    materials = MaterialRegistry()
    materials.register(leaf_material)
    materials.register(stem_material)

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

    anim = UsdAnimation(materials_registry=materials, fps=10.0)

    iterations = 10
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

        # anim.add_next_frame(meshes)

        # if i == iterations - 1:
        #     meshes.to_usd(materials).Save()
        #     for j, mesh in enumerate(meshes.submeshes):
        #         Path(f"exports/strawberry_plant_{j}").mkdir(parents=True, exist_ok=True)
        #         mesh.to_trimesh(materials).export( # type: ignore
        #             f"exports/strawberry_plant_{j}/model.glb"
        #         )
        #         mesh.to_trimesh(materials).export( # type: ignore
        #             f"exports/strawberry_plant_{j}/model.obj"
        #         )

    # anim.save("strawberry_plant_animation.usd")


if __name__ == "__main__":
    main()
