"""Shared strawberry plant L-system growth."""

from pydantic import BaseModel

from impostor_gen.engine import (
    AgeingContext,
    AgeingRule,
    BranchClose,
    BranchOpen,
    Context,
    Diameter,
    F,
    InterpolateRule,
    LSystem,
    MaterialKey,
    Pitch,
    Roll,
    Rule,
    StemContext,
    StemGrowthRule,
    StemTip,
    Writer,
)
from impostor_gen.leaf import create_trifoliate_leaf
from impostor_gen.material import Material
from impostor_gen.mesh.mesh_builder import generate_blueprint
from impostor_gen.mesh.stem_mesh_context import StemMeshContext
from impostor_gen.svg_guide import SvgGuide

leaf_material = Material(key="leaf", diffuse_color=(0.2, 0.6, 0.1))
leaf_svg_guide = SvgGuide("assets/strawberry_guides.svg")


class Crown(AgeingContext):
    shoot_period: int = 12
    max_shoots: int = 3 
    angle_step: float = 137.5


class IterateCrown(Rule, BaseModel):
    shoots: int = 0

    def apply(self, writer: "Writer", context: "Context"):
        crown = writer.peek(0).model_copy()
        if not isinstance(crown, Crown):
            return
        age = crown.age
        if min(1 + age // crown.shoot_period, crown.max_shoots) > self.shoots:
            self.shoots += 1
            roll = (age // crown.shoot_period) * crown.angle_step
            pitch = -30 + age // crown.shoot_period * 6.0
            writer.write([
                crown,
                BranchOpen(),
                MaterialKey(key="stem"),
                StemMeshContext(),
                Diameter(diameter=0.0035),
                F(length=0.0),
                Roll(angle=roll),
                Pitch(angle=pitch),
                StemContext(
                    target_length=0.13 + age * 0.0005,
                    growth_speed=0.01,
                    section_length=0.02,
                    diameter_start=0.0035,
                    diameter_end=0.0022,
                ),
                StemTip(),
                *create_trifoliate_leaf(leaf_material, leaf_svg_guide, size_scale=0.025),
                BranchClose(),
            ])


def grow_strawberry_plant(iterations: int = 50):
    lsystem = LSystem(
        world=[Crown()],
        rules=[InterpolateRule(), StemGrowthRule(), AgeingRule(), IterateCrown()],
    )
    for _ in range(iterations):
        lsystem.iterate()
    return generate_blueprint(lsystem.world)
