from pathlib import Path

import numpy as np
import rerun as rr
import warp as wp
from pydantic import BaseModel

import newton
import newton.examples

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
from impostor_gen.engine.symbol import Symbol
from impostor_gen.leaf import create_trifoliate_leaf
from impostor_gen.material import Material, MaterialRegistry
from impostor_gen.mesh.mesh_builder import (
    generate_blueprint,
    generate_blueprints,
    generate_mesh,
    log_transforms,
)
from impostor_gen.mesh.mesh_utils import log_mesh
from impostor_gen.mesh.stem_mesh_context import StemMeshContext
from impostor_gen.newton_builder import PlantSimulation

# ── Materials (only keys matter for physics) ────────────────────────────
leaf_material = Material(key="leaf", diffuse_color=(0.2, 0.6, 0.1))
stem_material = Material(key="stem", diffuse_color=(0.4, 0.3, 0.1))


# ── L-system rules ─────────────────────────────────────────────────────
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
        shoot_count = min(1 + age // crown.shoot_period, crown.max_shoots)
        if shoot_count > self.shoots:
            self.shoots += 1
            roll = (age // crown.shoot_period) * crown.angle_step
            pitch = -30 + age // crown.shoot_period * 6.0
            writer.write(
                [
                    crown,
                    BranchOpen(),
                    MaterialKey(key="stem"),
                    StemMeshContext(),
                    Diameter(diameter=0.005),
                    F(length=0.0),
                    Roll(angle=roll),
                    Pitch(angle=pitch),
                    StemContext(
                        target_length=0.26 + age * 0.001,
                        growth_speed=0.01,
                        section_length=0.04,
                        diameter_start=0.006,
                        diameter_end=0.0036,
                    ),
                    StemTip(),
                    *create_trifoliate_leaf(leaf_material, size_scale=0.05),
                    BranchClose(),
                ]
            )



# ── Generate plant blueprints via L-system ──────────────────────────────
def grow_plant(iterations: int = 50, visualize: bool = False):
    if visualize:
        rr.init("strawberry_newton_growth", spawn=True)
        materials = MaterialRegistry()
        materials.register(leaf_material)
        materials.register(stem_material)

    lsystem = LSystem(
        world=[Crown()],
        rules=[InterpolateRule(), StemGrowthRule(), AgeingRule(), IterateCrown()],
    )
    for i in range(iterations):
        lsystem.iterate()
        if visualize:
            rr.set_time("iteration", sequence=i)
            blueprints = generate_blueprints(lsystem.world)
            log_transforms(blueprints)
            meshes = generate_mesh(blueprints)
            log_mesh(meshes, materials)

    return generate_blueprint(lsystem.world)


# ── Newton simulation example ──────────────────────────────────────────
class Example:
    def __init__(self, viewer, args=None):
        self.fps = 20
        self.frame_dt = 1.0 / self.fps
        self.sim_time = 0.0
        self.sim_substeps = 20
        self.sim_dt = self.frame_dt / self.sim_substeps

        self.viewer = viewer

        # Grow the plant and build the simulation facade
        blueprints = grow_plant(iterations=50, visualize=False)
        self.plant = PlantSimulation(
            blueprints,
            include_stems=True,
            include_midrib=True,
            include_veins=True,
            include_cloth=True,
            rod_radius=1.0,
            bend_stiffness_modulus=1.0e9,
            stretch_stiffness_modulus=1.0e7,
            bend_damping_modulus=1.0e6,
            stretch_damping_modulus=1.0e1,
        )

        # Build the scene: plant + extras
        builder = newton.ModelBuilder()
        self.plant.add_to_builder(builder)

        builder.add_ground_plane(
            cfg=newton.ModelBuilder.ShapeConfig(ke=1e6, kd=1e1, mu=0.5)
        )
        self.sphere_pos = wp.vec3(0.2, -0.225, 0.04)
        body_sphere = builder.add_body(xform=wp.transform(p=self.sphere_pos, q=wp.quat_identity()), label="sphere")
        builder.add_shape_sphere(body_sphere, radius=0.04)
        builder.color(include_bending=True)

        # Finalize and initialize
        self.model = builder.finalize()
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()

        self.plant.initialize(self.model, self.state_0)

        self.solver = newton.solvers.SolverVBD(
            self.model,
            iterations=30,
            friction_epsilon=0.1,
            rigid_enable_dahl_friction=False,
        )

        self.viewer.set_model(self.model)
        self.viewer.show_triangles = False
        self.capture()

    def capture(self):
        if self.solver.device.is_cuda:
            with wp.ScopedCapture() as capture:
                self.simulate()
            self.graph = capture.graph
        else:
            self.graph = None

    def simulate(self):
        for substep in range(self.sim_substeps):
            self.state_0.clear_forces()
            self.viewer.apply_forces(self.state_0)

            self.plant.pre_step(self.state_0, self.state_1, substep)
            self.solver.set_rigid_history_update(substep % self.plant.collision_interval == 0)
            self.solver.step(
                self.state_0, self.state_1, self.control, self.plant.contacts, self.sim_dt
            )
            self.state_0, self.state_1 = self.state_1, self.state_0

    def step(self):
        if self.graph:
            wp.capture_launch(self.graph)
        else:
            self.simulate()
        self.sim_time += self.frame_dt

    def render(self):
        self.viewer.begin_frame(self.sim_time)
        self.viewer.log_state(self.state_0)
        self.viewer.log_contacts(self.plant.contacts, self.state_0)
        self.plant.render(self.viewer, self.state_0)
        self.viewer.end_frame()


if __name__ == "__main__":
    viewer, args = newton.examples.init()
    example = Example(viewer, args)
    newton.examples.run(example, args)
