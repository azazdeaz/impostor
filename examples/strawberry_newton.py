from pathlib import Path

import numpy as np
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
from impostor_gen.material import Material
from impostor_gen.mesh_builder import generate_blueprint
from impostor_gen.newton_builder import (
    bind_particles_to_bodies,
    build_newton_model,
    compute_cloth_local_offsets,
)

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
            pitch = -50 + age // crown.shoot_period * 6.0
            writer.write(
                [
                    crown,
                    BranchOpen(),
                    MaterialKey(key="stem"),
                    Diameter(diameter=0.41),
                    F(length=0.0),
                    Roll(angle=roll),
                    Pitch(angle=pitch),
                    StemContext(
                        target_length=5.2 + age * 0.02,
                        growth_speed=0.2,
                        section_length=0.8,
                        diameter_start=0.4,
                        diameter_end=0.25,
                    ),
                    StemTip(),
                    *create_trifoliate_leaf(leaf_material),
                    BranchClose(),
                ]
            )


class XY(Symbol):
    x: float = 0.0
    y: float = 0.0


# ── Generate plant blueprints via L-system ──────────────────────────────
def grow_plant(iterations: int = 50):
    lsystem = LSystem(
        world=[Crown(), XY()],
        rules=[InterpolateRule(), StemGrowthRule(), AgeingRule(), IterateCrown()],
    )
    for _ in range(iterations):
        lsystem.iterate()
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

        # Grow the plant and build as rod/cable structure
        blueprints = grow_plant(iterations=50)
        result = build_newton_model(
            blueprints,
            include_stems=True,
            include_midrib=True,
            include_veins=True,
            include_cloth=True,
            rod_radius=0.2,
            bend_stiffness=1.0e5,
            bend_damping=1.0e-1,
        )
        builder = result.builder
        self.cloth_bindings = result.cloth_bindings
        self.num_bindings = len(self.cloth_bindings.bind_particle_ids)

        # Ground plane
        builder.add_ground_plane(
            cfg=newton.ModelBuilder.ShapeConfig(ke=1e6, kd=1e1, mu=0.5)
        )

        # SPHERE
        self.sphere_pos = wp.vec3(0.0, -0.5, 0.8)
        body_sphere = builder.add_body(xform=wp.transform(p=self.sphere_pos, q=wp.quat_identity()), label="sphere")
        builder.add_shape_sphere(body_sphere, radius=0.85)

        builder.color(include_bending=True)

        # Finalize
        self.model = builder.finalize()
        self.model.soft_contact_ke = 1e4
        self.model.soft_contact_kd = 1e-4
        self.model.soft_contact_mu = 0.5

        self.solver = newton.solvers.SolverVBD(
            self.model,
            iterations=30,
            friction_epsilon=0.1,
            rigid_enable_dahl_friction=False,
        )
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()

        # Compute cloth particle → body local offsets
        if self.num_bindings > 0:
            device = self.solver.device
            self.bind_body_ids_wp, self.bind_particle_ids_wp, self.bind_local_offsets_wp = (
                compute_cloth_local_offsets(self.state_0, self.cloth_bindings, device=device)
            )

        self.collision_pipeline = newton.CollisionPipeline(
            self.model,
            broad_phase="nxn",
            soft_contact_margin=0.005,
        )
        self.contacts = self.collision_pipeline.contacts()

        self.viewer.set_model(self.model)
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

            # Bind cloth particles to rod body transforms
            if self.num_bindings > 0:
                wp.launch(
                    kernel=bind_particles_to_bodies,
                    dim=self.num_bindings,
                    inputs=[
                        self.state_0.body_q,
                        self.bind_body_ids_wp,
                        self.bind_particle_ids_wp,
                        self.bind_local_offsets_wp,
                    ],
                    outputs=[
                        self.state_0.particle_q,
                        self.state_1.particle_q,
                    ],
                )

            if substep % 16 == 0:
                self.collision_pipeline.collide(self.state_0, self.contacts)
            self.solver.set_rigid_history_update(substep % 16 == 0)
            self.solver.step(
                self.state_0, self.state_1, self.control, self.contacts, self.sim_dt
            )
            self.state_0, self.state_1 = self.state_1, self.state_0

    def step(self):
        if self.graph:
            wp.capture_launch(self.graph)
        else:
            self.simulate()
        num_rigid = self.contacts.rigid_contact_count.numpy()[0]
        print(f"Collisions: {num_rigid} rigid contacts")
        self.sim_time += self.frame_dt

    def render(self):
        self.viewer.begin_frame(self.sim_time)
        self.viewer.log_state(self.state_0)
        self.viewer.log_contacts(self.contacts, self.state_0)
        self.viewer.end_frame()


if __name__ == "__main__":
    viewer, args = newton.examples.init()
    example = Example(viewer, args)
    newton.examples.run(example, args)
