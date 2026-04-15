"""SO101 robot arm interacting with a simulated strawberry plant.

Demonstrates the PlantSimulation API with a single VBD solver that handles
both the robot arm (revolute joints with position drives) and the plant
(cable joints for rods + cloth for leaves).
The arm follows a looping trajectory that brushes through the plant.

Usage:
  python strawberry_newton_soarm.py           # plant only
  python strawberry_newton_soarm.py --soarm   # plant + SO101 arm
"""

import numpy as np
import warp as wp
from huggingface_hub import hf_hub_download
from pydantic import BaseModel

import newton
import newton.examples
from newton.solvers import SolverVBD

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
from impostor_gen.newton_builder import PlantSimulation
from soarm_control import add_cli_args, create_controller


# ── Asset download ──────────────────────────────────────────────────────
REPO_ID = "azazdeaz/impostor_demo_assets"


def download_so101() -> str:
    """Download SO101 USD files and return path to the main .usda."""
    usda = hf_hub_download(repo_id=REPO_ID, filename="so101_follower_newton.usda")
    # The .usda references this .usd via relative path; both must be cached together
    hf_hub_download(repo_id=REPO_ID, filename="so101_follower_leisaac.usd")
    return usda


# ── Plant growth (same L-system as strawberry_newton.py) ────────────────
leaf_material = Material(key="leaf", diffuse_color=(0.2, 0.6, 0.1))


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
                Diameter(diameter=0.0065),
                F(length=0.0),
                Roll(angle=roll),
                Pitch(angle=pitch),
                StemContext(
                    target_length=0.13 + age * 0.0005,
                    growth_speed=0.01,
                    section_length=0.02,
                    diameter_start=0.0065,
                    diameter_end=0.0042,
                ),
                StemTip(),
                *create_trifoliate_leaf(leaf_material, size_scale=0.025),
                BranchClose(),
            ])


def grow_plant(iterations: int = 50):
    lsystem = LSystem(
        world=[Crown()],
        rules=[InterpolateRule(), StemGrowthRule(), AgeingRule(), IterateCrown()],
    )
    for _ in range(iterations):
        lsystem.iterate()
    return generate_blueprint(lsystem.world)


# ── Example ─────────────────────────────────────────────────────────────
class Example:
    def __init__(self, viewer, args=None):
        self.fps = 30
        self.frame_dt = 1.0 / self.fps
        self.sim_time = 0.0
        self.sim_substeps = 20
        self.sim_dt = self.frame_dt / self.sim_substeps

        self.viewer = viewer
        self.use_robot = getattr(args, "soarm", False)

        # ── Robot (optional) ──
        robot_builder = newton.ModelBuilder()
        if self.use_robot:
            asset_file = download_so101()
            robot_builder.add_usd(
                asset_file,
                xform=wp.transform(wp.vec3(0.12, 0.5, 0.0)),
                collapse_fixed_joints=False,
                enable_self_collisions=False,
                hide_collision_shapes=True,
                floating=False,
            )
            # Position drives for VBD (kd is Rayleigh: D = kd * ke)
            for i in range(len(robot_builder.joint_target_ke)):
                robot_builder.joint_target_ke[i] = 500.0
                robot_builder.joint_target_kd[i] = 0.1

        # ── Plant ──
        self.plant = PlantSimulation(
            grow_plant(iterations=50),
            position=(0.0, 0.0, 0.0),
            # [grow_plant(iterations=50), grow_plant(iterations=50), grow_plant(iterations=50), grow_plant(iterations=50), grow_plant(iterations=50), grow_plant(iterations=50)],
            # position=[(0.0, 0.0, 0.0), (0.0, 0.0, 0.2), (0.0, 0.0, 0.4), (0.0, 0.0, 0.6), (0.0, 0.0, 0.8), (0.0, 0.0, 1.0)],
            include_stems=True,
            include_midrib=True,
            include_veins=True,
            include_cloth=False,
            rod_radius=0.5,
            bend_stiffness_modulus=1.0e9,
            stretch_stiffness_modulus=1.0e7,
            bend_damping_modulus=1.0e6,
            stretch_damping_modulus=1.0e1,
        )

        # ── Assemble scene ──
        builder = newton.ModelBuilder()
        builder.add_builder(robot_builder)
        self.robot_dof_count = builder.joint_dof_count

        self.plant.add_to_builder(builder)
        builder.add_ground_plane(
            cfg=newton.ModelBuilder.ShapeConfig(ke=1e6, kd=1e1, mu=0.5)
        )
        builder.color(include_bending=True)

        # ── Finalize ──
        self.model = builder.finalize()
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()

        self.plant.initialize(self.model, self.state_0)

        # ── Solver ──
        self.solver = SolverVBD(
            self.model,
            iterations=30,
            friction_epsilon=0.1,
            rigid_enable_dahl_friction=False,
        )

        if self.use_robot:
            # FK for initial body transforms
            newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state_0)

            # ── Controller ──
            self.controller = create_controller(args, self.robot_dof_count)
            self.controller.connect()

            # Pre-allocate staging buffer for graph-compatible target updates
            self._target_pos_np = np.zeros(self.model.joint_dof_count, dtype=np.float32)
            self._target_pos_wp = wp.array(self._target_pos_np, dtype=wp.float32)

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
            self.solver.set_rigid_history_update(
                substep % self.plant.collision_interval == 0
            )
            self.solver.step(
                self.state_0, self.state_1, self.control, self.plant.contacts, self.sim_dt,
            )

            self.state_0, self.state_1 = self.state_1, self.state_0

    def step(self):
        if self.use_robot:
            # Write new targets into the pre-allocated staging buffer,
            # then copy into the control array (same pointer — graph-compatible)
            target_q = self.controller.get_action(self.frame_dt)
            self._target_pos_np[:self.robot_dof_count] = target_q
            self._target_pos_wp.assign(self._target_pos_np)
            self.control.joint_target_pos.assign(self._target_pos_wp)

        if self.graph:
            wp.capture_launch(self.graph)
        else:
            self.simulate()
        self.sim_time += self.frame_dt

    def render(self):
        self.viewer.begin_frame(self.sim_time)
        self.viewer.log_state(self.state_0)
        if self.plant.contacts is not None:
            self.viewer.log_contacts(self.plant.contacts, self.state_0)
        self.plant.render(self.viewer, self.state_0)
        self.viewer.end_frame()


if __name__ == "__main__":
    parser = newton.examples.create_parser()
    parser.add_argument("--soarm", action="store_true", help="Add SO101 robot arm to the scene.")
    add_cli_args(parser)
    viewer, args = newton.examples.init(parser)
    example = Example(viewer, args)
    newton.examples.run(example, args)
