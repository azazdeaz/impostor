"""SO101 robot arm interacting with a simulated strawberry plant.

Demonstrates the PlantSimulation API with a single VBD solver that handles
both the robot arm (revolute joints with position drives) and the plant
(cable joints for rods + cloth for leaves).
The arm follows a looping trajectory that brushes through the plant.

Usage:
  python strawberry_newton_soarm.py                              # plant only
  python strawberry_newton_soarm.py --soarm                      # plant + arm (trajectory)
  python strawberry_newton_soarm.py --soarm --controller keyboard        # keyboard teleop (1-6 / Ctrl+1-6)
  python strawberry_newton_soarm.py --soarm --controller so_leader --port /dev/ttyACM0  # SO-101 leader arm
"""

import numpy as np
import warp as wp
from huggingface_hub import hf_hub_download

import newton
import newton.examples
from newton.solvers import SolverVBD

from impostor_gen.newton_builder import PlantSimulation
from soarm_control import add_cli_args, create_controller
from strawberry_plant import grow_strawberry_plant


# ── Asset download ──────────────────────────────────────────────────────
REPO_ID = "azazdeaz/impostor_demo_assets"


def download_so101() -> str:
    """Download SO101 USD files and return path to the main .usda."""
    usda = hf_hub_download(repo_id=REPO_ID, filename="so101_follower_newton.usda")
    # The .usda references this .usd via relative path; both must be cached together
    hf_hub_download(repo_id=REPO_ID, filename="so101_follower_leisaac.usd")
    return usda


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

        # ── Plant ──
        self.plant = PlantSimulation(
            grow_strawberry_plant(iterations=50),
            position=(0.0, 0.0, 0.0),
            # [grow_strawberry_plant(iterations=50) for _ in range(6)],
            # position=[(0.0, 0.0, i * 0.2) for i in range(6)],
            include_stems=True,
            include_midrib=True,
            include_veins=True,
            include_cloth=True,
            rod_radius=0.5,
            bend_stiffness_modulus=1.0e11,
            stretch_stiffness_modulus=1.0e9,
            bend_damping_modulus=1.0e8,
            stretch_damping_modulus=1.0e0,
        )

        # ── Assemble scene ──
        builder = newton.ModelBuilder()

        # ── Robot or draggable sphere ──
        if self.use_robot:
            robot_builder = newton.ModelBuilder()
            if self.use_robot:
                asset_file = download_so101()
                robot_builder.add_usd(
                    asset_file,
                    xform=wp.transform(wp.vec3(0.0, 0.28, -0.03)),
                    collapse_fixed_joints=False,
                    enable_self_collisions=False,
                    hide_collision_shapes=True,
                    floating=False,
                )
                # Position drives for VBD (kd is Rayleigh: D = kd * ke)
                for i in range(len(robot_builder.joint_target_ke)):
                    robot_builder.joint_target_ke[i] = 500.0
                    robot_builder.joint_target_kd[i] = 0.1
            builder.add_builder(robot_builder)
        else:
            self.sphere_pos = wp.vec3(0.2, -0.225, 0.04)
            body_sphere = builder.add_body(
                xform=wp.transform(p=self.sphere_pos), label="sphere"
            )
            builder.add_shape_sphere(body_sphere, radius=0.03)

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
            newton.eval_fk(
                self.model, self.model.joint_q, self.model.joint_qd, self.state_0
            )

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
                self.state_0,
                self.state_1,
                self.control,
                self.plant.contacts,
                self.sim_dt,
            )

            self.state_0, self.state_1 = self.state_1, self.state_0

    def step(self):
        if self.use_robot:
            # Write new targets into the pre-allocated staging buffer,
            # then copy into the control array (same pointer — graph-compatible)
            target_q = self.controller.get_action(self.frame_dt)
            self._target_pos_np[: self.robot_dof_count] = target_q
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
    parser.add_argument(
        "--soarm", action="store_true", help="Add SO101 robot arm to the scene."
    )
    add_cli_args(parser)
    viewer, args = newton.examples.init(parser)
    example = Example(viewer, args)
    newton.examples.run(example, args)
