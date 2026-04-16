"""Robot arm controllers for the strawberry soarm example.

Three controllers with a shared interface:
  - TrajectoryController: sinusoidal joint trajectory (default)
  - KeyboardController: pynput keyboard teleop (1-6 / Ctrl+1-6)
  - SOLeaderController: lerobot SO-101 leader arm

Each controller implements:
    connect()                    — start the controller
    get_action(dt) -> ndarray    — return joint position targets
    disconnect()                 — release resources

Usage:
    from soarm_control import add_cli_args, create_controller

    add_cli_args(parser)                           # --controller, --port, --step-size
    controller = create_controller(args, num_dofs)
    controller.connect()
    targets = controller.get_action(frame_dt)
    controller.disconnect()
"""

from __future__ import annotations

import numpy as np


# ── Trajectory ──────────────────────────────────────────────────────────
class TrajectoryController:
    """Sinusoidal joint trajectory that sweeps the first 4 joints."""

    def __init__(self, num_joints: int):
        self._t = 0.0
        self.home = np.zeros(num_joints, dtype=np.float32)
        self.amplitudes = np.zeros(num_joints, dtype=np.float32)
        self.frequencies = np.zeros(num_joints, dtype=np.float32)
        self.phases = np.zeros(num_joints, dtype=np.float32)
        # Defaults for the first 4 joints
        _amp = [0.4, 0.3, 0.4, 0.3]
        _freq = [0.3, 0.2, 0.25, 0.15]
        _phase = [0.0, 1.0, 0.5, 1.5]
        n = min(num_joints, len(_amp))
        self.amplitudes[:n] = _amp[:n]
        self.frequencies[:n] = _freq[:n]
        self.phases[:n] = _phase[:n]

    def connect(self):
        pass

    def get_action(self, dt: float) -> np.ndarray:
        self._t += dt
        return self.home + self.amplitudes * np.sin(
            2.0 * np.pi * self.frequencies * self._t + self.phases
        )

    def disconnect(self):
        pass


# ── Keyboard ────────────────────────────────────────────────────────────
class KeyboardController:
    """Keyboard teleop: keys 1-6 increment joints, Ctrl+1-6 decrement."""

    _DIGITS = {"1": 0, "2": 1, "3": 2, "4": 3, "5": 4, "6": 5}

    def __init__(self, num_joints: int, step_size: float = 0.05):
        self._num_joints = num_joints
        self._step_size = step_size
        self._positions = np.zeros(num_joints, dtype=np.float32)
        self._pressed: set[str] = set()
        self._ctrl_held = False
        self._listener = None

    def connect(self):
        from pynput import keyboard as kb

        self._kb = kb
        self._listener = kb.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self._listener.start()

    def _on_press(self, key):
        key = self._listener.canonical(key)
        if key == self._kb.Key.ctrl:
            self._ctrl_held = True
        if hasattr(key, "char") and key.char in self._DIGITS:
            self._pressed.add(key.char)

    def _on_release(self, key):
        key = self._listener.canonical(key)
        if key == self._kb.Key.ctrl:
            self._ctrl_held = False
        if hasattr(key, "char") and key.char in self._DIGITS:
            self._pressed.discard(key.char)

    def get_action(self, dt: float) -> np.ndarray:
        for ch in list(self._pressed):
            idx = self._DIGITS[ch]
            if idx < self._num_joints:
                sign = -1.0 if self._ctrl_held else 1.0
                self._positions[idx] += sign * self._step_size
        return self._positions.copy()

    def disconnect(self):
        if self._listener:
            self._listener.stop()


# ── SO-101 Leader ───────────────────────────────────────────────────────
class SOLeaderController:
    """Teleop via lerobot SO-101 leader arm (reads joint positions over serial)."""

    _MOTORS = [
        "shoulder_pan", "shoulder_lift", "elbow_flex",
        "wrist_flex", "wrist_roll", "gripper",
    ]

    def __init__(self, port: str):
        self._port = port

    def connect(self):
        from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig

        cfg = SOLeaderTeleopConfig(port=self._port, id="so_leader", use_degrees=False)
        self._leader = SOLeader(cfg)
        self._leader.connect()

    def get_action(self, dt: float) -> np.ndarray:
        action = self._leader.get_action()
        return np.array(
            [action[f"{m}.pos"] for m in self._MOTORS], dtype=np.float32,
        )

    def disconnect(self):
        self._leader.disconnect()


# ── CLI helpers ─────────────────────────────────────────────────────────
def add_cli_args(parser) -> None:
    """Add --controller, --port, and --step-size arguments to a parser."""
    parser.add_argument(
        "--controller",
        choices=["trajectory", "keyboard", "so_leader"],
        default="trajectory",
        help="Robot control method (default: trajectory).",
    )
    parser.add_argument(
        "--port", type=str, default=None,
        help="Serial port for so_leader (e.g. /dev/ttyACM0).",
    )
    parser.add_argument(
        "--step-size", type=float, default=0.05,
        help="Joint increment per key-press for keyboard controller.",
    )


def create_controller(args, num_joints: int):
    """Instantiate a controller from parsed CLI arguments."""
    kind = getattr(args, "controller", "trajectory")
    if kind == "keyboard":
        return KeyboardController(num_joints, step_size=getattr(args, "step_size", 0.05))
    if kind == "so_leader":
        port = getattr(args, "port", None)
        if port is None:
            raise ValueError("--port is required for the so_leader controller")
        return SOLeaderController(port)
    return TrajectoryController(num_joints)
