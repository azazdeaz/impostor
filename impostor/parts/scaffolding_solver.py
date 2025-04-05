from dataclasses import dataclass
from typing import Dict, List, Tuple
from impostor.parts import BasePart, Entity, Stick, ScaffoldingLayer, Position
from impostor.plant import Plant
from impostor import parts
import torch
import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation

class ScaffoldingSolver(BasePart, rr.AsComponents):
    def __init__(self):
        self._layers: Dict[Entity, ScaffoldingLayer] = {}
        self._num_positions = 0
        self.sticks: List[Stick] = []
        self.r1 = 0.23
        self.num_segments = 5
        self._current_positions = torch.tensor([])
        # Positions closer that this tresold to their ideal position will jump to the ideal position before the optimization starts
        self.snap_threshold = 0.01

        # Optimization parameters
        self.optimization_steps = 200
        self.learning_rate = 0.001
        self.convergence_threshold = 1e-12

        # Parameter tensor and optimizer (will be initialized when needed)
        self._position_param = None
        self._optimizer = None

    def step(self, plant: Plant, _entity: Entity):
        roots = (
            plant.query()
            .with_component(parts.Vascular)
            .without_component(parts.AxePrev)
            .filter(
                lambda comps: comps.get_by_type(parts.Vascular).type
                == parts.VascularType.STEM
            )
            .entities()
        )

        # Compute where all the transforms should be for each entity
        # This will be used to compute the target lengths for the sticks
        ideal_transforms: Dict[Entity, parts.RigidTransformation] = {}

        for root in roots:
            buffer = [root]

            # Get (or initialize) the transform for the root
            root_components = plant.get_components(root)
            if parts.RigidTransformation in root_components:
                ideal_transforms[root] = root_components.get_by_type(
                    parts.RigidTransformation
                )
            else:
                ideal_transforms[root] = parts.RigidTransformation()

            while buffer:
                entity = buffer.pop(0)
                components = plant.get_components(entity)
                vascular = components.get_by_type(parts.Vascular)
                next = components.get_by_type(parts.AxeNext)

                if entity not in self._layers:
                    self.add_layer(
                        entity,
                        ideal_transforms[entity],
                        vascular.radius if vascular else 0.0,
                    )
                    prev = components.get_by_type(parts.AxePrev)
                    if prev is not None:
                        self.connect_layers(prev.prev, entity)
                else:
                    self._layers[entity].update_positions(
                        ideal_transforms[entity], vascular.radius
                    )

                if next is not None:
                    # Compute the ideal transform for the next entity
                    transform = parts.RigidTransformation.from_rotation(
                        vascular.rotation
                    ).combine(
                        parts.RigidTransformation.from_z_translation(vascular.length)
                    )
                    transform = ideal_transforms[entity].combine(transform)

                    # Add this next entity to ideal_transforms and the buffer
                    next_entity = next.next
                    ideal_transforms[next_entity] = transform

                    buffer.append(next_entity)

        if len(self._layers) == 0 or len(self.sticks) == 0:
            return

        # Collect all positions as a single tensor
        ideal_positions = torch.zeros(self._num_positions, 3)
        for layer in self._layers.values():
            ideal_positions[layer.indices] = layer.get_positions()

        # Lock the poistions in the layers of the root entities
        locked_position_indices = []
        for root in roots:
            locked_position_indices += self._layers[root].get_global_indices()
        locked_position_indices = torch.tensor(locked_position_indices)
        locked_positions = ideal_positions[locked_position_indices]

        # If ideal_positions are longer, add the new positions to the current positions.
        if ideal_positions.shape[0] > self._current_positions.shape[0]:
            # Initialize or extend current positions
            if self._current_positions.shape[0] == 0:
                self._current_positions = ideal_positions.clone()
            else:
                self._current_positions = torch.cat(
                    [
                        self._current_positions,
                        ideal_positions[self._current_positions.shape[0] :],
                    ],
                    dim=0,
                )

        # Collect stick indices as a tensor
        stick_indices = torch.tensor([[stick.a, stick.b] for stick in self.sticks])

        # If average length lower than treshold, snap the positions to the ideal positions
        average_stick_length = torch.mean(
            torch.norm(
                ideal_positions[stick_indices[:, 0]]
                - ideal_positions[stick_indices[:, 1]],
                dim=1,
            )
        )
        print(f"Average stick length: {average_stick_length}")
        if average_stick_length < 0.02:
            self._current_positions = ideal_positions.clone()

        # if self.snap_threshold > 0.0:
        #     # Snap positions closer than the threshold to their ideal positions
        #     snap_mask = (
        #         torch.norm(ideal_positions - self._current_positions, dim=1)
        #         < self.snap_threshold
        #     )
        #     self._current_positions[snap_mask] = ideal_positions[snap_mask]

        # Create new parameter and optimizer
        self._position_param = torch.nn.Parameter(self._current_positions.clone())
        self._optimizer = torch.optim.Adam(
            [self._position_param], lr=self.learning_rate
        )

        # Run optimization loop
        prev_loss = float("inf")
        for step in range(self.optimization_steps):
            # Zero gradients before computing new ones
            self._optimizer.zero_grad()

            # Compute the stick target lengths from the ideal positions
            stick_target_lengths = torch.norm(
                ideal_positions[stick_indices[:, 0]]
                - ideal_positions[stick_indices[:, 1]],
                dim=1,
            )

            # Compute current lengths from the optimized parameter positions
            stick_current_lengths = torch.norm(
                self._position_param[stick_indices[:, 0]]
                - self._position_param[stick_indices[:, 1]],
                dim=1,
            )

            # Compute error and loss
            stick_errors = stick_target_lengths - stick_current_lengths
            stick_loss = torch.sum(stick_errors**2)

            lock_errors = (
                self._position_param[locked_position_indices] - locked_positions
            )
            lock_loss = torch.sum(lock_errors**2)

            loss = stick_loss + lock_loss

            # Backpropagate and optimize
            loss.backward()
            self._optimizer.step()

            # Check for convergence
            current_loss = loss.item()
            if abs(prev_loss - current_loss) < self.convergence_threshold:
                break

            prev_loss = current_loss

        # Update the current positions with optimized values (detached from graph)
        self._current_positions = self._position_param.detach().clone()

        # Visualization (use detached tensors to avoid computation graph issues)
        rr.log(
            "scaffolding/sticks",
            rr.LineStrips3D(
                self._current_positions[stick_indices].numpy(),
                radii=0.00005,
            ),
        )
        rr.log(
            "scaffolding/positions",
            rr.Points3D(self._current_positions.numpy(), radii=0.0005),
        )
        rr.log("scaffolding/stick_loss", rr.Scalar(prev_loss))

        # Also visualize ideal positions for comparison
        rr.log(
            "scaffolding/ideal_sticks",
            rr.LineStrips3D(
                ideal_positions[stick_indices].numpy(),
                radii=0.00005,
                colors=(0.2, 0.8, 0.2, 0.5),  # Green, semi-transparent
            ),
        )
        rr.log(
            "scaffolding/ideal_positions",
            rr.Points3D(
                ideal_positions.numpy(), radii=0.0004, colors=(0.2, 0.8, 0.2, 0.7)
            ),
        )

        # Update the transforms in the plant
        for entity in ideal_transforms.keys():
            layer = self._layers[entity]

            plant.add_components(
                entity,
                layer.to_transform(self._current_positions),
            )

    def get_flat_layer_positions(self) -> torch.Tensor:
        """Return all positions in the layers as a single (n,3) tensor"""
        return torch.cat([layer.get_positions() for layer in self._layers.values()])

    def add_layer(
        self, entity: Entity, transform: parts.RigidTransformation, radius: float
    ):
        """Add a new scaffolding layer to the solver"""
        layer = ScaffoldingLayer(
            r1_scale=2.5,
            n1=self.num_segments,
            global_index_start=self._num_positions,
        )
        layer.update_positions(transform, radius)
        self._num_positions += len(layer.indices)
        self._layers[entity] = layer

        # Connect ring positions to center
        for i in range(layer.n1):
            self.sticks.append(
                Stick(
                    layer.global_idx(0),
                    layer.global_idx(i + 1),
                )
            )
        # Connect ring positions to each other
        for i in range(layer.n1):
            self.sticks.append(
                Stick(
                    layer.global_idx(i + 1),
                    layer.global_idx((i + 1) % layer.n1 + 1),
                )
            )

    def connect_layers(self, lower: Entity, upper: Entity):
        """
        Connect two scaffolding layers together with structural supports.

        Args:
            lower: Entity representing the lower scaffolding layer
            upper: Entity representing the upper scaffolding layer
        """
        # Get the layers
        lower_layer = self._layers[lower]
        upper_layer = self._layers[upper]

        # Connect the center points of both layers
        self.sticks.append(
            Stick(
                lower_layer.global_idx(0),  # Center of lower layer
                upper_layer.global_idx(0),  # Center of upper layer
            )
        )

        # Get the number of points in each ring
        n_lower = lower_layer.n1
        n_upper = upper_layer.n1

        # Connect each point in the lower ring to corresponding points in the upper ring
        for i in range(n_lower):
            # Find the corresponding point in the upper ring by relative angular position
            upper_idx = int(i * n_upper / n_lower) % n_upper

            # Connect to the corresponding point (straight connection)
            self.sticks.append(
                Stick(
                    lower_layer.global_idx(
                        i + 1
                    ),  # Point i in lower ring (1-based index)
                    upper_layer.global_idx(
                        upper_idx + 1
                    ),  # Corresponding point in upper ring
                )
            )

            # Add diagonal cross connection for additional stability
            next_upper_idx = (upper_idx + 1) % n_upper
            self.sticks.append(
                Stick(
                    lower_layer.global_idx(i + 1),  # Point i in lower ring
                    upper_layer.global_idx(
                        next_upper_idx + 1
                    ),  # Next point in upper ring
                )
            )

            # Create triangular connections between centers and rings

            # Triangle 1: lower center -> lower ring -> upper ring
            self.sticks.append(
                Stick(
                    lower_layer.global_idx(0),  # Center of lower layer
                    upper_layer.global_idx(upper_idx + 1),  # Point on upper ring
                )
            )

            # Triangle 2: upper center -> upper ring -> lower ring
            self.sticks.append(
                Stick(
                    upper_layer.global_idx(0),  # Center of upper layer
                    lower_layer.global_idx(i + 1),  # Point on lower ring
                )
            )

    def reset_optimization(self):
        """Reset the optimization state, forcing a fresh optimization on next step"""
        self._position_param = None
        self._optimizer = None
        self._current_positions = torch.tensor([])

    def as_component_batches(self) -> List[rr.ComponentBatchLike]:
        return [
            rr.AnyBatchValue("ScaffoldingSolver.num_layers", len(self._layers)),
            rr.AnyBatchValue("ScaffoldingSolver.num_sticks", len(self.sticks)),
        ]
