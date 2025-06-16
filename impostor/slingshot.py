from impostor.rigid_transformation import RigidTransformation
from impostor.mujoco import save_mujoco_model
from impostor.mesh import Mesh, Bone, compute_vertex_weights
from scipy.spatial.transform._rotation import Rotation
from scipy.spatial.transform import Slerp
import numpy as np
import mujoco
import mujoco.viewer

from impostor.utils import Curve


def create_stem_mesh(length=1.0, radius=0.01, rows=10, columns=6, bone_count=5) -> Mesh:
    vertices = []
    faces = []

    # Create vertices for the cylinder sides
    for row in range(rows + 1):
        z = (row / rows) * length
        for col in range(columns):
            angle = (col / columns) * 2 * np.pi
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            vertices.append([x, y, z])

    # Add center vertex for top cap
    center_idx = len(vertices)
    vertices.append([0, 0, length])

    # Create faces for cylinder sides
    for row in range(rows):
        for col in range(columns):
            # Current row indices
            curr_base = row * columns
            next_base = (row + 1) * columns

            v1 = curr_base + col
            v2 = curr_base + (col + 1) % columns
            v3 = next_base + col
            v4 = next_base + (col + 1) % columns

            # Two triangles per quad
            faces.append([v1, v2, v3])
            faces.append([v2, v4, v3])

    # Create top cap faces
    top_base = rows * columns
    for col in range(columns):
        v1 = top_base + col
        v2 = top_base + (col + 1) % columns
        faces.append([center_idx, v1, v2])

    vertices = np.array(vertices)
    faces = np.array(faces, dtype=int)

    # Create simple bones along the stem
    bones = []
    for i in range(bone_count):
        z_pos = (i / (bone_count - 1)) * length
        bone = Bone(
            body_name=f"bone_{i}",
            bind_position=np.array([0, 0, z_pos]),
            bind_quaternion=np.array([1, 0, 0, 0]),  # Identity quaternion
            vertex_indices=[],
            vertex_weights=[],
        )
        bones.append(bone)

    mesh = Mesh(
        vertices=vertices,
        faces=faces,
        normals=np.zeros((len(vertices), 3)),
        uvs=np.zeros((len(vertices), 2)),
        bones=bones,
    )
    compute_vertex_weights(mesh, max_bones_per_vertex=4)
    return mesh


def test_grow():
    length = 0.8
    bone_count = 15
    xml_path = f"plant_model.xml"
    mesh = create_stem_mesh(
        length=length, radius=0.01, rows=170, columns=12, bone_count=bone_count
    )
    save_mujoco_model(mesh, xml_path, model_name="plant_model")

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    print(f"Model has {model.nbody} bodies")
    print(f"Model has {model.nmocap} mocap bodies")

    # Store original mocap positions and quaternions
    original_mocap_pos = data.mocap_pos.copy()
    original_mocap_quat = data.mocap_quat.copy()

    print("Original mocap positions:", original_mocap_pos)
    print("Original mocap quaternions:", original_mocap_quat)

    pitch_fn = Curve([(0, 1.6), (0.5, 0.8), (1, -3.0)])

    with mujoco.viewer.launch_passive(model, data) as viewer:

        natural_bone_vectors = []
        bone_length = length / (bone_count - 1)
        for i in range(bone_count):
            prev_vector = np.array([0, 0, 1]) if i == 0 else natural_bone_vectors[i - 1]
            p0 = (i - 1) / (bone_count - 1)
            p1 = i / (bone_count - 1)
            pitch = pitch_fn.integrate(p0, p1)
            bending_vector = Rotation.from_euler("y", pitch).apply(prev_vector)
            natural_bone_vectors.append(bending_vector)

        bone_transforms = [None] * bone_count

        while viewer.is_running():
            # Step the simulation
            mujoco.mj_step(model, data)
            viewer.sync()

            # Find the body called "draggable" and copy its position into the collider entity
            draggable_body_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, "draggable"
            )
            collider_translation = data.xpos[draggable_body_id].copy()
            collider_radius = 0.015 * 2.2

            
            for i, vector in enumerate(natural_bone_vectors):
                parent_transform = bone_transforms[i - 1] if i > 0 else None
                if parent_transform is None:
                    parent_translation = np.array([0, 0, 0])
                else:
                    parent_translation = parent_transform.translation

                translation = parent_translation + vector * bone_length
                if bone_transforms[i] is not None:
                    old_translation = bone_transforms[i].translation
                    translation = (
                        old_translation + (translation - old_translation) * 0.1
                    )

                
                rotation = Rotation.from_rotvec(
                    np.cross([0, 0, 1], vector)
                )
                transform = RigidTransformation(
                    translation=translation,
                    rotation=rotation
                )
                bone_transforms[i] = transform

            # Handle collisions
            for i, transform in enumerate(bone_transforms):
                if np.linalg.norm(transform.translation - collider_translation) < collider_radius:
                    # Adjust the translation to avoid collision
                    direction = transform.translation - collider_translation
                    direction /= np.linalg.norm(direction)
                    transform.translation = (
                        collider_translation + direction * collider_radius
                    )
                    # Adjust the transforms using FABRIK
                    fabrik_adjustment(bone_transforms, bone_length, i)

            for index, bone in enumerate(mesh.bones):
                body_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_BODY, bone.body_name
                )
                mocap_id = model.body_mocapid[body_id]
                transform = bone_transforms[index]
                data.mocap_pos[mocap_id] = transform.translation.copy()
                data.mocap_quat[mocap_id] = transform.rotation.as_quat(
                    scalar_first=True
                ).copy()
            # break


def fabrik_adjustment(transforms, bone_length, start_back_from):
    """
    Adjust the transforms using FABRIK algorithm.
    """
    n = len(transforms)
    if n == 0:
        return transforms

    # Save different between the rotation of the last bone and the second last
    last_rotation_diff = (
        transforms[-1].rotation.as_rotvec() - transforms[-2].rotation.as_rotvec()
    )

    # Backward pass
    for i in range(start_back_from, 1, -1):
        direction = transforms[i + 1].translation - transforms[i].translation
        length = np.linalg.norm(direction)
        if length > 0:
            direction /= length
            transforms[i].translation = (
                transforms[i + 1].translation - direction * bone_length
            )

    # Foward pass
    for i in range(1, n):
        direction = transforms[i].translation - transforms[i - 1].translation
        length = np.linalg.norm(direction)
        if length > 0:
            direction /= length
            transforms[i].translation = (
                transforms[i - 1].translation + direction * bone_length
            )

    # Update rotations to point towards the next bone
    for i in range(n - 1):
        direction = transforms[i + 1].translation - transforms[i].translation
        if np.linalg.norm(direction) > 0:
            direction /= np.linalg.norm(direction)
            transforms[i].rotation = Rotation.from_rotvec(
                np.cross([0, 0, 1], direction)
            )
    # # Make the last bone point the same direction as the second last plus the original difference
    # transforms[-1].rotation = Rotation.from_rotvec(
    #     transforms[-2].rotation.as_rotvec() + last_rotation_diff
    # )


if __name__ == "__main__":
    test_grow()
