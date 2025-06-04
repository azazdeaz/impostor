import xml.etree.ElementTree as ET
from typing import Dict
from xml.dom import minidom

from .mesh import PlantMesh


def create_mujoco_model(plant_mesh: PlantMesh, model_name: str = "plant_model") -> str:
    """Create a MuJoCo XML model from a plant."""

    # Create the root XML element
    mujoco = ET.Element("mujoco", model=model_name)

    # Add compiler settings
    _compiler = ET.SubElement(mujoco, "compiler", angle="radian")

    # Add default settings
    default = ET.SubElement(mujoco, "default")
    _default_geom = ET.SubElement(default, "geom", rgba="0.8 0.6 0.4 1")

    # Add assets section
    assets = ET.SubElement(mujoco, "asset")

    # Add material for the plant
    _material = ET.SubElement(
        assets, "material", name="plant_material", rgba="0.2 0.8 0.2 1"
    )

    # Add worldbody
    worldbody = ET.SubElement(mujoco, "worldbody")

    # Add light
    _light = ET.SubElement(
        worldbody, "light", diffuse=".5 .5 .5", pos="0 0 3", dir="0 0 -1"
    )

    # Add ground plane
    _geom_ground = ET.SubElement(
        worldbody,
        "geom",
        name="ground",
        type="plane",
        size="2 2 0.1",
        rgba="0.9 0.9 0.9 1",
    )

    # Filter empty bones
    bones = {e: b for e, b in plant_mesh.bones.items() if b.vertex_indices}

    for entity, bone in bones.items():
        body_name = f"body_{entity}"

        # Create mocap body as direct child of world
        body = ET.SubElement(
            worldbody,
            "body",
            name=body_name,
            pos=f"{bone.bind_position[0]} {bone.bind_position[1]} {bone.bind_position[2]}",
            mocap="true",
        )

        # # Add sphere collider using bone radius
        # ET.SubElement(
        #     body,
        #     "geom",
        #     name=f"geom_{entity}",
        #     type="sphere",
        #     size=str(bone.radius),
        #     rgba="0.2 0.8 0.2 0.3",  # Semi-transparent green
        #     mass="0.1",
        # )

    # Add draggable body to the world
    draggable_body = ET.SubElement(
        worldbody, "body", name="draggable", pos="0.05 0 0.09", gravcomp="1"
    )

    ET.SubElement(
        draggable_body,
        "geom",
        name="sphere_geom",
        type="sphere",
        size="0.015",
        rgba="0.2 0.2 0.8 0.4",  # Blue sphere
        mass="0.001",
        solimp="0.9 0.95 0.001",  # Soft contact parameters
        solref="0.02 1",
    )

    # Add joint to make it free-floating (6DOF)
    ET.SubElement(
        draggable_body, "joint", name="cube_joint", type="free", damping="0.05"
    )

    # Add deformable section with skin element
    deformable = ET.SubElement(mujoco, "deformable")
    skin = ET.SubElement(
        deformable,
        "skin",
        name="plant_skin",
        material="plant_material",
        inflate="0.001",
    )

    # Add vertex data
    vertex_data = " ".join([f"{v[0]} {v[1]} {v[2]}" for v in plant_mesh.vertices])
    skin.set("vertex", vertex_data)

    # Add face data
    face_data = " ".join([f"{f[0]} {f[1]} {f[2]}" for f in plant_mesh.faces])
    skin.set("face", face_data)

    # Add texture coordinates if available
    if plant_mesh.uvs.size > 0:
        texcoord_data = " ".join([f"{uv[0]} {uv[1]}" for uv in plant_mesh.uvs])
        skin.set("texcoord", texcoord_data)

    # Add bone elements
    for entity, bone in bones.items():
        bone_elem = ET.SubElement(
            skin,
            "bone",
            body=f"body_{entity}",
            bindpos=f"{bone.bind_position[0]} {bone.bind_position[1]} {bone.bind_position[2]}",
            bindquat=f"{bone.bind_quaternion[0]} {bone.bind_quaternion[1]} {bone.bind_quaternion[2]} {bone.bind_quaternion[3]}",
        )

        # Add vertex indices and weights
        vertid_data = " ".join([str(idx) for idx in bone.vertex_indices])
        bone_elem.set("vertid", vertid_data)

        vertweight_data = " ".join([str(w) for w in bone.vertex_weights])
        bone_elem.set("vertweight", vertweight_data)

    # Convert to pretty-printed string
    rough_string = ET.tostring(mujoco, "unicode")
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def save_mujoco_model(
    plant_mesh: PlantMesh, filename: str, model_name: str = "plant_model"
):
    """Save a MuJoCo model to file."""
    xml_content = create_mujoco_model(plant_mesh, model_name)
    with open(filename, "w") as f:
        f.write(xml_content)
