import xml.etree.ElementTree as ET
from xml.dom import minidom
import numpy as np
from typing import Dict, List, Optional
from .mesh import PlantMesh

def create_mujoco_model(plant_mesh: PlantMesh, model_name: str = "plant_model") -> str:
    """Create a MuJoCo XML model from a plant."""
    
    # Create the root XML element
    mujoco = ET.Element("mujoco", model=model_name)
    
    # Add compiler settings
    compiler = ET.SubElement(mujoco, "compiler", angle="radian")
    
    # Add default settings
    default = ET.SubElement(mujoco, "default")
    default_geom = ET.SubElement(default, "geom", rgba="0.8 0.6 0.4 1")
    
    # Add assets section
    assets = ET.SubElement(mujoco, "asset")
    
    # Add material for the plant
    material = ET.SubElement(assets, "material", 
                           name="plant_material", 
                           rgba="0.2 0.8 0.2 1")
    
    # Add worldbody
    worldbody = ET.SubElement(mujoco, "worldbody")
    
    # Add light
    light = ET.SubElement(worldbody, "light", 
                         diffuse=".5 .5 .5", 
                         pos="0 0 3", 
                         dir="0 0 -1")
    
    # Add ground plane
    geom_ground = ET.SubElement(worldbody, "geom", 
                               name="ground", 
                               type="plane", 
                               size="2 2 0.1", 
                               rgba="0.9 0.9 0.9 1")
    
    # Create flat structure - all mocap bodies as direct children of world
    entity_to_body: Dict[int, str] = {}
    body_counter = 0

    # Filter empty bones
    bones = [bone for bone in plant_mesh.bones if len(bone.vertex_indices) > 0]
    
    for bone in bones:
        entity = plant_mesh.vertex_to_entity.get(bone.vertex_indices[0])
        if entity and entity not in entity_to_body:
            body_name = f"body_{body_counter}"
            entity_to_body[entity] = body_name
            
            # Create mocap body as direct child of world
            body = ET.SubElement(worldbody, "body", 
                               name=body_name,
                               pos=f"{bone.bind_position[0]} {bone.bind_position[1]} {bone.bind_position[2]}",
                               mocap="true")
            
            # Add a small geom for visualization (optional)
            geom = ET.SubElement(body, "geom", 
                               name=f"geom_{body_counter}",
                               type="sphere",
                               size="0.001",
                               rgba="0 0 0 0",  # Transparent
                               mass="0")
            
            body_counter += 1
    
    # Add deformable section with skin element
    if bones:
        deformable = ET.SubElement(mujoco, "deformable")
        skin = ET.SubElement(deformable, "skin", 
                           name="plant_skin",
                           material="plant_material",
                           inflate="0.001")
        
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
        for bone in bones:
            entity = plant_mesh.vertex_to_entity.get(bone.vertex_indices[0])
            if entity and entity in entity_to_body:
                bone_elem = ET.SubElement(skin, "bone",
                                        body=entity_to_body[entity],
                                        bindpos=f"{bone.bind_position[0]} {bone.bind_position[1]} {bone.bind_position[2]}",
                                        bindquat=f"{bone.bind_quaternion[0]} {bone.bind_quaternion[1]} {bone.bind_quaternion[2]} {bone.bind_quaternion[3]}")
                
                # Add vertex indices and weights
                vertid_data = " ".join([str(idx) for idx in bone.vertex_indices])
                bone_elem.set("vertid", vertid_data)
                
                vertweight_data = " ".join([str(w) for w in bone.vertex_weights])
                bone_elem.set("vertweight", vertweight_data)
    
    # Convert to pretty-printed string
    rough_string = ET.tostring(mujoco, 'unicode')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def save_mujoco_model(plant_mesh: PlantMesh, filename: str, model_name: str = "plant_model"):
    """Save a MuJoCo model to file."""
    xml_content = create_mujoco_model(plant_mesh, model_name)
    with open(filename, 'w') as f:
        f.write(xml_content)