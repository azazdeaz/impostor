from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional
import numpy as np
import xml.etree.ElementTree as ET
from xml.dom import minidom
import math
import mujoco
import mujoco.viewer


class TreeGenerator:
    def create_mujoco_tree(self):
        """Generate a MuJoCo XML tree structure"""
        
        # Create root element
        mujoco_elem = ET.Element("mujoco", model="Cable")

        # Set options
        options = ET.SubElement(mujoco_elem, "option")
        options.set("solver", "CG")
        options.set("noslip_iterations", "10")
        
        # Add compiler
        compiler = ET.SubElement(mujoco_elem, "compiler")
        compiler.set("angle", "radian")
        compiler.set("meshdir", "asset/")
        compiler.set("texturedir", "asset/")
        
        # Add size
        size = ET.SubElement(mujoco_elem, "size")
        size.set("memory", "2M")
        
        # Add visual
        visual = ET.SubElement(mujoco_elem, "visual")
        global_elem = ET.SubElement(visual, "global")
        global_elem.set("elevation", "-30")
        
        map_elem = ET.SubElement(visual, "map")
        map_elem.set("stiffness", "700")
        map_elem.set("fogstart", "1")
        map_elem.set("fogend", "15")
        map_elem.set("zfar", "40")
        map_elem.set("haze", "1")
        map_elem.set("shadowscale", "0.5")
        
        rgba = ET.SubElement(visual, "rgba")
        rgba.set("haze", "0.15 0.25 0.35 1")
        
        # Add statistic
        statistic = ET.SubElement(mujoco_elem, "statistic")
        statistic.set("meansize", "0.05")
        statistic.set("extent", "1")
        statistic.set("center", "0 0 0.3")
        
        # Add extension
        extension = ET.SubElement(mujoco_elem, "extension")
        plugin = ET.SubElement(extension, "plugin")
        plugin.set("plugin", "mujoco.elasticity.cable")
        
        instance = ET.SubElement(plugin, "instance")
        instance.set("name", "composite")
        
        config_twist = ET.SubElement(instance, "config")
        config_twist.set("key", "twist")
        config_twist.set("value", "1e8")
        
        config_bend = ET.SubElement(instance, "config")
        config_bend.set("key", "bend")
        config_bend.set("value", "1e8")
        
        config_vmax = ET.SubElement(instance, "config")
        config_vmax.set("key", "vmax")
        config_vmax.set("value", "110.05")
        
        # Add custom
        custom = ET.SubElement(mujoco_elem, "custom")
        text = ET.SubElement(custom, "text")
        text.set("name", "composite_")
        text.set("data", "rope_")
        
        # Add asset
        asset = ET.SubElement(mujoco_elem, "asset")
        
        # Skybox texture
        texture_skybox = ET.SubElement(asset, "texture")
        texture_skybox.set("type", "skybox")
        texture_skybox.set("builtin", "gradient")
        texture_skybox.set("rgb1", "0.3 0.5 0.7")
        texture_skybox.set("rgb2", "0 0 0")
        texture_skybox.set("width", "512")
        texture_skybox.set("height", "3072")
        
        # Plane texture
        texture_plane = ET.SubElement(asset, "texture")
        texture_plane.set("type", "2d")
        texture_plane.set("name", "texplane")
        texture_plane.set("builtin", "checker")
        texture_plane.set("mark", "cross")
        texture_plane.set("rgb1", "0.2 0.3 0.4")
        texture_plane.set("rgb2", "0.1 0.15 0.2")
        texture_plane.set("markrgb", "0.8 0.8 0.8")
        texture_plane.set("width", "512")
        texture_plane.set("height", "512")
        
        # Material
        material = ET.SubElement(asset, "material")
        material.set("name", "matplane")
        material.set("texture", "texplane")
        material.set("texuniform", "true")
        material.set("texrepeat", "10 10")
        material.set("reflectance", "0.3")

        worldbody = ET.SubElement(mujoco_elem, "worldbody")
        contact = ET.SubElement(mujoco_elem, "contact")


        
        lengths_plvl = [0.041, 0.024, 0.025]
        segments_plvl = [24, 12, 6]
        branch_each_plvl = [8, 8, 8]
        segment_no = 0
        
        def add_level(level: int, parent_body: ET.Element, euler: str = "0 0 0"):
            """Recursively add levels to the tree."""
            nonlocal segment_no
            for i in range(segments_plvl[level]):
                segment_no += 1
                # Create a new body for the segment
                body = ET.SubElement(parent_body, "body")
                body.set("name", f"segment_{segment_no}")

                parent_body = body
                if i > 0:
                    euler = "0 0 0"  # Reset euler after the first segment
                # Set position and orientation
                pos = f"0 0 {lengths_plvl[level]}"
                body.set("pos", pos)
                body.set("euler", euler)
                
                # Add geometry
                geom = ET.SubElement(body, "geom")
                geom.set("name", f"G{segment_no}")
                geom.set("size", f"0.005 {lengths_plvl[level] / 2}")
                geom.set("pos", f"{lengths_plvl[level] / 2} 0 0")
                geom.set("type", "capsule")
                geom.set("condim", "1")
                geom.set("rgba", "0.487804 1 0 1")
                geom.set("euler", "0 0 1.8")

                plugin = ET.SubElement(body, "plugin")
                plugin.set("instance", "composite")

                # Add joint if not the first segment
                if segment_no > 1:
                    joint = ET.SubElement(body, "joint")
                    joint.set("name", f"J_{segment_no}")
                    joint.set("pos", "0 0 0")
                    joint.set("type", "ball")
                    joint.set("group", "3")
                    joint.set("actuatorfrclimited", "false")
                    joint.set("damping", "0.015")

                    exclude = ET.SubElement(contact, "exclude")
                    exclude.set("body1", f"segment_{segment_no - 1}")
                    exclude.set("body2", f"segment_{segment_no}")

                branch_per = branch_each_plvl[level]
                if i > 0 and branch_per > 0 and i % branch_per == 0 and level < len(lengths_plvl) - 1:
                    add_level(level + 1, body, euler="0.4 0 0")
                    add_level(level + 1, body, euler="-0.4 0 0")

        add_level(0, worldbody)

                
        
        # # Add ground plane
        # ground = ET.SubElement(worldbody, "geom")
        # ground.set("name", "ground")
        # ground.set("size", "0 0 1")
        # ground.set("type", "plane")
        # ground.set("condim", "1")
        # ground.set("material", "matplane")
        
        # # Add lights
        # light1 = ET.SubElement(worldbody, "light")
        # light1.set("pos", "0 0 2")
        # light1.set("dir", "0 0 -1")
        # light1.set("castshadow", "false")
        # light1.set("diffuse", "0.4 0.4 0.4")
        # light1.set("specular", "0.1 0.1 0.1")
        # light2 = ET.SubElement(worldbody, "light")
        # light2.set("pos", "0 0 4")
        # light2.set("dir", "0 0 -1")
        # light2.set("directional", "true")
        # light2.set("diffuse", "0.8 0.8 0.8")
        # light2.set("specular", "0.2 0.2 0.2")


        # # Create a bending cable with a fixed base
        # parent_body = worldbody
        # branch_body = worldbody
        # step = 0.025 
        # num_segments = 12
        # for i in range(num_segments):
        #     body = ET.SubElement(parent_body, "body")
        #     body.set("name", f"segment_{i}")
        #     body.set("pos", f"0 0 {step}")
        #     body.set("euler", "0.1 0 0")
            
        #     geom = ET.SubElement(body, "geom")
        #     geom.set("name", f"G{i}")
        #     geom.set("size", f"0.005 0.0025")
        #     geom.set("pos", f"{step / 2} 0 0")
        #     geom.set("type", "capsule")
        #     geom.set("condim", "1")
        #     geom.set("rgba", "0.487804 1 0 1")
        #     geom.set("euler", "1.4 0 0")
        #     # geom.set("quat", "0.707107 0 -0.707107 0")

        #     plugin = ET.SubElement(body, "plugin")
        #     plugin.set("instance", "composite")

        #     parent_body = body

        #     if i == 0:
        #         site = ET.SubElement(body, "site")
        #         site.set("name", "S_first")
        #         site.set("pos", "0 0 0")
        #         site.set("group", "3")
        #     if i == num_segments - 1:
        #         site = ET.SubElement(body, "site")
        #         site.set("name", "S_last")
        #         site.set("pos", f"{step / 2} 0 0")
        #         site.set("group", "3")

        #     if i == 3:
        #         branch_body = body

        #     if i > 0:
        #         joint = ET.SubElement(body, "joint")
        #         joint.set("name", f"J_{i}")
        #         joint.set("pos", "0 0 0")
        #         joint.set("type", "ball")
        #         joint.set("group", "3")
        #         joint.set("actuatorfrclimited", "false")
        #         joint.set("damping", "0.015")

        #         exclude = ET.SubElement(contact, "exclude")
        #         exclude.set("body1", f"segment_{i-1}")
        #         exclude.set("body2", f"segment_{i}")

        # num_segments = 6
        # parent_body = branch_body
        # for i in range(num_segments):
        #     body = ET.SubElement(parent_body, "body")
        #     body.set("name", f"segment_{i}b")
        #     body.set("pos", f"0 0 {step}")
        #     body.set("euler", "-0.1 0 0")
            
        #     geom = ET.SubElement(body, "geom")
        #     geom.set("name", f"G{i}b")
        #     geom.set("size", f"0.005 0.0025")
        #     geom.set("pos", f"{step / 2} 0 0")
        #     geom.set("type", "capsule")
        #     geom.set("condim", "1")
        #     geom.set("rgba", "0.487804 1 0 1")
        #     geom.set("euler", "1.4 0 0")
        #     # geom.set("quat", "0.707107 0 -0.707107 0")

        #     plugin = ET.SubElement(body, "plugin")
        #     plugin.set("instance", "composite")

        #     parent_body = body

        #     # if i == 0:
        #     #     site = ET.SubElement(body, "site")
        #     #     site.set("name", "S_first")
        #     #     site.set("pos", "0 0 0")
        #     #     site.set("group", "3")
        #     # if i == num_segments - 1:
        #     #     site = ET.SubElement(body, "site")
        #     #     site.set("name", "S_last")
        #     #     site.set("pos", f"{step / 2} 0 0")
        #     #     site.set("group", "3")

        #     # if i == 5:
        #     #     branch_body = body

        #     joint = ET.SubElement(body, "joint")
        #     joint.set("name", f"J_{i}b")
        #     joint.set("pos", "0 0 0")
        #     joint.set("type", "ball")
        #     joint.set("group", "3")
        #     joint.set("actuatorfrclimited", "false")
        #     joint.set("damping", "0.015")

        #     exclude = ET.SubElement(contact, "exclude")
        #     parent = f"segment_{i-1}" if i > 0 else "segment_3"
        #     exclude.set("body1", parent)
        #     exclude.set("body2", f"segment_{i}b")

        
    # <body name="slider" pos="0.7 0 0.6">
    #   <joint pos="0 0 0" axis="1 0 0" type="slide" damping="0.1"/>
    #   <geom size="0.01"/>
    # </body>
        # slider_body = ET.SubElement(worldbody, "body")
        # slider_body.set("name", "slider")
        # slider_body.set("pos", "0.7 0 0.6")
        # slider_joint = ET.SubElement(slider_body, "joint")
        # slider_joint.set("pos", "0 0 0")
        # slider_joint.set("axis", "1 0 0")
        # slider_joint.set("type", "slide")
        # slider_joint.set("damping", "0.1")
        # slider_geom = ET.SubElement(slider_body, "geom")
        # slider_geom.set("size", "0.01")
        # slider_exclude = ET.SubElement(contact, "exclude")
        # slider_exclude.set("body1", f"segment_{num_segments-1}")
        # slider_exclude.set("body2", "slider")
#         <equality>
#     <connect name="right_boundary" body1="B_last" body2="slider" anchor="0.025 0 0"/>
#   </equality>
        # equality = ET.SubElement(mujoco_elem, "equality")
        # connect = ET.SubElement(equality, "connect")
        # connect.set("name", "right_boundary")
        # connect.set("body1", f"segment_{num_segments-1}")
        # connect.set("body2", "slider")
        # connect.set("anchor", f"{step / 2} 0 0")



        # actuator = ET.SubElement(mujoco_elem, "actuator")
        # general_actuator = ET.SubElement(actuator, "general")
        # general_actuator.set("site", "S_last")
        # general_actuator.set("ctrlrange", "-0.03 0.03")
        # general_actuator.set("gear", "0 0 0 1 0 0") 
        return mujoco_elem

def main():
    """Generate different tree configurations"""
    
    generator = TreeGenerator()
    
    # Configuration 1: Simple tree with cable-like parameters
    print("Generating simple tree...")
    tree1 = generator.create_mujoco_tree()
    xml_path = Path("simple_tree.xml")
    tree1_str = ET.tostring(tree1, encoding='unicode', method='xml')
    with open(xml_path, "w") as f:
        f.write(minidom.parseString(tree1_str).toprettyxml(indent="  "))
    
    print("Tree generation complete!")
    print("Generated file: simple_tree.xml")

    # xml_path = "mj_test/model/plugin/elasticity/coil.xml"
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    viewer = mujoco.viewer.launch(model, data)
    # while viewer.is_running():
    #     mujoco.mj_step(model, data)
    #     viewer.sync()
    # viewer.close()

if __name__ == "__main__":
    main()