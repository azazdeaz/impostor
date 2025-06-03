from impostor.plant import Plant
import impostor.systems as syst
from impostor import parts
import rerun as rr
import time
import numpy as np
import mujoco
import mujoco.viewer


def test_grow(iterations=30):
    plant = Plant()
    stepper = parts.PartStepperSystem(
        exec_order=[
            parts.Collider,
            parts.ScaffoldingSolver,
            parts.Leaf,
        ]
    )
    plant.create_entity(parts.Crown())
    plant.create_entity(parts.ScaffoldingSolver())

    for i in range(iterations):
        print(f"Frame {i}")
        stepper.step_parts(plant)
        rr.set_time_sequence("frame_idx", i)
        # syst.rr_log_components(plant)
        syst.rr_log_transforms_system(plant)

        # mesh = syst.create_plant_mesh(plant)
        # mesh.rr_log()

    xml_path = f"plant_model_{iterations}.xml"
    mesh = syst.create_plant_mesh(plant)
    syst.save_mujoco_model(mesh, xml_path, model_name=f"plant_model_{i}")

    # Launch the MuJoCo viewer with mocap animation
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    print(f"Model has {model.nbody} bodies")
    print(f"Model has {model.nmocap} mocap bodies")
    
    # Store original mocap positions and quaternions
    original_mocap_pos = data.mocap_pos.copy()
    original_mocap_quat = data.mocap_quat.copy()
    
    print("Original mocap positions:", original_mocap_pos)
    print("Original mocap quaternions:", original_mocap_quat)
    
    last_mocap_pos = data.mocap_pos.copy()
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        
        while viewer.is_running():
            step_start = time.time()
            
            # # Animate mocap bodies with a wave
            # t = time.time() - start_time
            
            # for mocap_id in range(model.nmocap):
            #     # Create wave motion
            #     amplitude = 0.04  # Small amplitude for realistic motion
            #     frequency = 0.4   # 1 Hz oscillation
            #     phase_offset = mocap_id * 0.3  # Phase shift between bodies
                
            #     # Apply wave in X direction to position
            #     offset = amplitude * np.sin(2 * np.pi * frequency * t + phase_offset)
            #     data.mocap_pos[mocap_id] = original_mocap_pos[mocap_id].copy()
            #     offset *= np.sin(data.mocap_pos[mocap_id][2] * 20.0)
            #     data.mocap_pos[mocap_id][0] += offset  # Move in X direction
                
            #     # Optional: Add slight rotation as well
            #     rotation_amplitude = 0.1  # Small rotation in radians
            #     rotation_offset = rotation_amplitude * np.sin(2 * np.pi * frequency * t + phase_offset)
                
            #     # Create rotation quaternion around Z axis
            #     cos_half = np.cos(rotation_offset / 2)
            #     sin_half = np.sin(rotation_offset / 2)
            #     rotation_quat = np.array([cos_half, 0, 0, sin_half])  # [w, x, y, z]
                
            #     # Combine with original quaternion
            #     data.mocap_quat[mocap_id] = rotation_quat
                
                
            # Step the simulation
            mujoco.mj_step(model, data)
            viewer.sync()
            # Detect user drag
            for i in range(model.nmocap):
                if not np.allclose(data.mocap_pos[i], last_mocap_pos[i]):
                    print(f"Mocap body {i} is being dragged by the user!")
            last_mocap_pos = data.mocap_pos.copy()
            
            # Control frame rate (60 FPS)
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    # create a recording id so the Rust process can log to the same Rerun recording
    recording_id = str(int(time.time()))
    rr.init("impostor", spawn=True, recording_id=recording_id)

    test_grow(12)
