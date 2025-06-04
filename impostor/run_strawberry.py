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
    scaffolding_solver = parts.ScaffoldingSolver()
    scaffolding_solver_entity = plant.create_entity(scaffolding_solver)

    for i in range(iterations):
        print(f"Frame {i}")
        stepper.step_parts(plant)
        rr.set_time_sequence("frame_idx", i)
        # syst.rr_log_components(plant)
        # syst.rr_log_transforms_system(plant)

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

    # Create a sphere collider entity for the dragged body
    collider_entity = plant.create_entity(parts.SphereCollider(radius=0.015), parts.RigidTransformation())


    scaffolding_solver.optimization_steps = 12
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
            # # Detect user drag
            # for i in range(model.nmocap):
            #     if not np.allclose(data.mocap_pos[i], last_mocap_pos[i]):
            #         print(f"Mocap body {i} is being dragged by the user!")
            # last_mocap_pos = data.mocap_pos.copy()

            # Find the body called "draggable" and copy its position into the collider entity
            draggable_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "draggable")
            transform = plant.get_components(collider_entity).get_by_type(parts.RigidTransformation)
            
            # Update the collider's position to match the draggable body
            transform.translation = data.xpos[draggable_body_id].copy()
            
            # Log the new position
            # rr.log("collider/position", rr.Points3D([transform.translation]))

            # Time the scaffolding solver step
            solver_start = time.time()
            scaffolding_solver.step(plant, scaffolding_solver_entity, bail=True)
            solver_end = time.time()
            solver_duration = solver_end - solver_start
            
            print(f"Scaffolding solver step took: {solver_duration:.4f} seconds")
            # rr.log("performance/solver_step_time", rr.Scalar(solver_duration))
            # syst.rr_log_transforms_system(plant)

            # Copy the entity transforms to the mujoco bodies
            for entity in plant.query().with_components(
                parts.RigidTransformation
            ).entities():
                body_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_BODY, f"body_{entity}"
                )
                if body_id >= 0:
                    print(f"Updating body {body_id} for entity {entity}")
                    mocap_id = model.body_mocapid[body_id]
                    if mocap_id >= 0:
                        print(f"Updating mocap body {mocap_id} for entity {entity}")
                        transform = plant.get_components(entity).get_by_type(parts.RigidTransformation)
                        data.mocap_pos[mocap_id] = transform.translation.copy()
                        # # Add a random noise to the position for testing
                        # noise = np.random.normal(0, 0.001, size=3)
                        # data.mocap_pos[mocap_id] += noise
                        data.mocap_quat[mocap_id] = transform.rotation.as_quat().copy()

                    
            
            # Control frame rate (60 FPS)
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                print(f"Sleeping for {time_until_next_step:.4f} seconds to maintain frame rate")
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    # create a recording id so the Rust process can log to the same Rerun recording
    recording_id = str(int(time.time()))
    rr.init("impostor", spawn=True, recording_id=recording_id)

    test_grow(80)
