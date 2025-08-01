import numpy as np
from impostor_gen.trail import Trail
from impostor_gen.extrude import extrude_mesh2d_along_points
from impostor_gen.mesh2d import Mesh2D
import rerun as rr

def main():
    """
    This is an example of how to create and use a Trail object.
    """
    # Create a trail starting at the origin, moving along the x-axis
    my_trail = Trail(
        start_point=np.array([0, 0, 0]),
        start_direction=np.array([1, 0, 0]),
        length=5.0,
        steps=20
    )

    # Get the list of transforms
    transforms = my_trail.transforms()

    profile = Mesh2D(
        vertex_positions=np.array([[0, 0], [1, 0], [1, 1], [0, 1]]),
        line_indices=np.array([[0, 1], [1, 2], [2, 3], [3, 0]])
    )

    # Extrude the profile along the trail
    extruded_mesh = extrude_mesh2d_along_points(profile, transforms)

    print("Extruded Mesh:")
    print(extruded_mesh)

    rr.init("rerun_example_my_data", spawn=True)

    rr.log("extruded_mesh", extruded_mesh.to_rerun())

    

if __name__ == "__main__":
    main()