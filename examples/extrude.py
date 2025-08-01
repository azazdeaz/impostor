import numpy as np
from impostor_gen.trail import Trail

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

    print(f"Generated {len(transforms)} transforms for the trail.")

    # Print the position of the first and last transform
    if transforms:
        print(f"First transform position: {transforms[0].position}")
        print(f"Last transform position: {transforms[-1].position}")

if __name__ == "__main__":
    main()