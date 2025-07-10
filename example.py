from impostor import cube

# Use the builder-like API
cube().translate(1.0, 2.0, 3.0).color("#FF0000").print()

# You can also use the methods step-by-step
red_cube = cube().color("#FF0000")
translated_red_cube = red_cube.translate(1.0, 0.0, 0.0)
translated_red_cube.print()

# The original instance is also updated
red_cube.print()
