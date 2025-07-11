from impostor import World, Model, Link, Geometry

# Create a world and a model
world = World("my_first_simulation")
robot_model = Model("simple_robot")

# Create a link for the robot's body
body_link = Link("body")

# Create a box visual for the body and make it red
red_box = Geometry.new_box(0.5, 0.5, 1.0).set_color(1.0, 0.0, 0.0)
body_link.add_visual(red_box)

# Add the link to the model
robot_model.add_link(body_link)

# Create another link for a sensor, using a mesh
sensor_link = Link("sensor_unit")
mesh_geom = Geometry.new_mesh("path/to/your/sensor.obj").set_color(0.8, 0.8, 0.8)
sensor_link.add_visual(mesh_geom)

# Add the second link
robot_model.add_link(sensor_link)

# Add the complete model to the world
world.add_model(robot_model)


# You can also build things more fluently
world.add_model(
    Model("another_object")
    .add_link(
        Link("base")
        .add_visual(Geometry.new_cylinder(0.3, 0.1).set_color(0.3, 0.3, 0.3))
    )
)


# Print the world structure to see the result
print(world)
print(world.models["simple_robot"])
print(world.models["simple_robot"].links["body"])
print(world.models["simple_robot"].links["body"].visuals[0])
print(world.models["another_object"].links["base"].visuals[0])
