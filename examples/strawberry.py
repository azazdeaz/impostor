import rerun as rr

from impostor_gen.engine import (
    AgeingRule,
    InterpolateRule,
    LSystem,
    StemGrowthRule,
)
from impostor_gen.material import MaterialRegistry
from impostor_gen.mesh.mesh_builder import generate_blueprints, generate_mesh, log_transforms
from impostor_gen.mesh.mesh_utils import log_mesh, log_wireframe
from impostor_gen.usd_animation import UsdAnimation
from utils.strawberry_plant import Crown, IterateCrown


def main():
    materials = MaterialRegistry()
    # materials.register(leaf_material)
    # materials.register(stem_material)

    # Define an L-system
    lsystem = LSystem(
        world=[Crown()],
        rules=[
            InterpolateRule(),
            StemGrowthRule(),
            AgeingRule(),
            IterateCrown(),
        ],
    )

    rr.init("rerun_example_my_data", spawn=True)

    anim = UsdAnimation(materials_registry=materials, fps=10.0)

    iterations = 50
    for i in range(iterations):
        rr.set_time("frame_idx", sequence=i)
        lsystem.iterate()
        print(f"Iteration {i}, world size: {len(lsystem.world)}")
        # if i % 5 == 0 or i == iterations - 1:
        blueprints = generate_blueprints(lsystem.world)
        log_transforms(blueprints)
        meshes = generate_mesh(blueprints)
        log_mesh(meshes, materials)
        log_wireframe(meshes)
        lsystem.log_graph()
        lsystem.log_as_markdown()

        # anim.add_next_frame(meshes)

        # if i == iterations - 1:
        #     meshes.to_usd(materials).Save()
        #     for j, mesh in enumerate(meshes.submeshes):
        #         Path(f"exports/strawberry_plant_{j}").mkdir(parents=True, exist_ok=True)
        #         mesh.to_trimesh(materials).export( # type: ignore
        #             f"exports/strawberry_plant_{j}/model.glb"
        #         )
        #         mesh.to_trimesh(materials).export( # type: ignore
        #             f"exports/strawberry_plant_{j}/model.obj"
        #         )

    # anim.save("strawberry_plant_animation.usd")


if __name__ == "__main__":
    main()
