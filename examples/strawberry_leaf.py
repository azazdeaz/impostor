from impostor_gen.organs.strawberry_leaf import StrawberryLeaf
import rerun as rr


def main() -> None:
    rr.init("strawberry_leaf_experiment", spawn=True)

    leaf = StrawberryLeaf.from_guide("assets/strawberry_leaf_guide.svg")
    leaf.log_structure()

    print(f"Midrib length (m): {leaf.midrib_length_m:.6f}")
    print(f"Vein resolution (m): {leaf.vein_resolution_m:.6f}")
    print(f"Vein-free tip length (m): {leaf.vein_free_tip_length_m:.6f}")
    print(f"Midrib points: {len(leaf.midrib_positions_2d)}")
    print(f"Left secondary veins: {len(leaf.left_secondary_veins_2d)}")
    print(f"Right secondary veins: {len(leaf.right_secondary_veins_2d)}")


if __name__ == "__main__":
    main()
