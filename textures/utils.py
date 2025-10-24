from pathlib import Path
from typing import List, Tuple
from PIL import Image


def create_alpha_mask(image, tolerance=50):
    """
    Create a black and white alpha mask using the top-left pixel color as the clip color.

    Args:
        image: PIL Image in RGB or RGBA mode
        tolerance: How close colors need to be to the clip color (0-255)

    Returns:
        PIL Image: Black and white mask where white=opaque, black=transparent
    """
    # Get the background color from top-left pixel
    clip_color = image.getpixel((0, 0))
    print("Background color:", clip_color)

    # Convert to RGB if needed for consistent processing
    if image.mode == "RGBA":
        image = image.convert("RGB")

    # Get image data as array
    data = image.getdata()
    mask_data = []

    for pixel in data:
        r, g, b = pixel[:3]  # Get RGB values
        cr, cg, cb = clip_color[:3]  # Handle both RGB and RGBA clip colors

        # Calculate color distance
        distance = ((r - cr) ** 2 + (g - cg) ** 2 + (b - cb) ** 2) ** 0.5

        if distance <= tolerance:
            # Background color = black (transparent)
            mask_data.append(0)
        else:
            # Foreground = white (opaque)
            mask_data.append(255)

    # Create new grayscale mask image
    mask = Image.new("L", image.size)
    mask.putdata(mask_data)
    return mask


def get_bounding_box_from_mask(mask):
    """
    Get the bounding box of non-black pixels from an alpha mask.

    Args:
        mask: PIL Image in grayscale mode ('L') where white=content, black=background

    Returns:
        tuple: (left, top, right, bottom) bounding box or None if no content found
    """
    # For grayscale masks, getbbox() finds non-zero pixels
    bbox = mask.getbbox()
    if bbox:
        print(f"Bounding box from mask: {bbox}")
        print(f"Size: {bbox[2] - bbox[0]} x {bbox[3] - bbox[1]}")
    else:
        print("No content found in mask")
    return bbox


def crop_images_to_bbox(images, bbox):
    """
    Crop a list of images to the same bounding box.

    Args:
        images: List of PIL Images
        bbox: tuple (left, top, right, bottom) bounding box

    Returns:
        List of cropped PIL Images, all with the same size
    """
    if not images or not bbox:
        return images

    cropped_images = []
    for img in images:
        cropped = img.crop(bbox)
        cropped_images.append(cropped)

    return cropped_images


def crop_to_square(image: Image, padding=10):
    """
    Crop the image to bounding square of non-transparent pixels with optional padding.

    Args:
        image: PIL Image in RGBA mode
        padding: Number of pixels to pad around the content
    Returns:
        PIL Image: Cropped image
    """
    mask = create_alpha_mask(image)
    bbox = get_bounding_box_from_mask(mask)
    if not bbox:
        raise ValueError("No content found to crop.")
    # Make the bounding box square with padding
    left, top, right, bottom = bbox
    width = right - left
    height = bottom - top
    max_side = max(width, height) + 2 * padding
    center_x = left + width // 2
    center_y = top + height // 2
    left = max(0, center_x - max_side // 2)
    top = max(0, center_y - max_side // 2)
    right = min(image.width, center_x + max_side // 2)
    bottom = min(image.height, center_y + max_side // 2)
    return image.crop((left, top, right, bottom))


def create_alpha_mask(image, tolerance=50):
    """
    Create a black and white alpha mask using the top-left pixel color as the clip color.

    Args:
        image: PIL Image in RGB or RGBA mode
        tolerance: How close colors need to be to the clip color (0-255)

    Returns:
        PIL Image: Black and white mask where white=opaque, black=transparent
    """
    # Get the background color from top-left pixel
    clip_color = image.getpixel((0, 0))
    print("Background color:", clip_color)

    # Convert to RGB if needed for consistent processing
    if image.mode == "RGBA":
        image = image.convert("RGB")

    # Get image data as array
    data = image.getdata()
    mask_data = []

    for pixel in data:
        r, g, b = pixel[:3]  # Get RGB values
        cr, cg, cb = clip_color[:3]  # Handle both RGB and RGBA clip colors

        # Calculate color distance
        distance = ((r - cr) ** 2 + (g - cg) ** 2 + (b - cb) ** 2) ** 0.5

        if distance <= tolerance:
            # Background color = black (transparent)
            mask_data.append(0)
        else:
            # Foreground = white (opaque)
            mask_data.append(255)

    # Create new grayscale mask image
    mask = Image.new("L", image.size)
    mask.putdata(mask_data)
    return mask


import cv2
import matplotlib.pyplot as plt
import numpy as np
import triangle as tr
from PIL import Image
from pxr import Usd, UsdGeom, Kind, UsdSkel, Gf


def triangulate_mask(
    mask: np.ndarray,
    stage: Usd.Stage,
    epsilon: float = 1.2,
    max_area: float = 1000,
    debug: bool = False,
):
    """
    Create a triangular mesh from a binary mask using the triangle library.

    Args:
        mask: NumPy array representing a binary mask (white=leaf, black=background)
        epsilon: Approximation accuracy for contour simplification (larger means more simplification)
        max_area: Maximum area for triangles in the mesh
        debug: If True, display the triangulation result using matplotlib
    """

    # Convert the PIL image to a NumPy array for use with OpenCV
    numpy_image = np.array(mask)
    # Take only one channel if it's multi-channel
    if len(numpy_image.shape) == 3:
        numpy_image = numpy_image[:, :, 0]

    # Threshold the NumPy array to get a binary mask
    _, mask = cv2.threshold(numpy_image, 127, 255, cv2.THRESH_BINARY)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # `contours` is a list of all found outlines. For a simple leaf,
    # you'll likely want the largest one.
    leaf_contour = max(contours, key=cv2.contourArea)

    leaf_contour = cv2.approxPolyDP(leaf_contour, epsilon, True)

    # Squeeze the array to remove the extra dimension.
    points = np.squeeze(leaf_contour)

    # To keep triangles inside the shape, we must define the polygon's boundary.
    # We create a list of segments connecting each point to the next, closing the loop.
    num_points = len(points)
    segments = np.array([[i, (i + 1) % num_points] for i in range(num_points)])

    bbox = cv2.boundingRect(points)
    padding = 10
    x, y, w, h = bbox
    frame_points = np.array(
        [
            [x - padding, y - padding],
            [x + w + padding, y - padding],
            [x + w + padding, y + h + padding],
            [x - padding, y + h + padding],
        ]
    )
    frame_segments = np.array([[i, (i + 1) % 4] for i in range(4)])

    points = np.vstack([frame_points, points])
    segments = np.vstack([frame_segments, segments + len(frame_points)])

    # The `triangle` library expects a dictionary with 'vertices' and 'segments'.
    polygon_to_triangulate = dict(
        vertices=points, segments=segments, holes=[[x - 1, y - 1]]
    )

    # Triangulate the polygon. The segments constrain the triangulation.
    # The 'q' flag can improve quality, and 'a' can set a max triangle area if needed.
    triangulation = tr.triangulate(polygon_to_triangulate, f"qpa{max_area}")

    if debug:
        tr.compare(plt, polygon_to_triangulate, triangulation)
        plt.show()

    center_x = x + w / 2
    center_y = y + h / 2
    # Re-center the vertices
    triangulation["vertices"][:, 0] -= center_x
    triangulation["vertices"][:, 1] -= center_y

    name = "/triangulated_mesh"
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    modelRoot = UsdGeom.Xform.Define(stage, f"{name}")
    Usd.ModelAPI(modelRoot).SetKind(Kind.Tokens.component)
    mesh = UsdGeom.Mesh.Define(stage, f"{name}/mesh")
    mesh.CreatePointsAttr([tuple(v) + (0.0,) for v in triangulation["vertices"]])
    face_vertex_counts = [3] * len(triangulation["triangles"])
    face_vertex_indices = triangulation["triangles"].flatten().tolist()
    mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
    mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)

    return stage


def add_bones_to_mesh(
    stage: Usd.Stage,
    mesh_path: str,
    root_position: Tuple[float, float],
    midrib_bones: List[Tuple[float, float]],
    secondary_bones: List[List[Tuple[float, float]]],
    debug: bool = False,
):
    """Add bones to the USD leaf mesh.

    Args:
        stage: The USD stage containing the mesh.
        mesh_path: The path to the mesh in the USD stage.
        root_position: The (x, y) position of the root bone.
        midrib_bones: A list of (yaw, forward) tuples for midrib bones.
        secondary_bones: A list of lists of (yaw, forward) tuples for secondary bones. Each sublist corresponds to one midrib bone.
        debug: Plot the topdown mesh with bones using matplotlib.
    """
    root = UsdSkel.Root.Define(stage, "/Root")
    editor = Usd.NamespaceEditor(stage)
    new_mesh_path = "/Root/mesh"
    editor.MovePrimAtPath(mesh_path, new_mesh_path)
    editor.ApplyEdits()
    mesh = UsdGeom.Mesh.Get(stage, new_mesh_path)
    skel = UsdSkel.Skeleton.Define(stage, "/Root/Skel")

    vertices = mesh.GetPointsAttr().Get()
    mesh_size = max(v[1] for v in vertices) - min(v[1] for v in vertices)

    mesh_binding = UsdSkel.BindingAPI.Apply(mesh.GetPrim())
    UsdSkel.BindingAPI.Apply(root.GetPrim())
    UsdSkel.BindingAPI.Apply(skel.GetPrim())

    mesh_binding.CreateSkeletonRel().AddTarget(skel.GetPath())

    joints: List[str] = ["root_bone"]
    bindTransforms: List[Gf.Matrix4d] = [
        Gf.Matrix4d().SetTranslate(Gf.Vec3d(root_position[0], root_position[1], 0.0))
    ]

    for i in range(len(midrib_bones)):
        joints.append(f"{joints[-1]}/mr{i}")
        bindTransforms.append(
            Gf.Matrix4d()
            .SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), midrib_bones[i][0]))
            .SetTranslateOnly(Gf.Vec3d(0, midrib_bones[i][1] * mesh_size, 0.0))
        )

    for i in range(len(secondary_bones)):
        path_left = [joints[i]]
        path_right = [joints[i]]
        for j in range(len(secondary_bones[i])):
            path_left.append(f"sb{i}_{j}l")
            path_right.append(f"sb{i}_{j}r")
            joints.append("/".join(path_left))
            joints.append("/".join(path_right))

            bindTransforms.append(
                Gf.Matrix4d()
                .SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), secondary_bones[i][j][0]))
                .SetTranslateOnly(
                    Gf.Vec3d(0, secondary_bones[i][j][1] * mesh_size, 0.0)
                )
            )
            bindTransforms.append(
                Gf.Matrix4d()
                .SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), -secondary_bones[i][j][0]))
                .SetTranslateOnly(
                    Gf.Vec3d(0, secondary_bones[i][j][1] * mesh_size, 0.0)
                )
            )
    restTransforms = bindTransforms.copy()
    joint_indices_primvar = mesh_binding.CreateJointIndicesPrimvar(
        constant=False, elementSize=4
    )
    all_indices = list(range(len(joints)))
    indices = [np.random.choice(all_indices, 4).tolist() for _ in range(len(vertices))]
    indices = np.array(indices).flatten().tolist()
    print("Joint indices per vertex:", indices)
    joint_indices_primvar.Set(indices)
    joint_weights_primvar = mesh_binding.CreateJointWeightsPrimvar(
        constant=False, elementSize=4
    )
    joint_weights_primvar.Set([1.0, 0.0, 0.0, 0.0] * len(vertices))

    mesh_binding.CreateGeomBindTransformAttr().Set(Gf.Matrix4d().SetIdentity())

    skel.CreateJointsAttr(joints)
    skel.CreateBindTransformsAttr(bindTransforms)
    skel.CreateRestTransformsAttr(restTransforms)
