from pathlib import Path
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


import cv2
import matplotlib.pyplot as plt
import numpy as np
import triangle as tr
from PIL import Image
from  pxr import Usd, UsdGeom, Kind


def triangulate_mask(
    mask: np.ndarray, output: Path, epsilon: float = 1.2, max_area: float = 1000, debug: bool = False
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
    frame_segments = np.array(
        [[i, (i + 1) % 4] for i in range(4)]
    )

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


    stage = Usd.Stage.CreateNew(str(output))
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


