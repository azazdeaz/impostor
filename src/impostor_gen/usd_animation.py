from impostor_gen.material import MaterialRegistry
from impostor_gen.mesh3d import CompundMesh3D


from pxr import Usd, UsdGeom
from pydantic import BaseModel, Field


from typing import List, Optional


class UsdAnimation(BaseModel):
    """
    A class to build USD animations by adding mesh frames sequentially.
    Each frame is stored as a time sample in the USD stage.
    """

    frames: List[CompundMesh3D] = Field(default_factory=lambda: [])
    materials_registry: Optional[MaterialRegistry] = Field(default=None)
    fps: float = Field(default=24.0, description="Frames per second for the animation")

    class Config:
        arbitrary_types_allowed = True

    def add_next_frame(self, mesh: CompundMesh3D) -> None:
        """
        Add the next frame to the animation.

        Args:
            mesh: The CompundMesh3D to add as the next frame
        """
        self.frames.append(mesh)

    def save(self, filename: str) -> Usd.Stage:
        """
        Save the animation to a USD file.

        Args:
            filename: Path to the output USD file

        Returns:
            The created USD stage
        """
        if not self.frames:
            raise ValueError("No frames added to animation")

        if self.materials_registry is None:
            raise ValueError("No materials registry set")

        stage = Usd.Stage.CreateNew(filename)

        # Set time parameters
        stage.SetStartTimeCode(0)
        stage.SetEndTimeCode(len(self.frames) - 1)
        stage.SetTimeCodesPerSecond(self.fps)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

        # Create a parent xform for the animation
        UsdGeom.Xform.Define(stage, "/AnimationRoot")

        # Create a new xform for each frame
        for frame_idx in range(len(self.frames)):
            frame_path = f"/AnimationRoot/frame_{frame_idx}"
            frame_prim = UsdGeom.Xform.Define(stage, frame_path)

            for submesh in self.frames[frame_idx].submeshes:
                submesh.to_usd(stage, self.materials_registry, parent=frame_path)

                # Set animated visibility
                visibility_attr = frame_prim.CreateVisibilityAttr()
                visibility_attr.Set(UsdGeom.Tokens.invisible, time=0)
                visibility_attr.Set(UsdGeom.Tokens.inherited, time=float(frame_idx))
                if frame_idx < len(self.frames) - 1:
                    visibility_attr.Set(
                        UsdGeom.Tokens.invisible, time=float(frame_idx + 1)
                    )

        stage.Save()
        return stage
