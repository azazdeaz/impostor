from pathlib import Path
from typing import Optional

import trimesh.visual.material
from PIL import Image
from pxr import Sdf, Usd, UsdShade
from pydantic import BaseModel, Field


class Material(BaseModel):
    texture_base_color: Optional[Path] = Field(
        default=None, description="Path to the texture base color image"
    )
    texture_normal_map: Optional[Path] = Field(
        default=None, description="Path to the texture normal map image"
    )
    texture_opacity_map: Optional[Path] = Field(
        default=None, description="Path to the texture opacity map image"
    )
    texture_occlusion_map: Optional[Path] = Field(
        default=None, description="Path to the texture occlusion map image"
    )
    texture_displacement_map: Optional[Path] = Field(
        default=None, description="Path to the texture displacement map image"
    )

    def to_trimesh(self):
        return trimesh.visual.material.PBRMaterial(
            baseColorTexture=Image.open(self.texture_base_color)
            if self.texture_base_color
            else None,
            normalTexture=Image.open(self.texture_normal_map)
            if self.texture_normal_map
            else None,
            occlusionTexture=Image.open(self.texture_opacity_map)
            if self.texture_opacity_map
            else None,
        )

    def to_usd(self, stage: Usd.Stage, prim_path: str):
        material = UsdShade.Material.Define(stage, f"{prim_path}")
        pbrShader = UsdShade.Shader.Define(stage, f"{prim_path}/PBRShader")
        pbrShader.CreateIdAttr("UsdPreviewSurface")
        pbrShader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
        pbrShader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        pbrShader.CreateInput("opacityMode", Sdf.ValueTypeNames.Token).Set("presence")
        pbrShader.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.001)

        material.CreateSurfaceOutput().ConnectToSource(
            pbrShader.ConnectableAPI(), "surface"
        )

        stReader = UsdShade.Shader.Define(stage, f"{prim_path}/stReader")
        stReader.CreateIdAttr("UsdPrimvarReader_float2")

        if self.texture_base_color:
            diffuseTextureSampler = UsdShade.Shader.Define(
                stage, f"{prim_path}/diffuseTexture"
            )
            diffuseTextureSampler.CreateIdAttr("UsdUVTexture")
            diffuseTextureSampler.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
                str(self.texture_base_color)
            )
            diffuseTextureSampler.CreateInput(
                "st", Sdf.ValueTypeNames.Float2
            ).ConnectToSource(stReader.ConnectableAPI(), "result")
            diffuseTextureSampler.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            pbrShader.CreateInput(
                "diffuseColor", Sdf.ValueTypeNames.Color3f
            ).ConnectToSource(diffuseTextureSampler.ConnectableAPI(), "rgb")

        if self.texture_normal_map:
            normalTextureSampler = UsdShade.Shader.Define(
                stage, f"{prim_path}/normalTexture"
            )
            normalTextureSampler.CreateIdAttr("UsdUVTexture")
            normalTextureSampler.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
                str(self.texture_normal_map)
            )
            normalTextureSampler.CreateInput(
                "st", Sdf.ValueTypeNames.Float2
            ).ConnectToSource(stReader.ConnectableAPI(), "result")
            normalTextureSampler.CreateInput(
                "sourceColorSpace", Sdf.ValueTypeNames.Token
            ).Set("raw")
            normalTextureSampler.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            pbrShader.CreateInput(
                "normal", Sdf.ValueTypeNames.Normal3f
            ).ConnectToSource(normalTextureSampler.ConnectableAPI(), "rgb")

        if self.texture_opacity_map:
            opacityTextureSampler = UsdShade.Shader.Define(
                stage, f"{prim_path}/opacityTexture"
            )
            opacityTextureSampler.CreateIdAttr("UsdUVTexture")
            opacityTextureSampler.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
                str(self.texture_opacity_map)
            )
            opacityTextureSampler.CreateInput(
                "st", Sdf.ValueTypeNames.Float2
            ).ConnectToSource(stReader.ConnectableAPI(), "result")
            opacityTextureSampler.CreateOutput("r", Sdf.ValueTypeNames.Float)
            pbrShader.CreateInput("opacity", Sdf.ValueTypeNames.Float).ConnectToSource(
                opacityTextureSampler.ConnectableAPI(), "r"
            )

        if self.texture_displacement_map:
            displacementTextureSampler = UsdShade.Shader.Define(
                stage, f"{prim_path}/displacementTexture"
            )
            displacementTextureSampler.CreateIdAttr("UsdUVTexture")
            displacementTextureSampler.CreateInput(
                "file", Sdf.ValueTypeNames.Asset
            ).Set(str(self.texture_displacement_map))
            displacementTextureSampler.CreateInput(
                "sourceColorSpace", Sdf.ValueTypeNames.Token
            ).Set("raw")
            displacementTextureSampler.CreateInput(
                "st", Sdf.ValueTypeNames.Float2
            ).ConnectToSource(stReader.ConnectableAPI(), "result")
            displacementTextureSampler.CreateOutput("r", Sdf.ValueTypeNames.Float)
            pbrShader.CreateInput(
                "displacement", Sdf.ValueTypeNames.Float
            ).ConnectToSource(displacementTextureSampler.ConnectableAPI(), "r")

        stInput = material.CreateInput("frame:stPrimvarName", Sdf.ValueTypeNames.Token)
        stInput.Set("st")

        stReader.CreateInput("varname", Sdf.ValueTypeNames.Token).ConnectToSource(
            stInput
        )

        return material


class MaterialRegistry(BaseModel):
    materials: dict[str, Material] = Field(default_factory=lambda: {})

    def get(self, key: str) -> Optional[Material]:
        return self.materials.get(key, None)

    def register(self, key: str, material: Material):
        self.materials[key] = material
