#[derive(serde::Deserialize)]
pub struct Pose {
    pub position: (f32, f32, f32),
    pub orientation: (f32, f32, f32, f32),
}

#[derive(serde::Deserialize)]
pub struct StemRing {
    pub pose: Pose,
    pub radius: f32,
}

#[derive(serde::Deserialize)]
pub struct Stem {
    pub rings: Vec<StemRing>,
}

#[derive(serde::Deserialize)]
pub struct Plant {
    pub stem: Stem,
}

