#[derive(serde::Deserialize, Debug)]
pub struct Pose {
    pub position: (f32, f32, f32),
    pub orientation: (f32, f32, f32, f32),
}

#[derive(serde::Deserialize, Debug)]
pub struct StemRing {
    pub pose: Pose,
    pub radius: f32,
}

#[derive(serde::Deserialize, Debug)]
pub struct Stem {
    pub rings: Vec<StemRing>,
}

#[derive(serde::Deserialize, Debug)]
pub struct Plant {
    pub stem: Stem,
}

