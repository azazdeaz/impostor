use crate::structs::SoftBody;

impl SoftBody {
    pub fn build_rod() -> Self {
        SoftBody { ..Default::default() }
    }
}