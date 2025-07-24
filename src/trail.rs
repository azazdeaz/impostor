use pyo3::prelude::*;
use glam::{Vec3, Quat, Affine3A};

#[pyclass(name = "Trail")]
#[derive(Clone)]
pub struct PyTrail {
    pub start_direction: glam::Vec3,
    pub start_position: glam::Vec3,
}



#[pymethods]
impl PyTrail {
    pub fn new(
        start_direction: glam::Vec3,
        start_position: glam::Vec3,
        x_bend: PyObject,
    ) -> Self {
        Python::with_gil(|py| {
            if !x_bend.is_callable() {
                return Err(pyo3::exceptions::PyTypeError::new_err(
                    "x_bend must be a callable",
                ));
            }
            Ok(())
        })?;

        Self {
            start_direction,
            start_position,
            x_bend,
        }
    }

    pub fn frames(&self, count: usize, length: f32) -> Vec<Affine3A> {
        let mut frames = Vec::with_capacity(count);
        for i in 0..count {
            let t = i as f32 / (count - 1) as f32;
            let x = (t * length).max(0.0);
            let y = (self.x_bend)(x);
            let z = t * (length - self.start_position.z) + self.start_position.z;
            frames.push(Affine3A::from_translation(Vec3::new(x, y, z)));
        }
        frames
    }
}