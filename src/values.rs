use glam::{Affine3A, Quat, Vec3};
use pyo3::prelude::*;

#[pyclass(name = "Vec3")]
#[derive(Clone, Debug)]
pub struct PyVec3 {
    #[pyo3(get, set)]
    pub x: f32,
    #[pyo3(get, set)]
    pub y: f32,
    #[pyo3(get, set)]
    pub z: f32,
}

#[pymethods]
impl PyVec3 {
    #[new]
    fn new(x: f32, y: f32, z: f32) -> Self {
        PyVec3 { x, y, z }
    }
}

#[pyclass(name = "Rotation")]
#[derive(Clone, Debug)]
pub struct PyRotation {
    quat: Quat,
}

#[pymethods]
impl PyRotation {
    fn from_quat(x: f32, y: f32, z: f32, w: f32) -> Self {
        PyRotation {
            quat: Quat::from_xyzw(x, y, z, w),
        }
    }

    fn rotate_around(&self, axis: PyVec3, angle: f32) -> Self {
        let axis_vec = Vec3::new(axis.x, axis.y, axis.z);
        PyRotation {
            quat: self.quat * Quat::from_axis_angle(axis_vec, angle),
        }
    }
}

#[pyclass(name = "Transform")]
#[derive(Clone, Debug)]
pub struct PyTransform {
    transform: Affine3A,
}

#[pymethods]
impl PyTransform {
    fn from_translation(translation: PyVec3) -> Self {
        PyTransform {
            transform: Affine3A::from_translation(Vec3::new(translation.x, translation.y, translation.z)),
        }
    }

    fn from_rotation(rotation: Quat) -> Self {
        PyTransform {
            transform: Affine3A::from_quat(rotation),
        }
    }
}
