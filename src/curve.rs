use pyo3::prelude::*;
use glam::{Vec3, Quat};

#[pyclass(name = "Curve")]
#[derive(Clone, Debug, Default)]
pub struct PyBezierCurve {
    points: Vec<Vec3>
}

impl PyBezierCurve {
    fn new(points: Vec<Vec3>) -> Self {
        PyBezierCurve { points }
    }

    fn to_points(&self, num_points: usize) -> Vec<PyCurvePoint> {
        if self.points.len() != 4 || num_points == 0 {
            return Vec::new();
        }

        let p0 = self.points[0];
        let p1 = self.points[1];
        let p2 = self.points[2];
        let p3 = self.points[3];

        let mut curve_points = Vec::with_capacity(num_points);

        for i in 0..num_points {
            let t = if num_points > 1 {
                i as f32 / (num_points - 1) as f32
            } else {
                0.0
            };
            
            let position = Self::get_point(p0, p1, p2, p3, t);
            let tangent = Self::get_tangent(p0, p1, p2, p3, t);

            // Use a stable up vector for normal calculation
            let mut up = Vec3::Y;
            if tangent.dot(up).abs() > 0.999 {
                up = Vec3::Z; // Use a different up vector if tangent is (almost) vertical
            }

            let binormal = tangent.cross(up).normalize();
            let normal = binormal.cross(tangent).normalize();
            
            // Create a rotation quaternion that aligns Z to the tangent and Y to the normal
            let rotation = Quat::from_mat3(&glam::Mat3::from_cols(tangent, normal, binormal));

            curve_points.push(PyCurvePoint {    
                position,
                tangent,
                normal,
                rotation,
            });
        }

        curve_points
    }

    fn get_point(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3, t: f32) -> Vec3 {
        let t = t.clamp(0.0, 1.0);
        let one_minus_t = 1.0 - t;
        let one_minus_t_sq = one_minus_t * one_minus_t;
        let t_sq = t * t;

        one_minus_t_sq * one_minus_t * p0 +
        3.0 * one_minus_t_sq * t * p1 +
        3.0 * one_minus_t * t_sq * p2 +
        t_sq * t * p3
    }

    fn get_tangent(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3, t: f32) -> Vec3 {
        let t = t.clamp(0.0, 1.0);
        let one_minus_t = 1.0 - t;
        
        let tangent = 
            3.0 * one_minus_t * one_minus_t * (p1 - p0) +
            6.0 * one_minus_t * t * (p2 - p1) +
            3.0 * t * t * (p3 - p2);
        
        tangent.normalize_or_zero()
    }
}

#[pyclass(name = "CurvePoint")]
#[derive(Clone, Debug, Default)]
pub struct PyCurvePoint {
    #[pyo3(get, set)]
    pub position: Vec3,
    #[pyo3(get, set)]
    pub tangent: Vec3,
    #[pyo3(get, set)]
    pub normal: Vec3,
    #[pyo3(get, set)]
    rotation: Quat,
}