use pyo3::prelude::*;
use std::f64::consts::FRAC_PI_2;

// use materials::*;
use nalgebra::{Point3, Rotation3, Translation3, Vector3};

use curvo::prelude::*;
// mod materials;

/// Formats the sum of two numbers as string.
#[pyfunction]
fn sum_as_string(a: usize, b: usize) -> PyResult<String> {
    Ok((a + b).to_string())
}

#[pyfunction]
fn test_mesh() -> PyResult<String> {
    let interpolation_target = vec![
        Point3::new(-1.0, -1.0, 0.),
        Point3::new(1.0, -1.0, 0.),
        Point3::new(1.0, 1.0, 0.),
        Point3::new(-1.0, 1.0, 0.),
        Point3::new(-1.0, 2.0, 0.),
        Point3::new(1.0, 2.5, 0.),
    ];
    let interpolated = NurbsCurve3D::<f64>::try_interpolate(&interpolation_target, 3).unwrap();
    
    let c1 = NurbsCurve3D::try_circle(
        &Point3::origin(),
        &Vector3::x(),
        &Vector3::z(),
        1.4
    ).unwrap();
    let c2 = NurbsCurve3D::try_circle(
        &Point3::new(0., 1.5, 0.),
        &Vector3::x(),
        &Vector3::z(),
        1.
    ).unwrap();
    let c3 = NurbsCurve3D::try_circle(
        &Point3::new(0., 2.5, 0.),
        &Vector3::x(),
        &Vector3::z(),
        0.1
    ).unwrap();
    let rotation = Rotation3::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
    let translation = Translation3::new(0., 0., 1.5);
    let m = translation * rotation;
    let front = interpolated.transformed(&(translation.inverse()).into());
    let back = interpolated.transformed(&m.into());

    let lofted = NurbsSurface::try_loft(&[c1, c2, c3], Some(3)).unwrap();

    // TODO visualize with rerun on the python side
    return Ok("ok".to_string());
}

/// A Python module implemented in Rust.
#[pymodule]
fn impostor_core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(sum_as_string, m)?)?;
    m.add_function(wrap_pyfunction!(test_mesh, m)?)?;
    Ok(())
}
