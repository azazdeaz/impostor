

use pyo3::prelude::*;
use std::f64::consts::FRAC_PI_2;
use rerun::{demo_util::grid, external::glam::{self, UVec3}};


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

    let surf = NurbsSurface::try_loft(&[c1, c2, c3], Some(3)).unwrap();

    let option = AdaptiveTessellationOptions {
        norm_tolerance: 1e-2,
        ..Default::default()
    };
    let tess = surf.tessellate(Some(option));
    let tess = tess.cast::<f32>();

    // let tess = surf.regular_tessellate(32, 32);

    // let mut line_list =Mesh::new(bevy::render::mesh::PrimitiveTopology::LineList, default());
    // let normal_length = 0.15;
    // let normals = tess.normals();

    // let vertices = tess
    //     .points()
    //     .iter()
    //     .enumerate()
    //     .flat_map(|(i, p)| {
    //         let pt: glam::Vec3 = (p.x, p.y, p.z).into();
    //         let normal = normals[i].normalize();
    //         let normal: glam::Vec3 = (normal.x, normal.y, normal.z).into();
    //         [pt, pt + normal * normal_length]
    //     })
    //     .map(|p| p.to_array())
    //     .collect();

    // line_list.insert_attribute(
    //     Mesh::ATTRIBUTE_POSITION,
    //     VertexAttributeValues::Float32x3(vertices),
    // );

    let rec = rerun::RecordingStreamBuilder::new("rerun_example_minimal").spawn().unwrap();

    // let points = grid(glam::Vec3::splat(-10.0), glam::Vec3::splat(10.0), 10);
    // let colors = grid(glam::Vec3::ZERO, glam::Vec3::splat(255.0), 10)
    //     .map(|v| rerun::Color::from_rgb(v.x as u8, v.y as u8, v.z as u8));
    
    let vertices = tess.points().iter().map(|pt| (pt.x, pt.y, pt.z));
    let normals = tess.normals().iter().map(|pt| (pt.x, pt.y, pt.z));
    let uvs = tess.uvs().iter().map(|pt| (pt.x, pt.y));
    // let indices: UVec3 = tess
    //     .faces()
    //     .iter()
    //     .map(|f| f.iter().map(|i| *i as u32))
    //     .collect();
    rec.log(
        "my_points",
        &rerun::Mesh3D::new(vertices)
            .with_vertex_normals(normals)
            .with_vertex_texcoords(uvs)
            .with_triangle_indices(tess.faces().iter().map(|f| [f[0] as u32, f[1] as u32, f[2] as u32])),
    ).unwrap();

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
