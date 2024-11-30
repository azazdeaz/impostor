use impostor_msgs::Plant;
use pyo3::prelude::*;
use rerun::{
    demo_util::grid,
    external::glam::{self, UVec3},
};

use std::{f64::consts::FRAC_PI_2, fmt::Binary};

// use materials::*;
use nalgebra::{base, Point3, Quaternion, Rotation3, Translation3, UnitQuaternion, Vector3};

use curvo::prelude::*;
// mod materials;

/// Formats the sum of two numbers as string.
#[pyfunction]
fn sum_as_string(a: usize, b: usize) -> PyResult<String> {
    Ok((a + b).to_string())
}

#[pyfunction]
fn test_mesh(plant_data: &[u8], rerun_rec_id: String) -> anyhow::Result<String> {
    let rec = rerun::RecordingStreamBuilder::new("impostor")
        .recording_id(rerun_rec_id).connect_tcp()?;

    let plant: Plant = bincode::deserialize(plant_data).expect("Failed to deserialize Plant");
    log::error!("In Rust>: {:?}", plant);
    // let interpolation_target = vec![
    //     Point3::new(-1.0, -1.0, 0.),
    //     Point3::new(1.0, -1.0, 0.),
    //     Point3::new(1.0, 1.0, 0.),
    //     Point3::new(-1.0, 1.0, 0.),
    //     Point3::new(-1.0, 2.0, 0.),
    //     Point3::new(1.0, 2.5, 0.),
    // ];
    // let interpolated = NurbsCurve3D::<f64>::try_interpolate(&interpolation_target, 3).unwrap();

    let mut axes = Vec::new();

    for (stem_id, stem) in plant.stems.iter().enumerate() {
        let mut rings = Vec::new();
        for (ring_id, ring) in stem.rings.iter().enumerate() {
            let radius = ring.radius;
            let rotation = UnitQuaternion::from_quaternion(Quaternion::new(
                ring.pose.orientation.0,
                ring.pose.orientation.1,
                ring.pose.orientation.2,
                ring.pose.orientation.3,
            ));
            let c = NurbsCurve3D::try_circle(
                &Point3::new(
                    ring.pose.position.0,
                    ring.pose.position.1,
                    ring.pose.position.2,
                ),
                &rotation.transform_vector(&Vector3::x()),
                &rotation.transform_vector(&Vector3::y()),
                radius,
            )?;
            rings.push(c);
        }
        axes.push(rings);
    }

    // let c1 = NurbsCurve3D::try_circle(
    //     &Point3::origin(),
    //     &Vector3::x(),
    //     &Vector3::z(),
    //     1.4
    // ).unwrap();
    // let c2 = NurbsCurve3D::try_circle(
    //     &Point3::new(0., 1.5, 0.),
    //     &Vector3::x(),
    //     &Vector3::z(),
    //     1.
    // ).unwrap();
    // let c3 = NurbsCurve3D::try_circle(
    //     &Point3::new(0., 2.5, 0.),
    //     &Vector3::x(),
    //     &Vector3::z(),
    //     0.1
    // ).unwrap();
    let rotation = Rotation3::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
    let translation = Translation3::new(0., 0., 1.5);
    // let m = translation * rotation;
    // let front = interpolated.transformed(&(translation.inverse()).into());
    // let back = interpolated.transformed(&m.into());

    let tesselations = axes
        .iter()
        // remove all the stems that have less than 2 rings
        .filter(|rings| rings.len() > 1)
        .map(|rings| {
            let surf = NurbsSurface::try_loft(rings.as_slice(), Some(3))?;
            let option = AdaptiveTessellationOptions {
                norm_tolerance: 1e-2,
                ..Default::default()
            };
            let tess = surf.tessellate(Some(option));
            let tess = tess.cast::<f32>();
            anyhow::Ok(tess)
        })
        .filter_map(|result| match result {
            Ok(tess) => Some(tess),
            Err(e) => {
                log::warn!("Tessellation error: {:?}", e);
                None
            }
        })
        .collect::<Vec<_>>();

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

    // let points = grid(glam::Vec3::splat(-10.0), glam::Vec3::splat(10.0), 10);
    // let colors = grid(glam::Vec3::ZERO, glam::Vec3::splat(255.0), 10)
    //     .map(|v| rerun::Color::from_rgb(v.x as u8, v.y as u8, v.z as u8));
    // let vertices = tesselations.iter().flat_map(|tess| tess.points().iter().map(|pt| (pt.x, pt.y, pt.z)));

    // // let normals = tess.normals().iter().map(|pt| (pt.x, pt.y, pt.z));
    // // let uvs = tess.uvs().iter().map(|pt| (pt.x, pt.y));
    // // let faces = tess.faces().iter().map(|f| f.iter().map(|i| *i as u32)).collect::<Vec<_>>();
    // let normals = tesselations.iter().flat_map(|tess| tess.normals().iter().map(|pt| (pt.x, pt.y, pt.z)));
    // let uvs = tesselations.iter().flat_map(|tess| tess.uvs().iter().map(|pt| (pt.x, pt.y)));
    // let faces = tesselations.iter().flat_map(|tess| tess.faces().iter().map(|f| (f[0] as u32, f[1] as u32, f[2] as u32)));

    let mut vertices = Vec::new();
    let mut normals = Vec::new();
    let mut uvs = Vec::new();
    let mut faces = Vec::new();

    let mut base_index = 0;
    for tess in tesselations.iter() {
        for i in 0..tess.points().len() {
            let pt = tess.points()[i];
            let normal = tess.normals()[i].normalize();
            let uv = tess.uvs()[i];

            vertices.push((pt.x, pt.y, pt.z));
            normals.push((normal.x, normal.y, normal.z));
            uvs.push((uv.x, uv.y));
        }

        for face in tess.faces() {
            faces.push([
                (face[0] + base_index) as u32,
                (face[1] + base_index) as u32,
                (face[2] + base_index) as u32,
            ]);
        }

        base_index = vertices.len();
        // for i
        // let indices: UVec3 = tess
        //     .faces()
        //     .iter()
        //     .map(|f| f.iter().map(|i| *i as u32))
        //     .collect();
    }

    rec.log(
        "plant_mesh",
        &rerun::Mesh3D::new(vertices)
            .with_vertex_normals(normals)
            .with_vertex_texcoords(uvs)
            .with_triangle_indices(faces),
    )?;

    // TODO visualize with rerun on the python side
    return Ok("oki".to_string());
}

/// A Python module implemented in Rust.
#[pymodule]
fn impostor_core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    pyo3_log::init();

    m.add_function(wrap_pyfunction!(sum_as_string, m)?)?;
    m.add_function(wrap_pyfunction!(test_mesh, m)?)?;
    Ok(())
}
