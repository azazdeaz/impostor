use pyo3::prelude::*;
use std::collections::HashMap;

// --- Core Data Structures ---

#[pyclass(name = "World")]
#[derive(Clone, Debug, Default)]
pub struct PyWorld {
    #[pyo3(get, set)]
    pub name: String,
    #[pyo3(get)]
    pub models: HashMap<String, PyModel>,
}

#[pymethods]
impl PyWorld {
    #[new]
    fn new(name: String) -> Self {
        PyWorld {
            name,
            models: HashMap::new(),
        }
    }

    fn add_model(&mut self, model: PyModel) -> Self {
        self.models.insert(model.name.clone(), model);
        self.clone()
    }

    fn __repr__(&self) -> String {
        format!("<World name='{}' models={}>", self.name, self.models.len())
    }
}

#[pyclass(name = "Model")]
#[derive(Clone, Debug, Default)]
pub struct PyModel {
    #[pyo3(get, set)]
    pub name: String,
    #[pyo3(get)]
    pub links: HashMap<String, PyLink>,
}

#[pymethods]
impl PyModel {
    #[new]
    fn new(name: String) -> Self {
        PyModel {
            name,
            links: HashMap::new(),
        }
    }

    fn add_link(&mut self, link: PyLink) -> Self {
        self.links.insert(link.name.clone(), link);
        self.clone()
    }

    fn __repr__(&self) -> String {
        format!("<Model name='{}' links={}>", self.name, self.links.len())
    }
}

#[pyclass(name = "Link")]
#[derive(Clone, Debug, Default)]
pub struct PyLink {
    #[pyo3(get, set)]
    pub name: String,
    #[pyo3(get)]
    pub visuals: Vec<PyGeometry>,
}

#[pymethods]
impl PyLink {
    #[new]
    fn new(name: String) -> Self {
        PyLink {
            name,
            visuals: Vec::new(),
        }
    }

    fn add_visual(&mut self, visual: PyGeometry) -> Self {
        self.visuals.push(visual);
        self.clone()
    }

    fn __repr__(&self) -> String {
        format!("<Link name='{}' visuals={}>", self.name, self.visuals.len())
    }
}

#[pyclass(name = "Geometry")]
#[derive(Clone, Debug)]
pub struct PyGeometry {
    pub shape: Shape,
    #[pyo3(get)]
    pub color: (f32, f32, f32), // RGB
}

#[derive(Clone, Debug)]
pub enum Shape {
    Box { size: (f64, f64, f64) },
    Sphere { radius: f64 },
    Cylinder { radius: f64, length: f64 },
    Mesh { path: String },
}

#[pymethods]
impl PyGeometry {
    #[staticmethod]
    fn new_box(x: f64, y: f64, z: f64) -> Self {
        PyGeometry {
            shape: Shape::Box { size: (x, y, z) },
            color: (1.0, 1.0, 1.0),
        }
    }

    #[staticmethod]
    fn new_sphere(radius: f64) -> Self {
        PyGeometry {
            shape: Shape::Sphere { radius },
            color: (1.0, 1.0, 1.0),
        }
    }

    #[staticmethod]
    fn new_cylinder(radius: f64, length: f64) -> Self {
        PyGeometry {
            shape: Shape::Cylinder { radius, length },
            color: (1.0, 1.0, 1.0),
        }
    }

    #[staticmethod]
    fn new_mesh(path: String) -> Self {
        PyGeometry {
            shape: Shape::Mesh { path },
            color: (1.0, 1.0, 1.0),
        }
    }

    pub fn set_color(&mut self, r: f32, g: f32, b: f32) -> Self {
        self.color = (r, g, b);
        self.clone()
    }

    fn __repr__(&self) -> String {
        let shape_str = match &self.shape {
            Shape::Box { size } => format!("<Shape::Box size={:?}>", size),
            Shape::Sphere { radius } => format!("<Shape::Sphere radius={}>", radius),
            Shape::Cylinder { radius, length } => {
                format!("<Shape::Cylinder radius={}, length={}>", radius, length)
            }
            Shape::Mesh { path } => format!("<Shape::Mesh path='{}'>", path),
        };
        format!(
            "<Geometry shape={}, color={:?}>",
            shape_str, self.color
        )
    }
}

/// A Python module implemented in Rust.
#[pymodule]
fn impostor(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyWorld>()?;
    m.add_class::<PyModel>()?;
    m.add_class::<PyLink>()?;
    m.add_class::<PyGeometry>()?;
    Ok(())
}
