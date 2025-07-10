use pyo3::prelude::*;

#[pyclass(name = "Cube")]
#[derive(Clone)]
struct PyCube {
    position: (f64, f64, f64),
    color: String,
}

#[pymethods]
impl PyCube {
    #[new]
    fn new() -> Self {
        PyCube {
            position: (0.0, 0.0, 0.0),
            color: "#FFFFFF".to_string(),
        }
    }

    /// Translates the cube by the given x, y, and z values.
    /// Returns the cube instance to allow for method chaining.
    fn translate(&mut self, x: f64, y: f64, z: f64) -> Self {
        self.position.0 += x;
        self.position.1 += y;
        self.position.2 += z;
        self.clone()
    }

    /// Sets the color of the cube.
    /// Returns the cube instance to allow for method chaining.
    fn color(&mut self, value: String) -> Self {
        self.color = value;
        self.clone()
    }

    /// Prints the cube's current state to the console.
    fn print(&self) {
        println!(
            "Cube at position: {:?}, color: {}",
            self.position, self.color
        );
    }
}

/// A Python module implemented in Rust.
#[pymodule]
fn impostor(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyCube>()?;

    /// Creates a new Cube instance.
    #[pyfn(m)]
    #[pyo3(name = "cube")]
    fn cube_py() -> PyCube {
        PyCube::new()
    }

    Ok(())
}
