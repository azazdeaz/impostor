
use impostor_msgs::Plant;
use serde_reflection::{Tracer, TracerConfig};
use std::path::PathBuf;

pub fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut tracer = Tracer::new(TracerConfig::default());

    // Add all used message types to the tracer
    tracer
        .trace_simple_type::<Plant>()
        .unwrap();

    let registry = tracer.registry().unwrap();

    // Create Python class definitions.
    let mut source = Vec::new();
    let config = serde_generate::CodeGeneratorConfig::new("testing".to_string())
        .with_encodings(vec![serde_generate::Encoding::Bincode]);
    let generator = serde_generate::python3::CodeGenerator::new(&config);
    generator.output(&mut source, &registry)?;

    // Write the generated code to a file.
    let schema_path = PathBuf::from(format!(
        "{}/../impostor/messages/messages.py",
        env!("CARGO_MANIFEST_DIR")
    ));
    println!("Writing schema to {:?}", schema_path);
    std::fs::write(schema_path, source).expect("Failed to write schema file");

    Ok(())
}