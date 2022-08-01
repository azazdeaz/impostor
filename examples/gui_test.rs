use bevy::prelude::*;
// use bevy_inspector_egui::{InspectorPlugin, Inspectable};

// #[derive(Inspectable, Default)]
struct Data {
    should_render: bool,
    text: String,
    // #[inspectable(min = 42.0, max = 100.0)]
    size: f32,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // .add_plugin(InspectorPlugin::<Data>::new())
        .run();
}