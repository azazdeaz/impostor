use bevy::prelude::*;
use bevy_egui::{egui, EguiContext, EguiPlugin};


pub fn schemas_ui(mut egui_context: ResMut<EguiContext>) {
    egui::Window::new("Hello").show(egui_context.ctx_mut(), |ui| {
        ui.label("world");
    });
}



        // // init egui
        // if !app.world.contains_resource::<EguiContext>() {
        //     app.add_plugin(EguiPlugin);
        // }