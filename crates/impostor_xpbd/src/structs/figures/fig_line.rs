use bevy::prelude::{Vec3, Color};
use bevy_prototype_debug_lines::DebugShapes;


pub struct FigLine {
    start: Vec3,
    end: Vec3,
    start_color: Color,
    end_color: Color,
}

impl FigLine {
    pub fn new() -> Self {
        Self {
            start: Vec3::ZERO,
            end: Vec3::X,
            start_color: Color::WHITE,
            end_color: Color::WHITE,
        }
    }

    pub fn start(mut self, start: Vec3) -> Self {
        self.start = start;
        self
    }

    pub fn end(mut self, end: Vec3) -> Self {
        self.end = end;
        self
    }

    pub fn start_travel(mut self, start: Vec3, travel: Vec3) -> Self {
        self.start = start;
        self.end = start + travel;
        self
    }

    pub fn color(mut self, color: Color) -> Self {
        self.start_color = color;
        self.end_color = color;
        self
    }

    pub fn draw_with_debug_shapes(&self, shapes: &mut DebugShapes) {
        shapes.line()
            .start(self.start)
            .end(self.end)
            .gradient(self.start_color, self.end_color);
    }
}