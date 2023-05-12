use bevy_prototype_debug_lines::DebugShapes;

use super::FigLine;

pub enum DebugFigure {
    Line(FigLine),
}

impl DebugFigure {
    pub fn draw_with_debug_shapes(&self, lines: &mut DebugShapes) {
        match self {
            DebugFigure::Line(line) => line.draw_with_debug_shapes(lines),
        }
    }
}


impl From<FigLine> for DebugFigure {
    fn from(line: FigLine) -> Self {
        Self::Line(line)
    }
}