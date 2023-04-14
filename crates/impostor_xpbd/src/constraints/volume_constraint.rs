use crate::constraints::XPBDConstraint;
use crate::structs::*;
use bevy::prelude::*;
use bevy_prototype_debug_lines::DebugShapes;
use itertools::Itertools;

pub struct VolumeConstraint {
    pub a: usize,
    pub b: usize,
    pub c: usize,
    pub d: usize,
    pub rest_volume: f32,
    pub compliance: f32,
}
impl VolumeConstraint {
    // The particles should beordered so the volume function returns a positive value
    // The order good if the normal of (b-a) x (c-a) should point to (d-a) (and not in the opposite direction)
    // Another way to validate the order:
    //  - Right thumb points from a to d
    //  - Right index finger points from a to b
    //  - If you rotate your hand around the right thumb to point the index finger to c,
    //  - The order is correct if the rotation is clockwise (when the thumb points towards you)
    pub fn from_particles(
        particles: &mut Vec<Particle>,
        a: usize,
        b: usize,
        c: usize,
        d: usize,
    ) -> Self {
        let mut tetra = Self {
            a,
            b,
            c,
            d,
            rest_volume: 0.0,
            compliance: 0.1,
        };
        tetra.rest_volume = tetra.volume(particles);
        assert!(
            tetra.rest_volume > 0.0,
            "initial rest_volume must be positive (make sure the tetra indices order is correct)"
        );
        if tetra.rest_volume > 0.0 {
            let quarter_inverse_mass = 1.0 / (tetra.rest_volume / 4.0);
            particles[a].inverse_mass += quarter_inverse_mass;
            particles[b].inverse_mass += quarter_inverse_mass;
            particles[c].inverse_mass += quarter_inverse_mass;
            particles[d].inverse_mass += quarter_inverse_mass;
        }
        tetra
    }
    fn volume(&self, particles: &Vec<Particle>) -> f32 {
        let a_position = particles[self.a].position;
        let v1 = particles[self.b].position - a_position;
        let v2 = particles[self.c].position - a_position;
        let v3 = particles[self.d].position - a_position;
        v1.cross(v2).dot(v3) / 6.0
    }
}

impl XPBDConstraint for VolumeConstraint {
    fn get_compliance(&self) -> f32 {
        self.compliance
    }
    fn solve(&self, particles: &mut Vec<Particle>, delta_squared: f32) {
        let alpha = self.compliance / delta_squared;
        let mut w = 0.0;
        // all combinations of [id, ...opposite ids]
        // TODO: I think the order doesnt matter because ||AB x AC||^2 is the same regardless of the order, but i should check if this is true
        let id_views = [
            (self.a, self.b, self.d, self.c),
            (self.b, self.a, self.c, self.d),
            (self.c, self.a, self.d, self.b),
            (self.d, self.a, self.b, self.c),
        ];
        let gradients = id_views
            .iter()
            .map(|(pivot, a, b, c)| {
                let pa = particles[*a].position;
                let pb = particles[*b].position;
                let pc = particles[*c].position;
                let gradient = (pb - pa).cross(pc - pa) / 6.0;
                w += particles[*pivot].inverse_mass * gradient.length_squared();
                gradient
            })
            .collect_vec();
        if w == 0.0 {
            return;
        }
        let volume = self.volume(&particles);
        let residual = -(volume - self.rest_volume) / (w + alpha);
        for (index, gradient) in gradients.into_iter().enumerate() {
            let inverse_mass = particles[id_views[index].0].inverse_mass;
            let push = gradient * residual * inverse_mass;
            particles[id_views[index].0].position += push;
        }
    }

    fn debug_draw(&self, particles: &Vec<Particle>, shapes: &mut DebugShapes) {
        let a = particles[self.a].position;
        let b = particles[self.b].position;
        let c = particles[self.c].position;
        let d = particles[self.d].position;
        let center = (a + b + c + d) / 4.0;
        let scale = 0.7;
        let a = (a - center) * scale + center;
        let b = (b - center) * scale + center;
        let c = (c - center) * scale + center;
        let d = (d - center) * scale + center;

        let color = Color::YELLOW;
        shapes.line().start(a).end(b).color(color);
        shapes.line().start(b).end(c).color(color);
        shapes.line().start(c).end(a).color(color);
        shapes.line().start(a).end(d).color(color);
        shapes.line().start(b).end(d).color(color);
        shapes.line().start(c).end(d).color(color);
    }
}

#[cfg(test)]
mod tests {}
