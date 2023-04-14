use crate::constraints::XPBDConstraint;
use crate::structs::*;
use bevy::prelude::*;
use bevy_prototype_debug_lines::{DebugLinesPlugin, DebugShapes};
use itertools::izip;

pub struct IsometricBendingConstraint {
    // pivot particle (shared by both triangles)
    pub a: usize,
    // the other particle shared by both triangles
    pub b: usize,
    // head of the first triangle
    pub c: usize,
    // head of the second triangle
    pub d: usize,
    pub rest_bend: f32,
    pub compliance: f32,
}

impl IsometricBendingConstraint {
    pub fn from_particles(
        particles: &Vec<Particle>,
        a: usize,
        b: usize,
        c: usize,
        d: usize,
    ) -> Self {
        let mut bend = Self {
            a,
            b,
            c,
            d,
            rest_bend: 0.0,
            compliance: 0.1,
        };
        bend.rest_bend = bend.get_bend(particles);
        bend
    }

    pub fn get_bend(&self, particles: &Vec<Particle>) -> f32 {
        let pa = particles[self.a].position;
        let pb = particles[self.b].position;
        let pc = particles[self.c].position;
        let pd = particles[self.d].position;
        let vab = pb - pa;
        let vac = pc - pa;
        let vad = pd - pa;
        let norm1 = vab.cross(vac).normalize();
        let norm2 = vab.cross(vad).normalize();
        let bend = norm1.dot(norm2).acos();
        bend
    }
}

// TODO debug, something is obvously not right.
impl XPBDConstraint for IsometricBendingConstraint {
    fn get_compliance(&self) -> f32 {
        self.compliance
    }
    fn solve(&self, particles: &mut Vec<Particle>, delta_squared: f32) {
        let alpha = self.compliance / delta_squared;
        // As described  in "Position Based Dynamics" Appendix A, by Müller et al.
        let pa = particles[self.a].position;
        let pb = particles[self.b].position;
        let pc = particles[self.c].position;
        let pd = particles[self.d].position;
        let wa = particles[self.a].inverse_mass;
        let wb = particles[self.b].inverse_mass;
        let wc = particles[self.c].inverse_mass;
        let wd = particles[self.d].inverse_mass;
        let vab = pb - pa;
        let vac = pc - pa;
        let vad = pd - pa;

        let outer = |a: Vec3, b: Vec3| -> Mat3 { Mat3::from_cols(a * b.x, a * b.y, a * b.z) };

        // Transposed cross product matrix
        let tcpm = |vec: Vec3| -> Mat3 {
            Mat3::from_cols(
                Vec3::new(0.0, vec.z, -vec.y),
                Vec3::new(-vec.z, 0.0, vec.x),
                Vec3::new(vec.y, -vec.x, 0.0),
            )
        };

        // Gradient of the normalized cross product
        let norm_grads = |p1: Vec3, p2: Vec3| {
            let normal = p1.cross(p2);
            let normal_length = normal.length();
            let normal = normal / normal_length;
            let inverse_normal_length = 1.0 / normal_length;
            let grad1 = inverse_normal_length * (-tcpm(p2) + outer(normal, normal.cross(p2)));
            let grad2 = inverse_normal_length * (-tcpm(p1) + outer(normal, normal.cross(p1)));
            (normal, grad1, grad2)
        };

        let (n1, dn1_dpb, dn1_dpc) = norm_grads(vab, vac);
        let (n2, dn2_dpb, dn2_dpd) = norm_grads(vab, vad);

        // let c1 = (pa + pb + pc) / 3.0;
        // let c2 = (pa + pb + pd) / 3.0;
        // debug.line().start(c1).end(c1 + n1).color(Color::RED);
        // debug.line().start(c2).end(c2 + n2).color(Color::RED);

        let d = n1.dot(n2);
        let d_arccos_dx = -1.0 / (1.0 - d * d).sqrt();
        let delta_c = d_arccos_dx * (dn1_dpc.transpose() * n2);
        let delta_d = d_arccos_dx * (dn2_dpd.transpose() * n1);
        let delta_b = d_arccos_dx * (dn1_dpb.transpose() * n2 + dn2_dpb.transpose() * n1);
        let delta_a = -delta_b - delta_c - delta_d;

        let ids = [self.a, self.b, self.c, self.d];
        let ws = [wa, wb, wc, wd];
        let deltas = [delta_a, delta_b, delta_c, delta_d];
        let divisor = izip!(&ws, &deltas)
            .map(|(w, delta)| w * delta.length_squared())
            .sum::<f32>();
        for (particle, &delta, w) in izip!(&ids, &deltas, &ws) {
            let constraint_violation: Vec3 =
                -((w * (1.0 - d * d).sqrt() * (d.acos() - self.rest_bend)) / divisor) * delta;
            // TODO: find out how alpha works
            let push = constraint_violation / (w + alpha);
            println!(
                "particle {} constraint_violation: {:?} alpha: {:?}",
                particle, constraint_violation, alpha
            );

            let particle = &mut particles[*particle];

            // debug
            //     .line()
            //     .start(particle.position)
            //     .end(particle.position + push)
            //     .color(Color::LIME_GREEN);

            particle.position += push;
        }
    }

    fn debug_draw(&self, particles: &Vec<Particle>, shapes: &mut DebugShapes) {
        let a = particles[self.a].position;
        let b = particles[self.b].position;
        let c = particles[self.c].position;
        let d = particles[self.d].position;
        let center = (a + b + c + d) / 4.0;
        let scale = 1.0;
        let a = (a - center) * scale + center;
        let b = (b - center) * scale + center;
        let c = (c - center) * scale + center;
        let d = (d - center) * scale + center;

        let color_base = Color::BLUE;
        let color_up = Color::ALICE_BLUE;
        let color_down = Color::MIDNIGHT_BLUE;
        shapes.line().start(a).end(b).color(color_base);
        shapes.line().start(a).end(c).color(color_down);
        shapes.line().start(a).end(d).color(color_up);
        shapes.line().start(b).end(c).color(color_down);
        shapes.line().start(b).end(d).color(color_up);
    }
}

#[cfg(test)]
mod tests {}
