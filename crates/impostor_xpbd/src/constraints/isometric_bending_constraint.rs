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
    // Q
    pub initial_bending_energy: Mat4,
    pub compliance: f32,
}

//              pb
//            /-^<\
//       e1/--  |  --\e4
//      /--     |     --\
// pd <-        e0       -> pc
//      \--     |     --/
//       e2\--  |  --/e3
//            \>|-/
//              pa
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
            initial_bending_energy: Mat4::default(),
            compliance: 0.1,
        };
        bend.initial_bending_energy = bend.get_bending_energy(particles);
        bend
    }

    // fn get_bend(&self, particles: &Vec<Particle>) -> f32 {
    //     let pa = particles[self.a].position;
    //     let pb = particles[self.b].position;
    //     let pc = particles[self.c].position;
    //     let pd = particles[self.d].position;
    //     let vab = pb - pa;
    //     let vac = pc - pa;
    //     let vad = pd - pa;
    //     let norm1 = vab.cross(vac).normalize();
    //     let norm2 = vab.cross(vad).normalize();
    //     let bend = norm1.dot(norm2).acos();
    //     bend
    // }

    // Q
    fn get_bending_energy(&self, particles: &Vec<Particle>) -> Mat4 {
        let pa = particles[self.a].position;
        let pb = particles[self.b].position;
        let pc = particles[self.c].position;
        let pd = particles[self.d].position;

        let e0 = pb - pa;
        let e1 = pd - pb;
        let e2 = pa - pd;
        let e3 = pc - pa;
        let e4 = pb - pc;

        let area_left = e0.cross(pd - pa).length() * 0.5;
        let area_right = e0.cross(e3).length() * 0.5;
        let area = area_left + area_right;

        let cot = |v0: Vec3, v1: Vec3| -> f32 { v0.dot(v1) / v0.cross(v1).length() };
        // K = (c01 + c04, c02 + c03, −c01 − c02, −c03 − c04)
        let c01 = cot(e0, e1);
        let c02 = cot(e0, e2);
        let c03 = cot(e0, e3);
        let c04 = cot(e0, e4);
        let k = Vec4::new(c01 + c04, c02 + c03, -c01 - c02, -c03 - c04);
        outer_product(k, k) * (3.0 / area)
    }

    fn calculate_constraint_value(&self, particles: &Vec<Particle>) -> f32 {
        let mut sum = 0.0;
        let particles = [
            particles[self.a].position,
            particles[self.b].position,
            particles[self.c].position,
            particles[self.d].position,
        ];
        for i in 0..4 {
            for j in 0..4 {
                sum += self.initial_bending_energy.row(i)[j] * particles[i].dot(particles[j]);
            }
        }
        sum / 2.0
    }

    fn calculate_gradient(&self, particles: &Vec<Particle>) -> Vec<Vec3> {
        let mut gradient = vec![Vec3::default(); 4];
        let particles = [
            particles[self.a].position,
            particles[self.b].position,
            particles[self.c].position,
            particles[self.d].position,
        ];
        for i in 0..4 {
            for j in 0..4 {
                gradient[i] += self.initial_bending_energy.row(i)[j] * particles[j];
            }
        }
        gradient
    }
}

fn outer_product(v0: Vec4, v1: Vec4) -> Mat4 {
    Mat4::from_cols(
        Vec4::new(v0.x * v1.x, v0.x * v1.y, v0.x * v1.z, v0.x * v1.w),
        Vec4::new(v0.y * v1.x, v0.y * v1.y, v0.y * v1.z, v0.y * v1.w),
        Vec4::new(v0.z * v1.x, v0.z * v1.y, v0.z * v1.z, v0.z * v1.w),
        Vec4::new(v0.w * v1.x, v0.w * v1.y, v0.w * v1.z, v0.w * v1.w),
    )
}

impl XPBDConstraint for IsometricBendingConstraint {
    fn get_compliance(&self) -> f32 {
        self.compliance
    }

    fn solve(&self, particles: &mut Vec<Particle>, delta_squared: f32) {
        let alpha = self.compliance / delta_squared;
        // As described  in "Position Based Dynamics" Appendix A, by Müller et al.
        // let pa = particles[self.a].position;
        // let pb = particles[self.b].position;
        // let pc = particles[self.c].position;
        // let pd = particles[self.d].position;
        let ima = particles[self.a].inverse_mass;
        let imb = particles[self.b].inverse_mass;
        let imc = particles[self.c].inverse_mass;
        let imd = particles[self.d].inverse_mass;

        let energy = self.calculate_constraint_value(particles);
        if energy < 1e-12 {
            return;
        }

        let grad = self.calculate_gradient(particles);
        let mut sum_normal_grad = 0.0;
        sum_normal_grad += ima * grad[0].length_squared();
        sum_normal_grad += imb * grad[1].length_squared();
        sum_normal_grad += imc * grad[2].length_squared();
        sum_normal_grad += imd * grad[3].length_squared();

        if sum_normal_grad.abs() < 1e-9 {
            return;
        }

        // compute impulse-based scaling factor
        let s = energy / sum_normal_grad;

        particles[self.a].position += s * ima * grad[0] * alpha;
        particles[self.b].position += s * imb * grad[1] * alpha;
        particles[self.c].position += s * imc * grad[2] * alpha;
        particles[self.d].position += s * imd * grad[3] * alpha;
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
