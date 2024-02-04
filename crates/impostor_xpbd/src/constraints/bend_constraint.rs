use crate::constraints::XPBDConstraint;
use crate::structs::*;
use bevy::prelude::*;
use bevy_prototype_debug_lines::DebugShapes;

pub struct BendConstraint {
    pub pa: ParticleKey,
    pub pb: ParticleKey,
    pub oa: OrientationKey,
    pub ob: OrientationKey,
    pub rest_bend: Quat,
    pub compliance: f32,
}
impl BendConstraint {
    pub fn from_particles(
        body: &SoftBodyData,
        pa: ParticleKey,
        pb: ParticleKey,
        oa: OrientationKey,
        ob: OrientationKey,
    ) -> Self {
        let direction = (body.particles[pb].position - body.particles[pa].position).normalize();
        let qa = body.orientations[oa].quaternion;
        let qb = body.orientations[ob].quaternion;
        

        Self {
            pa,
            pb,
            oa,
            ob,
            rest_bend: Quat::default(),
            compliance: 0.1,
        }
    }
}

impl XPBDConstraint for BendConstraint {
    fn get_compliance(&self) -> f32 {
        self.compliance
    }
    fn solve(&self, body: &mut SoftBodyData, delta_squared: f32) {
        // let alpha = self.compliance / delta_squared;
        // let w = body.particles[self.a].inverse_mass + body.particles[self.b].inverse_mass;
        // if w == 0.0 {
        //     return;
        // }
        // let p1 = body.particles[self.a].position;
        // let p2 = body.particles[self.b].position;
        // let diff = p1 - p2;
        // let distance = diff.length();
        // if distance == 0.0 {
        //     return;
        // }
        // let direction = diff / distance;
        // let delta = p2 - p1;
        // let distance = delta.length();
        // let residual = -(distance - self.rest_length) / (w + alpha);
        // body.particles[self.a].position =
        //     p1 + direction * residual * body.particles[self.a].inverse_mass;
        // body.particles[self.b].position =
        //     p2 - direction * residual * body.particles[self.b].inverse_mass;
    }

    fn debug_draw(&self, body: &SoftBodyData, shapes: &mut DebugShapes) {
        // shapes
        //     .line()
        //     .start(body.particles[self.a].position)
        //     .end(body.particles[self.b].position)
        //     .color(Color::WHITE);
    }
}

#[cfg(test)]
mod tests {}
