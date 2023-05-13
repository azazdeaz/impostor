use std::f32::EPSILON;

use crate::constraints::XPBDConstraint;
use crate::structs::*;
use bevy::prelude::*;
use bevy_prototype_debug_lines::DebugShapes;
use bevy_prototype_debug_lines::shapes::Shape;

/**
 * Implementation is based on:
 *  - https://github.com/InteractiveComputerGraphics/PositionBasedDynamics/blob/d2d1d7b48408e1c0e05e5cad5b3b0fa78da3b1eb/Simulation/Constraints.cpp#L2391-L2454
 *  - https://github.com/InteractiveComputerGraphics/PositionBasedDynamics/blob/abd45249c2763cdd07497c314344f7749042e199/PositionBasedDynamics/PositionBasedElasticRods.cpp#LL57C28-L79C2
 */

pub struct BendTwistConstraint {
    orientation_1: OrientationKey,
    orientation_2: OrientationKey,
    twisting_stiffness: f32,
    bending_stiffness_1: f32,
    bending_stiffness_2: f32,
    rest_darboux_vector: Quat,
}
impl BendTwistConstraint {
    pub fn new(
        body: &SoftBodyData,
        orientation_1: OrientationKey,
        orientation_2: OrientationKey,
        twisting_stiffness: f32,
        bending_stiffness_1: f32,
        bending_stiffness_2: f32,
    ) -> Self {
        // m_restDarbouxVector = q1_0.conjugate() * q2_0;
        // Quaternionr omega_plus, omega_minus;
        // omega_plus.coeffs() = m_restDarbouxVector.coeffs() + Quaternionr(1, 0, 0, 0).coeffs();
        // omega_minus.coeffs() = m_restDarbouxVector.coeffs() - Quaternionr(1, 0, 0, 0).coeffs();
        // if (omega_minus.squaredNorm() > omega_plus.squaredNorm())
        //     m_restDarbouxVector.coeffs() *= -1.0;
        let q1 = body.orientations[orientation_1].quaternion;
        let q2 = body.orientations[orientation_2].quaternion;
        let mut rest_darboux_vector = q1.conjugate() * q2;
        let omega_plus = rest_darboux_vector + Quat::from_xyzw(1.0, 0.0, 0.0, 0.0);
        let omega_minus = rest_darboux_vector - Quat::from_xyzw(1.0, 0.0, 0.0, 0.0);
        let rest_darboux_vector = if omega_minus.length_squared() > omega_plus.length_squared() {
            println!("flip rest_darboux_vector;");  
            rest_darboux_vector * -1.0
        } else {
            rest_darboux_vector
        };

        Self {
            orientation_1,
            orientation_2,
            twisting_stiffness,
            bending_stiffness_1,
            bending_stiffness_2,
            rest_darboux_vector,
        }
    }
}

impl XPBDConstraint for BendTwistConstraint {
    fn get_compliance(&self) -> f32 {
        0.0
    }
    fn solve(&self, body: &mut SoftBodyData, delta_squared: f32) {
        // OrientationData &od = model.getOrientations();

        // const unsigned i1 = m_bodies[0];
        // const unsigned i2 = m_bodies[1];

        // Quaternionr &q1 = od.getQuaternion(i1);
        // Quaternionr &q2 = od.getQuaternion(i2);
        // const Real invMass1 = od.getInvMass(i1);
        // const Real invMass2 = od.getInvMass(i2);
        // Vector3r stiffness(m_bendingStiffness1,
        //                 m_bendingStiffness2,
        //                 m_twistingStiffness);
        let q1 = body.orientations[self.orientation_1].quaternion;
        let q2 = body.orientations[self.orientation_2].quaternion;
        let inv_mass_1 = body.orientations[self.orientation_1].inverse_mass;
        let inv_mass_2 = body.orientations[self.orientation_2].inverse_mass;
        let stiffness = Vec3::new(
            self.bending_stiffness_1,
            self.bending_stiffness_2,
            self.twisting_stiffness,
        );

        // Quaternionr corr1, corr2;
        // const bool res = PositionBasedCosseratRods::solve_BendTwistConstraint(
        //     q1, invMass1, q2, invMass2,
        //     stiffness,
        //     m_restDarbouxVector, corr1, corr2);
        // Quaternionr omega = q0.conjugate() * q1;   //darboux vector

        let mut omega = q1.conjugate() * q2; //darboux vector
        println!("\n\nrest_darboux_vector: {}", self.rest_darboux_vector);
        println!("omega: {}", omega);
        // Quaternionr omega_plus;
        // omega_plus.coeffs() = omega.coeffs() + restDarbouxVector.coeffs();     //delta Omega with -Omega_0
        // omega.coeffs() = omega.coeffs() - restDarbouxVector.coeffs();                 //delta Omega with + omega_0
        // if (omega.squaredNorm() > omega_plus.squaredNorm()) omega = omega_plus;

        let omega_plus = omega + self.rest_darboux_vector; //delta Omega with -Omega_0
        println!("delta omega_plus: {}", omega_plus);
        omega = omega - self.rest_darboux_vector; //delta Omega with + omega_0
        println!("delta omega: {}", omega);
        if omega.length_squared() > omega_plus.length_squared() {
            println!("flip omega");
            omega = omega_plus;
        }

        if omega.length_squared() < EPSILON {
            println!("omega length squared < EPSILON");
            return;
        }

        // for (int i = 0; i < 3; i++) omega.coeffs()[i] *= bendingAndTwistingKs[i] / (invMassq0 + invMassq1 + static_cast<Real>(1.0e-6));
        // omega.w() = 0.0;    //discrete Darboux vector does not have vanishing scalar part

        let inv_mass_sum = (inv_mass_1 + inv_mass_2 + EPSILON);
        omega.x *= stiffness.x / inv_mass_sum;
        omega.y *= stiffness.y / inv_mass_sum;
        omega.z *= stiffness.z / inv_mass_sum;
        omega.w = 0.0; //discrete Darboux vector does not have vanishing scalar part
        // omega = omega.normalize();

        // corrq0 = q1 * omega;
        // corrq1 = q0 * omega;
        // corrq0.coeffs() *= invMassq0;
        // corrq1.coeffs() *= -invMassq1;
        let corr_1 = (q2 * omega) * inv_mass_1;
        let corr_2 = (q1 * omega) * -inv_mass_2;

        // if (res)
        // {
        //     if (invMass1 != 0.0)
        //     {
        //         q1.coeffs() += corr1.coeffs();
        //         q1.normalize();
        //     }

        //     if (invMass2 != 0.0)
        //     {
        //         q2.coeffs() += corr2.coeffs();
        //         q2.normalize();
        //     }
        // }
        // return res;
        if inv_mass_1 != 0.0 {
            body.orientations[self.orientation_1].quaternion =
                (body.orientations[self.orientation_1].quaternion + corr_1).normalize();
        }
        if inv_mass_2 != 0.0 {
            body.orientations[self.orientation_2].quaternion =
                (body.orientations[self.orientation_2].quaternion + corr_2).normalize();
        }
    }

    fn debug_draw(&self, body: &SoftBodyData, shapes: &mut DebugShapes) {
        // shapes
        //     .line()
        //     .start(body.particles[self.particle_1].position)
        //     .end(body.particles[self.particle_2].position)
        //     .color(Color::WHITE);
    }
}

#[cfg(test)]
mod tests {}
