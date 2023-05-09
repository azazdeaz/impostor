use std::f32::EPSILON;

use crate::constraints::XPBDConstraint;
use crate::structs::*;
use bevy::prelude::*;
use bevy_prototype_debug_lines::DebugShapes;

pub struct StretchShearConstraint {
    particle_1: ParticleKey,
    particle_2: ParticleKey,
    orientation_1: OrientationKey,
    stretching_stiffness: f32,
    shearing_stiffness_1: f32,
    shearing_stiffness_2: f32,
    rest_length: f32,
}
impl StretchShearConstraint {
    pub fn new(
        body: &SoftBodyData,
        particle_1: ParticleKey,
        particle_2: ParticleKey,
        orientation_1: OrientationKey,
        stretching_stiffness: f32,
        shearing_stiffness_1: f32,
        shearing_stiffness_2: f32,
    ) -> Self {
        let rest_length = body.particles[particle_2]
            .position
            .distance(body.particles[particle_1].position);

        Self {
            particle_1,
            particle_2,
            orientation_1,
            stretching_stiffness,
            shearing_stiffness_1,
            shearing_stiffness_2,
            rest_length,
        }
    }
}

impl XPBDConstraint for StretchShearConstraint {
    fn get_compliance(&self) -> f32 {
        0.0
    }
    fn solve(&self, body: &mut SoftBodyData, delta_squared: f32) {
        // ParticleData &pd = model.getParticles();
        // OrientationData &od = model.getOrientations();

        // const unsigned i1 = m_bodies[0];
        // const unsigned i2 = m_bodies[1];
        // const unsigned iq1 = m_bodies[2];

        // Vector3r &x1 = pd.getPosition(i1);
        // Vector3r &x2 = pd.getPosition(i2);
        // Quaternionr &q1 = od.getQuaternion(iq1);
        // const Real invMass1 = pd.getInvMass(i1);
        // const Real invMass2 = pd.getInvMass(i2);
        // const Real invMassq1 = od.getInvMass(iq1);
        // Vector3r stiffness(m_shearingStiffness1,
        //                     m_shearingStiffness2,
        //                     m_stretchingStiffness);
        let stiffness = Vec3::new(
            self.shearing_stiffness_1,
            self.shearing_stiffness_2,
            self.stretching_stiffness,
        );

        // Vector3r corr1, corr2;
        // Quaternionr corrq1;
        // const bool res = PositionBasedCosseratRods::solve_StretchShearConstraint(
        //     x1, invMass1, x2, invMass2, q1, invMassq1,
        //     stiffness,
        //     m_restLength, corr1, corr2, corrq1);

        // Vector3r d3;	//third director d3 = q0 * e_3 * q0_conjugate
        // d3[0] = static_cast<Real>(2.0) * (q0.x() * q0.z() + q0.w() * q0.y());
        // d3[1] = static_cast<Real>(2.0) * (q0.y() * q0.z() - q0.w() * q0.x());
        // d3[2] = q0.w() * q0.w() - q0.x() * q0.x() - q0.y() * q0.y() + q0.z() * q0.z();

        //third director d3 = q0 * e_3 * q0_conjugate
        let orientation = body.orientations[self.orientation_1].quaternion;
        let d3 = Vec3::new(
            2.0 * (orientation.x * orientation.z + orientation.w * orientation.y),
            2.0 * (orientation.y * orientation.z - orientation.w * orientation.x),
            orientation.w * orientation.w
                - orientation.x * orientation.x
                - orientation.y * orientation.y
                + orientation.z * orientation.z,
        );

        // Vector3r gamma = (p1 - p0) / restLength - d3;
        // gamma /= (invMass1 + invMass0) / restLength + invMassq0 * static_cast<Real>(4.0)*restLength + eps;
        let position_1 = body.particles[self.particle_1].position;
        let position_2 = body.particles[self.particle_2].position;
        let inv_mass_1 = body.particles[self.particle_1].inverse_mass;
        let inv_mass_2 = body.particles[self.particle_2].inverse_mass;
        let inv_mass_quat = body.orientations[self.orientation_1].inverse_mass;
        let mut gamma = (position_2 - position_1) / self.rest_length - d3;
        gamma /= (inv_mass_1 + inv_mass_2) / self.rest_length
            + inv_mass_quat * 4.0 * self.rest_length
            + EPSILON;

        // if (std::abs(stretchingAndShearingKs[0] - stretchingAndShearingKs[1]) < eps && std::abs(stretchingAndShearingKs[0] - stretchingAndShearingKs[2]) < eps)	//all Ks are approx. equal
        //     for (int i = 0; i<3; i++) gamma[i] *= stretchingAndShearingKs[i];
        // else	//diffenent stretching and shearing Ks. Transform diag(Ks[0], Ks[1], Ks[2]) into world space using Ks_w = R(q0) * diag(Ks[0], Ks[1], Ks[2]) * R^T(q0) and multiply it with gamma
        // {
        //     Matrix3r R = q0.toRotationMatrix();
        //     gamma = (R.transpose() * gamma).eval();
        //     for (int i = 0; i<3; i++) gamma[i] *= stretchingAndShearingKs[i];
        //     gamma = (R * gamma).eval();
        // }

        // If all Ks are approx. equal
        if (stiffness[0] - stiffness[1]).abs() < EPSILON
            && (stiffness[0] - stiffness[2]).abs() < EPSILON
        {
            let rot = Mat3::from_quat(orientation);
            gamma = rot.transpose() * gamma;
            gamma *= stiffness;
            gamma = rot * gamma;
        }

        // corr0 = invMass0 * gamma;
        // corr1 = -invMass1 * gamma;
        let corr_1 = inv_mass_1 * gamma;
        let corr_2 = -inv_mass_2 * gamma;

        // Quaternionr q_e_3_bar(q0.z(), -q0.y(), q0.x(), -q0.w());	//compute q*e_3.conjugate (cheaper than quaternion product)
        // corrq0 = Quaternionr(0.0, gamma.x(), gamma.y(), gamma.z()) * q_e_3_bar;
        // corrq0.coeffs() *= static_cast<Real>(2.0) * invMassq0 * restLength;

        let corr_quat = Quat::from_xyzw(0.0, gamma.x, gamma.y, gamma.z).normalize()
            * Quat::from_xyzw(orientation.z, -orientation.y, orientation.x, -orientation.w).normalize();
        let corr_quat = corr_quat * (2.0 * inv_mass_quat * self.rest_length);

        // if (res)
        // {
        //     if (invMass1 != 0.0)
        //         x1 += corr1;
        //     if (invMass2 != 0.0)
        //         x2 += corr2;
        //     if (invMassq1 != 0.0)
        //     {
        //         q1.coeffs() += corrq1.coeffs();
        //         q1.normalize();
        //     }
        // }
        if inv_mass_1 != 0.0 {
            body.particles[self.particle_1].position += corr_1;
        }
        if inv_mass_2 != 0.0 {
            body.particles[self.particle_2].position += corr_2;
        }
        if inv_mass_quat != 0.0 {
            body.orientations[self.orientation_1].quaternion =
                (body.orientations[self.orientation_1].quaternion + corr_quat).normalize();
        }
        // return res;
    }

    fn debug_draw(&self, body: &SoftBodyData, shapes: &mut DebugShapes) {
        shapes
            .line()
            .start(body.particles[self.particle_1].position)
            .end(body.particles[self.particle_2].position)
            .color(Color::WHITE);
    }
}

#[cfg(test)]
mod tests {}
