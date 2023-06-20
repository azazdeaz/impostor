use bevy::prelude::*;

use super::{Orientation, Particle};

// #include "TimeIntegration.h"

// using namespace PBD;

// // ----------------------------------------------------------------------------------------------
// void TimeIntegration::semiImplicitEuler(
// 	const Real h,
// 	const Real mass,
// 	Vector3r &position,
// 	Vector3r &velocity,
// 	const Vector3r &acceleration)
// {
// 	if (mass != 0.0)
// 	{
// 		velocity += acceleration * h;
// 		position += velocity * h;
// 	}
// }

pub fn semi_implicit_euler(delta: f32, acceleration: Vec3, particle: &mut Particle) {
    if particle.inverse_mass != 0.0 {
        particle.velocity += acceleration * delta;
        particle.position += particle.velocity * delta;
    }
}

// // ----------------------------------------------------------------------------------------------
// void TimeIntegration::semiImplicitEulerRotation(
// 	const Real h,
// 	const Real mass,
// 	const Matrix3r& invertiaW,
// 	const Matrix3r &invInertiaW,
// 	Quaternionr &rotation,
// 	Vector3r &angularVelocity,
// 	const Vector3r &torque)
// {
// 	if (mass != 0.0)
// 	{
// 		angularVelocity += h * invInertiaW * (torque - (angularVelocity.cross(invertiaW * angularVelocity)));

// 		Quaternionr angVelQ(0.0, angularVelocity[0], angularVelocity[1], angularVelocity[2]);
// 		rotation.coeffs() += h * 0.5 * (angVelQ * rotation).coeffs();
// 		rotation.normalize();
// 	}
// }

pub fn semi_implicit_euler_rotation(delta: f32, torque: Vec3, orientation: &mut Orientation) {
    if orientation.inverse_mass == 0.0 {
        return;
    }

    let interia_w = Mat3::IDENTITY * (1.0 / orientation.inverse_mass);
    let inv_inertia_w = Mat3::IDENTITY * orientation.inverse_mass;

    orientation.angular_velocity += delta
        * inv_inertia_w
        * (torque
            - (orientation
                .angular_velocity
                .cross(interia_w * orientation.angular_velocity)));
    let ang_vel_q = Quat::from_xyzw(
        orientation.angular_velocity.x,
        orientation.angular_velocity.y,
        orientation.angular_velocity.z,
        0.0,
    );
    // let ang_vel_q = Quat::from_scaled_axis(orientation.angular_velocity);
    // add to quaternion
    orientation.quaternion = orientation.quaternion + (ang_vel_q * orientation.quaternion) * 0.5 * delta;
    orientation.quaternion = orientation.quaternion.normalize();

}

// // ----------------------------------------------------------------------------------------------
// void TimeIntegration::velocityUpdateFirstOrder(
// 	const Real h,
// 	const Real mass,
// 	const Vector3r &position,
// 	const Vector3r &oldPosition,
// 	Vector3r &velocity)
// {
// 	if (mass != 0.0)
// 		velocity = (1.0 / h) * (position - oldPosition);
// }

// // ----------------------------------------------------------------------------------------------
// void TimeIntegration::angularVelocityUpdateFirstOrder(
// 	const Real h,
// 	const Real mass,
// 	const Quaternionr &rotation,
// 	const Quaternionr &oldRotation,
// 	Vector3r &angularVelocity)
// {
// 	if (mass != 0.0)
// 	{
// 		const Quaternionr relRot = (rotation * oldRotation.conjugate());
// 		angularVelocity = relRot.vec() *(2.0 / h);
// 	}
// }

pub fn angular_velocity_update_first_order(delta: f32, orientation: &mut Orientation) {
    if orientation.inverse_mass == 0.0 {
        return;
    }
    let relative_rotation = orientation.quaternion * orientation.old_quaternion.conjugate();
    orientation.angular_velocity = relative_rotation.xyz() * (2.0 / delta);
}

// // ----------------------------------------------------------------------------------------------
// void TimeIntegration::velocityUpdateSecondOrder(
// 	const Real h,
// 	const Real mass,
// 	const Vector3r &position,
// 	const Vector3r &oldPosition,
// 	const Vector3r &positionOfLastStep,
// 	Vector3r &velocity)
// {
// 	if (mass != 0.0)
// 		velocity = (1.0 / h) * (1.5*position - 2.0*oldPosition + 0.5*positionOfLastStep);
// }

// // ----------------------------------------------------------------------------------------------
// void TimeIntegration::angularVelocityUpdateSecondOrder(
// 	const Real h,
// 	const Real mass,
// 	const Quaternionr &rotation,
// 	const Quaternionr &oldRotation,
// 	const Quaternionr &rotationOfLastStep,
// 	Vector3r &angularVelocity)
// {
// 	// ToDo: is still first order
// 	if (mass != 0.0)
// 	{
// 		const Quaternionr relRot = (rotation * oldRotation.conjugate());
// 		angularVelocity = relRot.vec() *(2.0 / h);
// 	}
// }
