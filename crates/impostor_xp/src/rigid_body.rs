use bevy::prelude::*;

pub struct RigidBody {
    mass: f32,
    inv_mass: f32,
    // Center of mass
    x: Vec3,
    x_old: Vec3,
    x_last: Vec3,

    x0: Vec3,
    /** center of mass velocity */
    v: Vec3,
    v0: Vec3,
    /** acceleration (by external forces) */
    a: Vec3,

    /** Inertia tensor in the principal axis system: \n
    * After the main axis transformation the inertia tensor is a diagonal matrix.
    * So only three values are required to store the inertia tensor. These values
    * are constant over time.
    */
    inertiaTensor: Vec3,
    /** 3x3 matrix, inertia tensor in world space */
    inertiaTensorW: Mat3,
    /** Inverse inertia tensor in body space */
    inertiaTensorInverse: Vec3,
    /** 3x3 matrix, inverse of the inertia tensor in world space */
    inertiaTensorInverseW: Mat3,
    /** Quaternion that describes the rotation of the body in world space */
    q: Quat,
    lastQ: Quat,
    oldQ: Quat,
    q0: Quat,
    /** Quaternion representing the rotation of the main axis transformation
    that is performed to get a diagonal inertia tensor */
    q_mat: Quat,
    /** Quaternion representing the initial rotation of the geometry */
    q_initial: Quat,
    /** difference of the initial translation and the translation of the main axis transformation */
    x0_mat: Vec3,
    /** rotationMatrix = 3x3 matrix. 
    * Important for the transformation from world in body space and vice versa.
    * When using quaternions the rotation matrix is computed out of the quaternion.
    */
    rot: Mat3,
    /** Angular velocity, defines rotation axis and velocity (magnitude of the vector) */
    omega: Vec3,
    omega0: Vec3,
    /** external torque */
    torque: Vec3,

    restitutionCoeff f32,
    frictionCoeff f32,

    // RigidBodyGeometry geometry;

    // transformation required to transform a point to local space or vice vera
    transformation_R: Mat3,
    transformation_v1: Vec3,
    transformation_v2: Vec3,
    transformation_R_X_v1: Vec3,

}

impl RigidBody {
    pub set_mass(mass: f32) {
        self.mass = mass;
        if mass != 0.0 {
            self.inv_mass = 1.0 / mass;
        } else {
            self.inv_mass = 0.0;
        }
    }
}