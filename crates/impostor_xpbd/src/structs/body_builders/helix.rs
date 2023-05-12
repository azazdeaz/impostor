use crate::{structs::{SoftBody, Particle, Orientation}, constraints::{StretchShearConstraint, BendTwistConstraint}};
use bevy::prelude::*;

impl SoftBody {
    pub fn build_helix(
        position: Vec3,
        orientation: Quat,
        radius: f32,
        height: f32,
        total_angle: f32,
        n_points: usize,
    ) -> Self {
        let stretching_stiffness = 1.0;
        let shearing_stiffness_x = 1.0;
        let shearing_stiffness_y = 1.0;
        let bending_stiffness_x = 0.5;
        let bending_stiffness_y = 0.5;
        let twisting_stiffness = 0.5;

        let mut body = SoftBody { ..Default::default() };
        // void createHelix(const Vector3r &position, const Matrix3r &orientation, Real radius, Real height, Real totalAngle, int nPoints)
        // {
        // int nQuaternions = nPoints - 1;
        // vector<Vector3r> points(nPoints);
        // vector<Quaternionr> quaternions(nQuaternions);
        let n_quaternions = n_points - 1;
        let mut points = Vec::with_capacity(n_points);
        let mut quaternions = Vec::with_capacity(n_quaternions);
        let mut particles = Vec::with_capacity(n_points);
        let mut orientations = Vec::with_capacity(n_quaternions);
        
        // //init particles
        // for (int i = 0; i<nPoints; i++)   
        // {
        //     points[i].x() = radius * std::cos(totalAngle / ((Real)nPoints) * (Real)i);
        //     points[i].y() = radius * std::sin(totalAngle / ((Real)nPoints) * (Real)i);
        //     points[i].z() = height / ((Real)nPoints) * (Real)i;

        //     points[i] = orientation * points[i] + position;
        // }

        //init particles
        for i in 0..n_points {
            let angle = total_angle / (n_points as f32) * (i as f32);
            let x = radius * angle.cos();
            let z = radius * angle.sin();
            let y = 0.5 - height / (n_points as f32) * (i as f32);
            let mut point = Vec3::new(x, y, z);
            point = orientation * point + position;
            points.push(point);
            println!("point: {}", point);
        }

        // //init quaternions
        // Vector3r from(0, 0, 1);
        // for(int i=0; i<nQuaternions; i++)	
        // {
        //     Vector3r to = (points[i + 1] - points[i]).normalized();
        //     Quaternionr dq = Quaternionr::FromTwoVectors(from, to);
        //     if(i == 0) quaternions[i] = dq;
        //     else quaternions[i] = dq * quaternions[i - 1];
        //     from = to;
        // }

        // init quaternions
        let mut from = Vec3::Y;
        for i in 0..n_quaternions {
            let to = (points[i + 1] - points[i]).normalize();
            let mut dq = Quat::from_rotation_arc(from, to);
            if i == 0 {
                quaternions.push(dq);
            } else {
                dq = dq * quaternions[i - 1];
                quaternions.push(dq);
            }
            from = to;
        }

        // vector<unsigned int> indices(2 * nPoints - 1);
        // vector<unsigned int> indicesQuaternions(nQuaternions);

        // for(int i=0; i < nPoints -1; i++)
        // {
        //     indices[2 * i] = i;
        //     indices[2 * i + 1] = i + 1;
        // }

        // for (int i = 0; i < nQuaternions; i++)
        // {
        //     indicesQuaternions[i] = i;
        // }

        for point in points {
            particles.push(body.data.particles.insert(Particle::from_position(point)));
        }
        for quaternion in quaternions {
            orientations.push(body.data.orientations.insert(Orientation::from_quaternion(quaternion)));
        }


        // SimulationModel *model = Simulation::getCurrent()->getModel();
        // model->addLineModel(nPoints, nQuaternions, &points[0], &quaternions[0], &indices[0], &indicesQuaternions[0]);

        // ParticleData &pd = model->getParticles();
        // const int nPointsTotal = pd.getNumberOfParticles();
        // for (int i = nPointsTotal - 1; i > nPointsTotal - nPoints; i--)
        // {
        //     pd.setMass(i, 1.0);
        // }

        // // Set mass of points to zero => make it static
        // pd.setMass(nPointsTotal - nPoints, 0.0);

        body.data.particles[particles[0]].inverse_mass = 0.0;

        // OrientationData &od = model->getOrientations();
        // const unsigned int nQuaternionsTotal = od.getNumberOfQuaternions();
        // for(unsigned int i = nQuaternionsTotal - 1; i > nQuaternionsTotal - nQuaternions; i--)
        // {
        //     od.setMass(i, 1.0);
        // }
        
        // // Set mass of quaternions to zero => make it static
        // od.setMass(nQuaternionsTotal - nQuaternions, 0.0);

        body.data.orientations[orientations[0]].inverse_mass = 0.0;

        // // init constraints
        // const size_t rodNumber = model->getLineModels().size() - 1;
        // const unsigned int offset = model->getLineModels()[rodNumber]->getIndexOffset();
        // const unsigned int offsetQuaternions = model->getLineModels()[rodNumber]->getIndexOffsetQuaternions();
        // const size_t nEdges = model->getLineModels()[rodNumber]->getEdges().size();
        // const LineModel::Edges &edges = model->getLineModels()[rodNumber]->getEdges();
            
        // //stretchShear constraints
        // for(unsigned int i=0; i < nEdges; i++)
        // {
        //     const unsigned int v1 = edges[i].m_vert[0] + offset;
        //     const unsigned int v2 = edges[i].m_vert[1] + offset;
        //     const unsigned int q1 = edges[i].m_quat + offsetQuaternions;
        //     model->addStretchShearConstraint(v1, v2, q1, model->getRodStretchingStiffness(), model->getRodShearingStiffnessX(), model->getRodShearingStiffnessY());
        // }

        //stretchShear constraints
        for i in 0..n_quaternions {
            let v1 = particles[i];
            let v2 = particles[i + 1];
            let q1 = orientations[i];
            let constraint = StretchShearConstraint::new(
                &body.data,
                v1,
                v2,
                q1,
                stretching_stiffness,
                shearing_stiffness_x,
                shearing_stiffness_y,
            );
            body.constraints.push(Box::new(constraint));
        }

        // //bendTwist constraints
        // for(unsigned int i=0; i < nEdges - 1; i++)
        // {
        //     const unsigned int q1 = edges[i].m_quat + offsetQuaternions;
        //     const unsigned int q2 = edges[i + 1].m_quat + offsetQuaternions;
        //     model->addBendTwistConstraint(q1, q2, model->getRodTwistingStiffness(), model->getRodBendingStiffnessX(), model->getRodBendingStiffnessY());
        // }
        
        // TODO what is offsetQuaternions?
        for i in 0..n_quaternions-1 {
            let q1 = orientations[i];
            let q2 = orientations[i + 1];
            let constraint = BendTwistConstraint::new(
                &body.data,
                q1,
                q2,
                twisting_stiffness,
                bending_stiffness_x,
                bending_stiffness_y,
            );
            body.constraints.push(Box::new(constraint));
        }
        body
    }
}