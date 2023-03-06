use bevy::prelude::*;

struct StemBuilder {
    steps: Vec<Transform>,
}

struct Ring {
    transform: Transform,
    radius: f32,
    joint_indices: [u16; 4],
    joint_weights: [f32; 4],
    from_start: f32,
}

impl StemBuilder {
    fn gen_rings(&self) {
        assert!(self.steps.len() > 1, "At least two steps needed (for start and end");
        

        let mut rings = vec![Ring {
            transform: steps[0].clone(),
            radius: axis_joints[0].2,
            joint_indices: [0, 0, 0, 0],
            joint_weights: [1.0, 0.0, 0.0, 0.0],
            from_start: 0.0,
        }];

        // full distances up until each joints
        let distances = axis_joints
            .windows(2)
            .map(|w| w[0].1.translation.distance(w[1].1.translation))
            .collect_vec();
        let radii = axis_joints
            .iter()
            .map(|(_, _, radius)| radius)
            .collect_vec();

        let full_distances = distances.iter().fold(Vec::<f32>::new(), |mut acc, axe| {
            let last = acc.last().unwrap_or(&0.0);
            acc.push(*last + axe);
            return acc;
        });
        let full_length = *full_distances.last().unwrap();
        println!(
            "Distances={:?}\nFull distances={:?}\nFull{:?}",
            distances, full_distances, full_length
        );

        let mut state = 0.0;
        while state < full_length {
            let i = full_distances
                .iter()
                .position(|distance| distance > &state)
                .unwrap();
            // TODO there is probably a simpler way to use lerp
            let a = EaseValue(axis_joints[i].1.clone());
            let b = EaseValue(axis_joints[i + 1].1.clone());

            // 0..1.0 place of the ring between the two joints
            let start = if i == 0 { 0.0 } else { full_distances[i - 1] };
            let joint_p = (state - start) / distances[i];
            let w0 = 1.0 - joint_p;
            let w1 = joint_p;

            rings.push(Ring {
                transform: a.lerp(&b, &joint_p).0,
                radius: radii[i].lerp(radii[i + 1], &joint_p),
                joint_indices: [i as u16, i as u16 + 1, 0, 0],
                joint_weights: [w0, w1, 0.0, 0.0],
                from_start: state,
            });

            // Increment state
            state += vertical_resolution;
        }

        rings.push(Ring {
            transform: axis_joints.last().unwrap().1.clone(),
            radius: 0.0,
            joint_indices: [axis_joints.len() as u16 - 1, 0, 0, 0],
            joint_weights: [1.0, 0.0, 0.0, 0.0],
            from_start: full_length,
        });
    }
    pub fn gen(&self) -> Mesh {
        todo!();
    }
}