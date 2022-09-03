use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use impostor_schemas::schemas;

#[derive(Clone)]
pub struct TeleopDiffDriveConfig {
    left_tag: schemas::Tag,
    right_tag: schemas::Tag,
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}

fn keyboard_input_system(
    keyboard_input: Res<Input<KeyCode>>,
    mut joints: Query<(&mut ImpulseJoint, &schemas::Tag)>,
    config: Res<TeleopDiffDriveConfig>,
) {
    let mut speed = (0., 0.);
    if keyboard_input.pressed(KeyCode::Up) {
        speed = (12., 12.);
    } else if keyboard_input.pressed(KeyCode::Down) {
        speed = (-12., -12.);
    } else if keyboard_input.pressed(KeyCode::Left) {
        speed = (-12., 12.);
    } else if keyboard_input.pressed(KeyCode::Right) {
        speed = (12., -12.);
    }

    for (mut joint, tag) in joints.iter_mut() {
        if let Some(joint) = joint.data.as_revolute_mut() {
            if *tag == config.left_tag {
                joint.set_motor_velocity(speed.0, 10.);
            } else if *tag == config.right_tag {
                joint.set_motor_velocity(speed.1, 10.);
            }
        }
    }
}

pub struct TeleopPlugin {
    config: TeleopDiffDriveConfig,
}

impl TeleopPlugin {
    pub fn new(left_tag: schemas::Tag, right_tag: schemas::Tag) -> Self {
        Self {
            config: TeleopDiffDriveConfig {
                left_tag,
                right_tag,
            },
        }
    }
}

impl Plugin for TeleopPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(self.config.clone())
            .add_system(keyboard_input_system);
    }
}
