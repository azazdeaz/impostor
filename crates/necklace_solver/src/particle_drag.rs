use bevy::prelude::*;

use crate::Point;

pub struct DragParticlePlugin;
impl Plugin for DragParticlePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, drag_particles);
    }
}

#[derive(Debug, Clone, Copy)]
struct DragState {
    id: Entity,
    grab_distance: f32,
    // grab_inverse_mass: InverseMass,
}

#[derive(Component, Default)]
pub struct DragParticle {
    pub enabled: bool,
}
impl DragParticle {
    pub fn enabled() -> Self {
        Self { enabled: true }
    }
}

fn drag_particles(
    camera_query: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
    mut particles: Query<(Entity, &mut Point, &DragParticle)>,
    buttons: Res<Input<MouseButton>>,
    windows: Query<&Window>,
    mut gizmos: Gizmos,
    mut drag_state: Local<Option<DragState>>,
) {
    let Some(cursor_position) = windows.single().cursor_position() else {
        return;
    };
    let Ok((camera, camera_transform)) = camera_query.get_single() else {
        warn!("No camera");
        return;
    };
    // Calculate a ray pointing from the camera into the world based on the cursor's position.
    let Some(ray) = camera.viewport_to_world(camera_transform, cursor_position) else {
        warn!("No ray");
        return;
    };

    let mut update_drag_state =
        |new_drag_state: Option<DragState>,
         particles: &mut Query<'_, '_, (Entity, &mut Point, &DragParticle)>| {
            // if let Some(state) = drag_state.take() {
            //     if let Ok((mut body, _)) = particles.get_mut(state.id) {
            //         // Reset the inverse_mass of the released body
            //         body.inverse_mass.0 = state.grab_inverse_mass.0;
            //     }
            // }
            *drag_state = new_drag_state;
            // if let Some(state) = new_drag_state {
            //     if let Ok((mut body, _)) = particles.get_mut(state.id) {
            //         // Set inverse_mass to 0 to make the dragged body immovable
            //         body.inverse_mass.0 = 0.0;
            //     }
            // }
        };

    if buttons.just_pressed(MouseButton::Left) {
        let mut closest: Option<(DragState, f32)> = None;
        for (point_entity, point) in particles.into_iter().filter_map(
            |(entity, point, drag)| {
                if drag.enabled {
                    Some((entity, point))
                } else {
                    None
                }
            },
        ) {
            // calculate the particle distance from the ray
            let particle_from_origin = point.0 - ray.origin;
            let closest_point_on_ray = ray.direction * particle_from_origin.dot(ray.direction);
            let distance = (particle_from_origin - closest_point_on_ray).length();
            if distance < 0.5 && (closest.is_none() || distance < closest.unwrap().1) {
                closest = Some((
                    DragState {
                        id: point_entity,
                        grab_distance: point.0.distance(ray.origin),
                        // grab_inverse_mass: *point.inverse_mass,
                    },
                    distance,
                ));
            }
        }
        if let Some((info, _)) = closest.take() {
            update_drag_state(Some(info), &mut particles);
        }
    } else if buttons.just_released(MouseButton::Left) {
        update_drag_state(None, &mut particles);
    }

    if let Some(info) = &*drag_state {
        if let Ok((_, mut point, _)) = particles.get_mut(info.id) {
            let new_pos = ray.origin + ray.direction * info.grab_distance;
            point.0 = new_pos;
            gizmos.cuboid(
                Transform::from_translation(new_pos).with_scale(Vec3::splat(0.1)),
                Color::hex("#568C4D").unwrap(),
            );
        }
    }
}
