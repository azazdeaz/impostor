use bevy::prelude::*;



#[derive(Component, Reflect, Default)]
#[reflect(Component)]
pub struct Transform(pub bevy::prelude::Transform);


#[derive(Component, Reflect, Default)]
#[reflect(Component)]
pub struct Primitive {
    pub shape: String,
}