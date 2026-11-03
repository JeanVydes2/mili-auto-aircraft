use bevy::prelude::*;

#[derive(Resource, Default)]
pub struct MotorRPMs {
    pub rpms: [f32; 4],
    pub target_rpms: [f32; 4],
}
