use bevy::prelude::*;

#[derive(Component, Default, Clone, Debug)]
pub struct MotorRPMs {
    pub rpms: [f32; 4],
    pub target_rpms: [f32; 4],
}
