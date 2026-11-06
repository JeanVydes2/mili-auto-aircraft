//! Aircraft-independent rigid-body aerodynamics and propulsion.
//!
//! A vehicle is assembled from an airframe, any number of propulsors, and any
//! number of aerodynamic surfaces. Flight controllers write normalized inputs;
//! the world only applies forces and does not depend on a vehicle type.

use avian3d::prelude::*;
use bevy::prelude::*;
use serde::Deserialize;
use std::path::PathBuf;

use crate::environment::CurrentWind;
use crate::sensors::{PrevVelocity, Telemetry};

#[derive(Component)]
pub struct Aircraft;

/// Stable per-vehicle index (0-based) assigned at spawn time in `setup_scene`
/// and preserved across respawns by `select_profile`. This drives MAVLink
/// `system_id`, FPV camera buffers and video feeds, so vehicle N always maps
/// to feed N regardless of Bevy query iteration order.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct VehicleIndex(pub usize);

#[derive(Component, Clone, Debug)]
pub struct Airframe {
    pub mass: f32,
    pub inertia: Vec3,
    pub center_of_mass: Vec3,
    pub linear_drag: Vec3,
    pub quadratic_drag: Vec3,
    pub angular_drag: Vec3,
}

#[derive(Clone, Debug)]
pub struct Propulsor {
    pub position: Vec3,
    pub axis: Vec3,
    pub max_thrust: f32,
    pub reaction_torque: f32,
    pub response_time: f32,
    pub control_channel: usize,
    pub throttle: f32,
}

#[derive(Component, Clone, Debug, Default)]
pub struct Propulsion(pub Vec<Propulsor>);

#[derive(Clone, Debug)]
pub struct AeroSurface {
    pub position: Vec3,
    pub chord_axis: Vec3,
    pub lift_axis: Vec3,
    pub area: f32,
    pub zero_lift_angle: f32,
    pub lift_slope: f32,
    pub max_lift_coefficient: f32,
    pub zero_lift_drag: f32,
    pub induced_drag: f32,
    pub control_channel: Option<usize>,
    pub control_sign: f32,
    pub control_lift: f32,
}

#[derive(Component, Clone, Debug, Default)]
pub struct FlightSurfaces(pub Vec<AeroSurface>);

#[derive(Component, Clone, Debug, Default)]
pub struct AircraftControls {
    pub propulsion: Vec<f32>,
    pub surfaces: Vec<f32>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AircraftType {
    #[default]
    Quad,
    Hex,
    FixedWing,
    Vtol,
}

#[derive(Component, Clone, Copy, Debug, Default)]
pub struct AircraftModel(pub AircraftType);

#[derive(Clone)]
pub struct AircraftDefinition {
    pub kind: AircraftType,
    pub body: Airframe,
    pub propulsion: Propulsion,
    pub surfaces: FlightSurfaces,
    pub collider_half_extents: Vec3,
}

#[derive(Resource, Clone)]
pub struct AircraftCatalog {
    pub definitions: Vec<AircraftDefinition>,
    pub selected: usize,
}

#[derive(Deserialize)]
struct ProfileFile {
    aircraft: Vec<ProfileEntry>,
}

#[derive(Deserialize)]
struct ProfileEntry {
    kind: AircraftType,
    mass: f32,
    inertia: [f32; 3],
    center_of_mass: [f32; 3],
    linear_drag: [f32; 3],
    quadratic_drag: [f32; 3],
    angular_drag: [f32; 3],
    collider_half_extents: [f32; 3],
    #[serde(default)]
    propulsion: Vec<PropulsorEntry>,
    #[serde(default)]
    surfaces: Vec<SurfaceEntry>,
}

#[derive(Deserialize)]
struct PropulsorEntry {
    position: [f32; 3],
    axis: [f32; 3],
    max_thrust: f32,
    reaction_torque: f32,
    response_time: f32,
    control_channel: usize,
}

#[derive(Deserialize)]
struct SurfaceEntry {
    position: [f32; 3],
    chord_axis: [f32; 3],
    lift_axis: [f32; 3],
    area: f32,
    zero_lift_angle: f32,
    lift_slope: f32,
    max_lift_coefficient: f32,
    zero_lift_drag: f32,
    induced_drag: f32,
    control_channel: Option<usize>,
    control_sign: f32,
    control_lift: f32,
}

impl AircraftCatalog {
    pub fn load() -> Self {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("assets")
            .join("aircraft_profiles.toml");
        let source = std::fs::read_to_string(&path)
            .unwrap_or_else(|error| panic!("Could not read {}: {error}", path.display()));
        let file: ProfileFile = toml::from_str(&source).unwrap_or_else(|error| {
            panic!("Invalid aircraft profile in {}: {error}", path.display())
        });
        assert!(!file.aircraft.is_empty(), "The aircraft catalog is empty");
        let definitions = file
            .aircraft
            .into_iter()
            .map(|entry| {
                let mut definition = AircraftDefinition::profile(entry.kind);
                definition.body.mass = entry.mass;
                definition.body.inertia = Vec3::from_array(entry.inertia);
                definition.body.center_of_mass = Vec3::from_array(entry.center_of_mass);
                definition.body.linear_drag = Vec3::from_array(entry.linear_drag);
                definition.body.quadratic_drag = Vec3::from_array(entry.quadratic_drag);
                definition.body.angular_drag = Vec3::from_array(entry.angular_drag);
                definition.collider_half_extents = Vec3::from_array(entry.collider_half_extents);
                if !entry.propulsion.is_empty() {
                    definition.propulsion = Propulsion(
                        entry
                            .propulsion
                            .into_iter()
                            .map(|propulsor| Propulsor {
                                position: Vec3::from_array(propulsor.position),
                                axis: Vec3::from_array(propulsor.axis),
                                max_thrust: propulsor.max_thrust,
                                reaction_torque: propulsor.reaction_torque,
                                response_time: propulsor.response_time,
                                control_channel: propulsor.control_channel,
                                throttle: 0.0,
                            })
                            .collect(),
                    );
                }
                if !entry.surfaces.is_empty() {
                    definition.surfaces = FlightSurfaces(
                        entry
                            .surfaces
                            .into_iter()
                            .map(|surface| AeroSurface {
                                position: Vec3::from_array(surface.position),
                                chord_axis: Vec3::from_array(surface.chord_axis),
                                lift_axis: Vec3::from_array(surface.lift_axis),
                                area: surface.area,
                                zero_lift_angle: surface.zero_lift_angle,
                                lift_slope: surface.lift_slope,
                                max_lift_coefficient: surface.max_lift_coefficient,
                                zero_lift_drag: surface.zero_lift_drag,
                                induced_drag: surface.induced_drag,
                                control_channel: surface.control_channel,
                                control_sign: surface.control_sign,
                                control_lift: surface.control_lift,
                            })
                            .collect(),
                    );
                }
                assert!(
                    definition.body.mass > 0.0
                        && definition.body.inertia.min_element() > 0.0
                        && definition.collider_half_extents.min_element() > 0.0,
                    "Mass, inertia, and collider dimensions must be positive for {:?}",
                    entry.kind
                );
                definition
            })
            .collect();
        Self {
            definitions,
            selected: 0,
        }
    }

    pub fn selected(&self) -> &AircraftDefinition {
        &self.definitions[self.selected]
    }

    pub fn selected_kind(&self) -> AircraftType {
        self.selected().kind
    }

    pub fn profile(&self, kind: AircraftType) -> AircraftDefinition {
        self.definitions
            .iter()
            .find(|d| d.kind == kind)
            .cloned()
            .unwrap_or_else(|| self.selected().clone())
    }
}

impl AircraftDefinition {
    pub fn profile(kind: AircraftType) -> Self {
        let body = match kind {
            AircraftType::Quad => Airframe {
                mass: 1.5,
                inertia: Vec3::new(0.029, 0.055, 0.029),
                center_of_mass: Vec3::ZERO,
                linear_drag: Vec3::splat(0.1),
                quadratic_drag: Vec3::splat(0.01),
                angular_drag: Vec3::splat(0.05),
            },
            AircraftType::Hex => Airframe {
                mass: 2.5,
                inertia: Vec3::new(0.12, 0.20, 0.12),
                center_of_mass: Vec3::ZERO,
                linear_drag: Vec3::splat(0.14),
                quadratic_drag: Vec3::splat(0.015),
                angular_drag: Vec3::splat(0.08),
            },
            AircraftType::FixedWing => Airframe {
                mass: 3.0,
                inertia: Vec3::new(0.35, 0.18, 0.48),
                center_of_mass: Vec3::ZERO,
                linear_drag: Vec3::splat(0.03),
                quadratic_drag: Vec3::new(0.08, 0.12, 0.035),
                angular_drag: Vec3::splat(0.12),
            },
            AircraftType::Vtol => Airframe {
                mass: 4.0,
                inertia: Vec3::new(0.48, 0.62, 0.45),
                center_of_mass: Vec3::ZERO,
                linear_drag: Vec3::splat(0.06),
                quadratic_drag: Vec3::new(0.08, 0.1, 0.04),
                angular_drag: Vec3::splat(0.1),
            },
        };

        let mut propulsion = Vec::new();
        let mut surfaces = Vec::new();
        match kind {
            AircraftType::Quad | AircraftType::Hex => {
                let count = if kind == AircraftType::Quad { 4 } else { 6 };
                let arm = if kind == AircraftType::Quad {
                    0.22
                } else {
                    0.35
                };
                for i in 0..count {
                    let angle = std::f32::consts::TAU * i as f32 / count as f32
                        + std::f32::consts::FRAC_PI_4;
                    propulsion.push(Propulsor {
                        position: Vec3::new(angle.cos() * arm, 0.0, angle.sin() * arm),
                        axis: Vec3::Y,
                        max_thrust: if kind == AircraftType::Quad {
                            12.0
                        } else {
                            11.0
                        },
                        reaction_torque: if i % 2 == 0 { 0.18 } else { -0.18 },
                        response_time: 0.02,
                        control_channel: i,
                        throttle: 0.0,
                    });
                }
            }
            AircraftType::FixedWing => {
                propulsion.push(pusher_propeller(0.22, 0));
                surfaces.extend(fixed_wing_surfaces(0.95));
            }
            AircraftType::Vtol => {
                for (i, position) in [
                    Vec3::new(-0.55, 0.0, -0.45),
                    Vec3::new(0.55, 0.0, -0.45),
                    Vec3::new(-0.55, 0.0, 0.45),
                    Vec3::new(0.55, 0.0, 0.45),
                ]
                .into_iter()
                .enumerate()
                {
                    propulsion.push(Propulsor {
                        position,
                        axis: Vec3::Y,
                        max_thrust: 15.0,
                        reaction_torque: if i % 2 == 0 { 0.2 } else { -0.2 },
                        response_time: 0.025,
                        control_channel: i,
                        throttle: 0.0,
                    });
                }
                propulsion.push(pusher_propeller(0.28, 4));
                surfaces.extend(fixed_wing_surfaces(1.3));
            }
        }

        Self {
            kind,
            body,
            propulsion: Propulsion(propulsion),
            surfaces: FlightSurfaces(surfaces),
            collider_half_extents: match kind {
                AircraftType::Quad => Vec3::new(0.35, 0.06, 0.35),
                AircraftType::Hex => Vec3::new(0.55, 0.08, 0.55),
                AircraftType::FixedWing => Vec3::new(0.9, 0.12, 0.55),
                AircraftType::Vtol => Vec3::new(0.75, 0.12, 0.65),
            },
        }
    }

    pub fn spawn(&self, commands: &mut Commands, position: Vec3) -> Entity {
        let propulsion_channels = self
            .propulsion
            .0
            .iter()
            .map(|propulsor| propulsor.control_channel + 1)
            .max()
            .unwrap_or_default();
        let surface_channels = self
            .surfaces
            .0
            .iter()
            .filter_map(|surface| surface.control_channel.map(|channel| channel + 1))
            .max()
            .unwrap_or_default();
        commands
            .spawn((
                Aircraft,
                AircraftModel(self.kind),
                self.body.clone(),
                self.propulsion.clone(),
                self.surfaces.clone(),
                AircraftControls {
                    propulsion: vec![0.0; propulsion_channels],
                    surfaces: vec![0.0; surface_channels],
                },
                RigidBody::Dynamic,
                Collider::cuboid(
                    self.collider_half_extents.x * 2.0,
                    self.collider_half_extents.y * 2.0,
                    self.collider_half_extents.z * 2.0,
                ),
                Mass(self.body.mass),
                AngularInertia::new(self.body.inertia),
                CenterOfMass(self.body.center_of_mass),
                NoAutoMass,
                NoAutoAngularInertia,
                NoAutoCenterOfMass,
            ))
            .insert((
                CollidingEntities::default(),
                Transform::from_translation(position),
                GlobalTransform::default(),
                Visibility::default(),
                crate::motor_rpms::MotorRPMs::default(),
                crate::config::BatteryState::default(),
                Telemetry::default(),
                PrevVelocity::default(),
                crate::motors::FlightControlState::default(),
                crate::motors::MotorDynamicsState::default(),
                crate::motors::CommandInput::default(),
            ))
            .id()
    }
}

fn pusher_propeller(position_z: f32, channel: usize) -> Propulsor {
    Propulsor {
        position: Vec3::new(0.0, 0.0, position_z),
        axis: Vec3::Z,
        max_thrust: 24.0,
        reaction_torque: 0.12,
        response_time: 0.08,
        control_channel: channel,
        throttle: 0.0,
    }
}

fn fixed_wing_surfaces(wing_area: f32) -> Vec<AeroSurface> {
    let panel_area = wing_area * 0.42;
    vec![
        AeroSurface {
            position: Vec3::new(-0.42, 0.0, 0.0),
            chord_axis: Vec3::Z,
            lift_axis: Vec3::Y,
            area: panel_area,
            zero_lift_angle: 0.0,
            lift_slope: 5.2,
            max_lift_coefficient: 1.35,
            zero_lift_drag: 0.025,
            induced_drag: 0.055,
            control_channel: Some(1),
            control_sign: -1.0,
            control_lift: 1.4,
        },
        AeroSurface {
            position: Vec3::new(0.42, 0.0, 0.0),
            control_sign: 1.0,
            ..fixed_wing_surfaces_panel(panel_area)
        },
        AeroSurface {
            position: Vec3::new(0.0, 0.0, -0.48),
            area: wing_area * 0.16,
            control_channel: Some(2),
            control_sign: -1.0,
            control_lift: 1.7,
            ..fixed_wing_surfaces_panel(wing_area * 0.16)
        },
        AeroSurface {
            position: Vec3::new(0.0, 0.16, -0.40),
            chord_axis: Vec3::Z,
            lift_axis: Vec3::X,
            area: wing_area * 0.08,
            zero_lift_angle: 0.0,
            lift_slope: 3.5,
            max_lift_coefficient: 1.0,
            zero_lift_drag: 0.04,
            induced_drag: 0.04,
            control_channel: Some(3),
            control_sign: 1.0,
            control_lift: 1.5,
        },
    ]
}

fn fixed_wing_surfaces_panel(area: f32) -> AeroSurface {
    AeroSurface {
        position: Vec3::ZERO,
        chord_axis: Vec3::Z,
        lift_axis: Vec3::Y,
        area,
        zero_lift_angle: 0.0,
        lift_slope: 5.2,
        max_lift_coefficient: 1.35,
        zero_lift_drag: 0.025,
        induced_drag: 0.055,
        control_channel: Some(1),
        control_sign: 1.0,
        control_lift: 1.4,
    }
}

pub fn from_quad_config(config: &crate::config::DroneConfig) -> (Airframe, Propulsion) {
    let propulsion = config
        .motors
        .iter()
        .enumerate()
        .map(|(index, motor)| {
            let omega = motor.max_rpm.max(0.0) * std::f32::consts::TAU / 60.0;
            Propulsor {
                position: motor.position,
                axis: Vec3::Y,
                max_thrust: motor.thrust_coeff.max(0.0)
                    * motor.voltage_efficiency.max(0.0)
                    * omega.powi(2),
                reaction_torque: motor.rotation_dir * motor.torque_coeff.max(0.0) * omega.powi(2),
                response_time: 1e-4,
                control_channel: index,
                throttle: 0.0,
            }
        })
        .collect();
    (
        Airframe {
            mass: config.mass,
            inertia: config.inertia,
            center_of_mass: config.center_of_mass_offset,
            linear_drag: config.linear_drag,
            quadratic_drag: Vec3::splat(config.induced_drag.max(0.0)),
            angular_drag: config.angular_drag,
        },
        Propulsion(propulsion),
    )
}

pub fn apply_aircraft_forces_system(
    time: Res<Time>,
    wind: Res<CurrentWind>,
    env: Res<crate::config::EnvironmentConfig>,
    mut aircraft: Query<
        (
            &Transform,
            &Airframe,
            &mut Propulsion,
            &FlightSurfaces,
            &AircraftControls,
            Forces,
        ),
        With<Aircraft>,
    >,
) {
    let dt = time.delta_secs().max(0.0);
    for (transform, body, mut propulsion, surfaces, controls, mut forces) in &mut aircraft {
        let (force, torque) = aircraft_wrench(
            body,
            &mut propulsion,
            surfaces,
            controls,
            transform.rotation,
            forces.linear_velocity(),
            forces.angular_velocity(),
            wind.0,
            env.air_density,
            dt,
        );
        forces.apply_force(transform.rotation * force);
        forces.apply_torque(transform.rotation * torque);
    }
}

pub fn aircraft_wrench(
    body: &Airframe,
    propulsion: &mut Propulsion,
    surfaces: &FlightSurfaces,
    controls: &AircraftControls,
    rotation: Quat,
    velocity_world: Vec3,
    angular_velocity_world: Vec3,
    wind_world: Vec3,
    air_density: f32,
    dt: f32,
) -> (Vec3, Vec3) {
    let density = (air_density / 1.225).max(0.0);
    let velocity = rotation.inverse() * (velocity_world - wind_world);
    let angular_velocity = rotation.inverse() * angular_velocity_world;
    let mut force =
        -(body.linear_drag * velocity + body.quadratic_drag * velocity * velocity.abs()) * density;
    let mut torque = -body.angular_drag * angular_velocity * density;

    for propulsor in &mut propulsion.0 {
        let command = controls
            .propulsion
            .get(propulsor.control_channel)
            .copied()
            .unwrap_or(0.0)
            .clamp(0.0, 1.0);
        let response = 1.0 - (-dt.max(0.0) / propulsor.response_time.max(1e-4)).exp();
        propulsor.throttle += (command - propulsor.throttle) * response;
        let throttle_squared = propulsor.throttle.clamp(0.0, 1.0).powi(2);
        let axis = propulsor.axis.normalize_or_zero();
        let thrust = axis * propulsor.max_thrust.max(0.0) * throttle_squared * density;
        force += thrust;
        torque += (propulsor.position - body.center_of_mass).cross(thrust)
            + axis * propulsor.reaction_torque * throttle_squared * density;
    }

    for surface in &surfaces.0 {
        let chord = surface.chord_axis.normalize_or_zero();
        let lift_axis =
            (surface.lift_axis - chord * surface.lift_axis.dot(chord)).normalize_or_zero();
        let local_velocity =
            velocity + angular_velocity.cross(surface.position - body.center_of_mass);
        let speed_squared = local_velocity.length_squared();
        if speed_squared < 1e-6 || surface.area <= 0.0 {
            if speed_squared < 1e-6
                && surface.control_channel.is_some()
                && surface.lift_axis.x.abs() > surface.lift_axis.y.abs()
            {
                let command = surface
                    .control_channel
                    .and_then(|channel| controls.surfaces.get(channel))
                    .copied()
                    .unwrap_or(0.0)
                    * surface.control_sign;
                torque.y += command.clamp(-1.0, 1.0) * body.mass * 0.9;
            }
            continue;
        }
        let speed = speed_squared.sqrt();
        let alpha = -local_velocity
            .dot(lift_axis)
            .atan2(local_velocity.dot(chord))
            - surface.zero_lift_angle
            + surface.control_channel.map_or(0.0, |channel| {
                controls.surfaces.get(channel).copied().unwrap_or(0.0)
                    * surface.control_sign
                    * surface.control_lift
            });
        let cl = (surface.lift_slope * alpha).clamp(
            -surface.max_lift_coefficient.abs(),
            surface.max_lift_coefficient.abs(),
        );
        let cd = surface.zero_lift_drag.max(0.0) + surface.induced_drag.max(0.0) * cl * cl;
        let dynamic_pressure_area = 0.5 * air_density.max(0.0) * speed_squared * surface.area;
        let drag = -local_velocity / speed * (dynamic_pressure_area * cd);
        let lift_direction = (lift_axis
            - local_velocity / speed * lift_axis.dot(local_velocity / speed))
        .normalize_or_zero();
        let lift = lift_direction * (dynamic_pressure_area * cl);
        let surface_force = drag + lift;
        force += surface_force;
        torque += (surface.position - body.center_of_mass).cross(surface_force);
        // A parked fixed-wing aircraft still needs a bounded ground-steering
        // moment. Aerodynamic rudder authority is zero at standstill, so use
        // the rudder command as a low-speed nose-wheel/tailwheel surrogate.
        if speed_squared < 16.0
            && surface.control_channel.is_some()
            && lift_axis.x.abs() > lift_axis.y.abs()
        {
            let command = surface
                .control_channel
                .and_then(|channel| controls.surfaces.get(channel))
                .copied()
                .unwrap_or(0.0)
                * surface.control_sign;
            torque.y += command.clamp(-1.0, 1.0) * body.mass * 0.9;
        }
    }

    (force, torque)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn zero_drag_body(mass: f32) -> Airframe {
        Airframe {
            mass,
            inertia: Vec3::ONE,
            center_of_mass: Vec3::ZERO,
            linear_drag: Vec3::ZERO,
            quadratic_drag: Vec3::ZERO,
            angular_drag: Vec3::ZERO,
        }
    }

    #[test]
    fn profiles_are_composed_from_arbitrary_propulsor_and_surface_counts() {
        let quad = AircraftDefinition::profile(AircraftType::Quad);
        let hex = AircraftDefinition::profile(AircraftType::Hex);
        let fixed_wing = AircraftDefinition::profile(AircraftType::FixedWing);
        let vtol = AircraftDefinition::profile(AircraftType::Vtol);

        assert_eq!(quad.propulsion.0.len(), 4);
        assert_eq!(hex.propulsion.0.len(), 6);
        assert_eq!(fixed_wing.propulsion.0.len(), 1);
        assert!(fixed_wing.surfaces.0.len() >= 3);
        assert_eq!(vtol.propulsion.0.len(), 5);
        assert!(vtol.surfaces.0.len() >= 3);
    }

    #[test]
    fn editable_profile_file_loads_all_types_and_physical_parameters() {
        let catalog = AircraftCatalog::load();
        assert_eq!(catalog.definitions.len(), 4);
        assert_eq!(catalog.definitions[0].kind, AircraftType::Quad);
        assert_eq!(catalog.definitions[1].kind, AircraftType::Hex);
        assert_eq!(catalog.definitions[2].kind, AircraftType::FixedWing);
        assert_eq!(catalog.definitions[3].kind, AircraftType::Vtol);
        assert_eq!(catalog.definitions[0].body.mass, 1.5);
        assert_eq!(
            catalog.definitions[2].collider_half_extents,
            Vec3::new(0.9, 0.12, 0.55)
        );
    }

    #[test]
    fn profile_spawner_creates_an_aircraft_without_world_specific_drone_components() {
        let mut app = App::new();
        app.add_systems(Startup, |mut commands: Commands| {
            AircraftDefinition::profile(AircraftType::FixedWing)
                .spawn(&mut commands, Vec3::new(0.0, 3.0, 0.0));
        });
        app.update();

        let world = app.world_mut();
        let mut query = world.query_filtered::<(
            &AircraftModel,
            &Airframe,
            &Propulsion,
            &FlightSurfaces,
            &Transform,
        ), With<Aircraft>>();
        let (model, body, propulsion, surfaces, transform) = query
            .single(world)
            .expect("fixed-wing profile should spawn");
        assert_eq!(model.0, AircraftType::FixedWing);
        assert!(body.mass > 0.0 && body.inertia.min_element() > 0.0);
        assert_eq!(propulsion.0.len(), 1);
        assert!(surfaces.0.len() >= 3);
        assert_eq!(transform.translation, Vec3::new(0.0, 3.0, 0.0));
    }

    #[test]
    fn any_number_of_propulsors_contributes_force_and_moment_about_com() {
        let body = Airframe {
            center_of_mass: Vec3::new(0.1, 0.0, 0.0),
            ..zero_drag_body(2.0)
        };
        let mut propulsion = AircraftDefinition::profile(AircraftType::Hex).propulsion;
        let controls = AircraftControls {
            propulsion: vec![1.0; 6],
            ..default()
        };
        let (force, torque) = aircraft_wrench(
            &body,
            &mut propulsion,
            &FlightSurfaces::default(),
            &controls,
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::ZERO,
            1.225,
            1.0,
        );
        assert!(force.y > 0.0);
        assert!(torque.is_finite());
    }

    #[test]
    fn existing_quad_configuration_maps_to_the_generic_wrench_without_force_drift() {
        let config = crate::config::DroneConfig::default();
        let env = crate::config::EnvironmentConfig::default();
        let (body, mut propulsion) = from_quad_config(&config);
        let controls = AircraftControls {
            propulsion: vec![1.0; config.motors.len()],
            ..default()
        };
        let (generic_force, generic_torque) = aircraft_wrench(
            &body,
            &mut propulsion,
            &FlightSurfaces::default(),
            &controls,
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::ZERO,
            env.air_density,
            1.0,
        );
        let rpms = crate::motor_rpms::MotorRPMs {
            rpms: std::array::from_fn(|i| config.motors[i].max_rpm),
            target_rpms: [0.0; 4],
        };
        let (legacy_force, legacy_torque) =
            crate::motors::motor_forces_and_torques(&config, &env, &rpms);
        assert!((generic_force - legacy_force).length() < 1e-3);
        assert!((generic_torque - legacy_torque).length() < 1e-3);
    }

    #[test]
    fn wing_generates_lift_only_with_airflow_and_changes_with_control_input() {
        let body = zero_drag_body(3.0);
        let mut surfaces = AircraftDefinition::profile(AircraftType::FixedWing).surfaces;
        let mut propulsion = Propulsion::default();
        let controls = AircraftControls {
            surfaces: vec![0.0, 0.0, 0.0, 0.0],
            ..default()
        };
        let (stationary, _) = aircraft_wrench(
            &body,
            &mut propulsion,
            &surfaces,
            &controls,
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::ZERO,
            1.225,
            1.0,
        );
        assert_eq!(stationary, Vec3::ZERO);

        let parked_controls = AircraftControls {
            surfaces: vec![0.0, 0.0, 0.0, 1.0],
            ..default()
        };
        let (_, parked_steering_torque) = aircraft_wrench(
            &body,
            &mut propulsion,
            &surfaces,
            &parked_controls,
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::ZERO,
            1.225,
            1.0,
        );
        assert!(
            parked_steering_torque.y.abs() > 0.1,
            "rudder must provide bounded ground steering at rest"
        );

        let (level, _) = aircraft_wrench(
            &body,
            &mut propulsion,
            &surfaces,
            &controls,
            Quat::IDENTITY,
            Vec3::new(0.0, 0.0, 18.0),
            Vec3::ZERO,
            Vec3::ZERO,
            1.225,
            1.0,
        );
        assert!(level.y.abs() < 1e-4);

        let controls = AircraftControls {
            surfaces: vec![0.0, 0.1, 0.0, 0.0],
            ..default()
        };
        let (deflected, _) = aircraft_wrench(
            &body,
            &mut propulsion,
            &surfaces,
            &controls,
            Quat::IDENTITY,
            Vec3::new(0.0, -2.0, 18.0),
            Vec3::ZERO,
            Vec3::ZERO,
            1.225,
            1.0,
        );
        assert!(deflected.y > 0.0);
        surfaces.0.clear();
    }

    #[test]
    fn propulsor_throttle_respects_first_order_response() {
        let body = zero_drag_body(1.0);
        let mut propulsion = Propulsion(vec![Propulsor {
            position: Vec3::ZERO,
            axis: Vec3::Z,
            max_thrust: 20.0,
            reaction_torque: 0.0,
            response_time: 0.1,
            control_channel: 0,
            throttle: 0.0,
        }]);
        let controls = AircraftControls {
            propulsion: vec![1.0],
            ..default()
        };
        let (force, _) = aircraft_wrench(
            &body,
            &mut propulsion,
            &FlightSurfaces::default(),
            &controls,
            Quat::IDENTITY,
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::ZERO,
            1.225,
            0.01,
        );
        assert!(force.z > 0.0 && force.z < 1.0);
    }

    #[test]
    fn drag_uses_relative_airflow_and_damping_removes_energy_at_any_heading() {
        let body = Airframe {
            linear_drag: Vec3::splat(0.1),
            quadratic_drag: Vec3::splat(0.01),
            angular_drag: Vec3::splat(0.05),
            ..zero_drag_body(1.0)
        };
        let mut propulsion = Propulsion::default();
        let surfaces = FlightSurfaces::default();
        for yaw in [0.0, 1.0, 2.5] {
            let rotation = Quat::from_rotation_y(yaw);
            let velocity = Vec3::new(2.0, -1.0, 3.0);
            let rate = Vec3::new(-0.2, 0.1, 0.4);
            let (drag, torque) = aircraft_wrench(
                &body,
                &mut propulsion,
                &surfaces,
                &AircraftControls::default(),
                rotation,
                velocity,
                rate,
                Vec3::ZERO,
                1.225,
                1.0 / 240.0,
            );
            assert!((rotation * drag).dot(velocity) < 0.0);
            assert!((rotation * torque).dot(rate) < 0.0);
        }
        let wind = Vec3::new(3.0, 0.0, -2.0);
        let (matching, _) = aircraft_wrench(
            &body,
            &mut propulsion,
            &surfaces,
            &AircraftControls::default(),
            Quat::IDENTITY,
            wind,
            Vec3::ZERO,
            wind,
            1.225,
            1.0 / 240.0,
        );
        assert_eq!(matching, Vec3::ZERO);
    }
}
