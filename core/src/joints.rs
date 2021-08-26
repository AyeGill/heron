use bevy::math::prelude::*;
use bevy::ecs::prelude::*;
use bevy::reflect::prelude::*;
use duplicate::duplicate;
use crate::velocity::AxisAngle;


#[derive(Debug, Copy, Clone, PartialEq, Reflect)]
pub struct Joint {
    pub entity_1: Entity,
    pub entity_2: Entity,
    pub spec: JointSpec,
}

#[derive(Debug, Copy, Clone, PartialEq, Reflect)]
pub enum JointSpec {
    Ball {
        point_1: Vec3,
        point_2: Vec3,
    },
    Fixed {
        iso_1: Isometry,
        iso_2: Isometry,
    },
    Revolute {
        point_1: Vec3,
        axis_1: Vec3,
        point_2: Vec3,
        axis_2: Vec3,
    },
    Prismatic {
        //for simplicity, never specify local tangent axis
        point_1: Vec3,
        axis_1: Vec3,
        point_2: Vec3,
        axis_2: Vec3,
    },
}

impl JointSpec {
    /// creates a ball joint "centered on" entity 1 with entity moving around it
    fn ball_arm(arm: Vec3) -> JointSpec {
        JointSpec::Ball {point_1: Vec3::ZERO, point_2: arm}
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Reflect, Default)]
pub struct Isometry {
    //rotation *followed by* translation, following the nalgebra convention
    pub rot : AxisAngle,
    pub trans: Vec3,
}
