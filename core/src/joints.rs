use bevy::math::prelude::*;
use bevy::ecs::prelude::*;
use bevy::prelude::GlobalTransform;
use bevy::reflect::prelude::*;
use crate::velocity::AxisAngle; //might be better to factor this out?


#[derive(Debug, Copy, Clone, PartialEq, Reflect)]
///A rapier Joint component.
pub struct Joint {
    ///The entity corresponding to the first body in the joint.
    pub entity_1: Entity,
    ///The entity corresponding to the second body in the joint.
    pub entity_2: Entity,
    ///The details of the joint.
    pub spec: JointSpec,
}

#[derive(Debug, Copy, Clone, PartialEq, Reflect)]
/// A specification of a joint. The type of joint and the parameters (distance, angle, etc).
pub enum JointSpec {
    /// A ball joint. Fixes two points on the linked bodies to each other,
    /// but allows free rotation around that point.
    Ball {
        /// The coordinates, in the local coordinate system of the first body,
        /// of the first point.
        point_1: Vec3,
        /// The coordinates, in the local coordinate system of the second body,
        /// of the second point.
        point_2: Vec3,
    },
    /// A fixed joint. Fixes the two bodies completely together, allowing
    /// no independent movement. You must specify both the points on the
    /// two bodies that should be fixed together, and the angle. (This
    /// data is captured in the Isometry struct)
    Fixed {
        /// The first isometry
        iso_1: Isometry,
        /// The second isometry
        iso_2: Isometry,
    },
    /// A revolute joint. Fixes the two bodies together, allowing only rotation
    /// about a specific axis. You must specify the points where the bodies
    /// should be fixed, and the axis of rotation in both bodies' local
    /// coordinate system.
    /// In 2d, simply treated as a BallJoint with the given points.
    Revolute {
        /// The point on the first body.
        point_1: Vec3,
        /// The axis in the first body's local coordinates.
        /// Not assumed to be normalized.
        axis_1: Vec3,
        /// The point on the second body.
        point_2: Vec3,
        /// The axis in the second body's local coordinates.
        /// Not assumed to be normalized.
        axis_2: Vec3,
    },
    ///A prismatic joint. Fixes the two bodies together, allowing only
    /// translation along a specific axis. You must specify the points where the bodies
    /// should be fixed, and the axis of rotation in both bodies' local
    /// coordinate system.
    Prismatic {
        // For simplicity we simply don't specify the tangent axes that
        // one can optionally specify in Rapier.

        /// The point on the first body.
        point_1: Vec3,
        /// The axis in the first body's local coordinates.
        /// Not assumed to be normalized.
        axis_1: Vec3,
        /// The point on the second body.
        point_2: Vec3,
        /// The axis in the second body's local coordinates.
        /// Not assumed to be normalized.
        axis_2: Vec3,
    },
}

impl Joint {
    /// A ball joint centered at the first entity,
    /// and at coordinate arm with respect to the second entity.
    /// (In other words, places the second entity on an "arm" attached to
    /// the first entity).
    pub fn ball_arm(e1 : Entity, e2: Entity, arm: Vec3) -> Joint {
        Joint {
            entity_1: e1,
            entity_2: e2,
            spec: JointSpec::Ball {
                point_1: Vec3::ZERO,
                point_2: arm
            }
        }
    }


    /// Create a ball joint on the entities, using their current positions,
    /// with the "center" of the ball joint being at `pt` in the local coords
    /// of e1. Fails if either entity doesn't have a GlobalTransform component.
    pub fn ball_current(
        transforms: Query<'_, &GlobalTransform>,
        e1: Entity,
        e2: Entity,
        pt1: Vec3
    ) -> Option<Joint> {
        if let (Ok(trans_1),Ok(trans_2)) =
            (transforms.get(e1),transforms.get(e2)) {
                Some(Joint::ball_with_transforms(e1, e2, *trans_1, *trans_2, pt1))
            } else {
                None
            }
    }

    /// Create a fixed joint on the two entities,
    /// using their current positions and rotations.
    pub fn fixed_current(
        transforms: &Query<'_, &GlobalTransform>,
        e1: Entity,
        e2: Entity
    ) -> Option<Joint> {
        if let (Ok(trans_1),Ok(trans_2)) =
            (transforms.get(e1),transforms.get(e2)) {
                Some(Joint::fixed_with_transforms(e1, e2, *trans_1, *trans_2))
            }
        else {None}
    }

    /// Create a prismatic joint for the two entities, using their current
    /// positions. The joint is on the given point and axis given in the
    /// coordinate system of the *first* entity.
    pub fn revolute_current(
        transforms: Query<'_, &GlobalTransform>,
        e1: Entity,
        e2: Entity,
        pt1: Vec3,
        ax1: Vec3,
    ) -> Option<Joint> {
        if let(Ok(trans_1),Ok(trans_2)) =
            (transforms.get(e1),transforms.get(e2)) {
                Some(Joint::revolute_with_transforms(e1, e2, *trans_1, *trans_2, pt1, ax1))
            } else {None}
    }

    /// Create a prismatic joint for the two entities, using their current
    /// positions. The joint is on the given point and axis given in the
    /// coordinate system of the *first* entity.
    pub fn prismatic_current(
        transforms: Query<'_, &GlobalTransform>,
        e1: Entity,
        e2: Entity,
        pt1: Vec3,
        ax1: Vec3,
    ) -> Option<Joint> {
        if let(Ok(trans_1),Ok(trans_2)) =
            (transforms.get(e1),transforms.get(e2)) {
                Some(Joint::prismatic_with_transforms(e1, e2, *trans_1, *trans_2, pt1, ax1))
            } else {None}
    }

    pub fn fixed_with_transforms(
        e1: Entity,
        e2: Entity,
        t1: GlobalTransform,
        t2: GlobalTransform
    ) -> Joint {
        Joint {
            entity_1: e1,
            entity_2: e2,
            spec: JointSpec::Fixed {
                iso_1: Isometry::default(),
                iso_2: Isometry::from(t2)
                    .inverse().mul(Isometry::from(t1))
            }
        }
    }
    pub fn ball_with_transforms(
        e1: Entity,
        e2: Entity,
        t1: GlobalTransform,
        t2: GlobalTransform,
        pt1: Vec3,
    ) -> Joint {
        Joint {
            entity_1: e1,
            entity_2: e2,
            spec: JointSpec::Ball {
                point_1: pt1,
                point_2: conversion_matrix(t2, t1).transform_point3(pt1)
            }
        }
    }
    pub fn revolute_with_transforms(
        e1: Entity,
        e2: Entity,
        t1: GlobalTransform,
        t2: GlobalTransform,
        pt1: Vec3,
        ax1: Vec3,
    ) -> Joint {
        Joint {
            entity_1: e1,
            entity_2: e2,
            spec: JointSpec::Revolute {
                point_1: pt1,
                axis_1: ax1,
                point_2: conversion_matrix(t2, t1).transform_point3(pt1),
                axis_2: conversion_matrix(t2, t1).transform_vector3(ax1)
            }
        }
    }

    pub fn prismatic_with_transforms(
        e1: Entity,
        e2: Entity,
        t1: GlobalTransform,
        t2: GlobalTransform,
        pt1: Vec3,
        ax1: Vec3
    ) -> Joint {
        Joint {
            entity_1: e1,
            entity_2: e2,
            spec: JointSpec::Prismatic {
                point_1: pt1,
                axis_1: ax1,
                point_2: conversion_matrix(t2,t1).transform_point3(pt1),
                axis_2: conversion_matrix(t2,t1).transform_vector3(ax1),
            },
        }
    }


}

fn conversion_matrix(to: GlobalTransform, from: GlobalTransform) -> Mat4 {
    to.compute_matrix().inverse().mul_mat4(&from.compute_matrix())
}

#[derive(Debug, Copy, Clone, PartialEq, Reflect)]
/// An Isometry, implemented as a bevy Mat4 (which is assumed) to
/// *be* an isometry. All the exported constructors guarantee this.
pub struct Isometry(Mat4);



impl Default for Isometry {
    fn default() -> Isometry {
        Isometry(Mat4::IDENTITY)
    }
}

impl Isometry {
    /// An isometry given by translation by the given vector
    pub fn from_translation(trans: Vec3) -> Isometry {
        Isometry(Mat4::from_translation(trans))
    }
    /// An isometry given by rotation by the given axis+angle
    pub fn from_rotation(rot: AxisAngle) -> Isometry {
        Isometry(Mat4::from_quat(Quat::from(rot)))
    }
    /// Compose two isometries (by matrix multiplication)
    pub fn mul(self, iso: Isometry) -> Isometry {
        Isometry(self.0.mul_mat4(&iso.0))
    }
    /// Compose the isometry by a translation
    pub fn mul_translation(self, trans: Vec3) -> Isometry {
        self.mul(Isometry::from_translation(trans))
    }
    /// Compose the isometry by a rotation
    pub fn mul_rotation(self, rot: AxisAngle) -> Isometry {
        self.mul(Isometry::from_rotation(rot))
    }
    /// Find the inverse of the given isometry
    pub fn inverse(self) -> Isometry {
        Isometry(self.0.inverse())
    }
    /// represent as rotation followed by translation
    pub fn to_rotation_translation(self) -> (Quat,Vec3) {
        let (_,rot,trans) = self.0.to_scale_rotation_translation();
        (rot,trans)
    }
}

impl From<GlobalTransform> for Isometry {
    fn from(tr: GlobalTransform) -> Isometry {
        Isometry(tr.compute_matrix())
    }
}

impl From<(Vec3, AxisAngle)> for Isometry {
    fn from((trans,rot): (Vec3, AxisAngle)) -> Isometry {
        Isometry(Mat4::from_rotation_translation(Quat::from(rot), trans))
    }
}
