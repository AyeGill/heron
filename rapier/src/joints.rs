use bevy::ecs::prelude::*;


use crate::rapier::dynamics::{
    IslandManager, JointHandle, JointSet,
    RigidBodyHandle, RigidBodySet,
    BallJoint, FixedJoint, RevoluteJoint, PrismaticJoint,
};

use crate::nalgebra::Unit;

use heron_core::{Joint};
use crate::convert::{IntoRapier};

/*
Systems need to:
- create actual joints for any entities with `Joint` components that don't already have them
- whenever a joint component is removed, delete the associated joint
- whenever a joint is deleted (eg by deleting one of its bodies), delete the associated joint component

There is also a question about what to do if a joint is created/exists
pointing at an entity which doesn't actually have a RigidBody component.
It might make sense to create those components in that case, but for simplicity,
for now at least, we just delete the Joint component instead.
*/

pub(crate) fn create(
    mut commands: Commands,
    mut bodies: ResMut<'_, RigidBodySet>,
    mut joints: ResMut<JointSet>,
    joint_query: Query<
            (
                Entity,
                &Joint,
            ),
        Without<JointHandle>>,
    body_query: Query<&RigidBodyHandle>
) {
    use heron_core::JointSpec::*;
    for (entity, joint) in joint_query.iter() {
        //Get RigidBodyHandles associated to the two entities
        //On failure, delete this Joint component and exit.
        let b1 = match body_query.get(joint.entity_1) {
            Ok(b) => *b,
            Err(_) => {
                commands.entity(entity).remove::<Joint>();
                return;
            }
        };
        let b2 = match body_query.get(joint.entity_1) {
            Ok(b) => *b,
            Err(_) => {
                commands.entity(entity).remove::<Joint>();
                return;
            }
        };

        let handle = match joint.spec {
            Ball {point_1: v1, point_2: v2} => {
                let j = BallJoint::new(v1.into_rapier(), v2.into_rapier());
                joints.insert(&mut *bodies, b1, b2, j)
            },
            Fixed {iso_1: i1, iso_2: i2} => {
                let j = FixedJoint::new(i1.into_rapier(),i2.into_rapier());
                joints.insert(&mut *bodies, b1, b2,j)
            },
            Revolute {point_1: p1, axis_1: a1, point_2: p2, axis_2: a2} => {
                let j = RevoluteJoint::new(p1.into_rapier(),Unit::new_normalize(a1.into_rapier()),
                                           p2.into_rapier(),Unit::new_normalize(a2.into_rapier()));
                joints.insert(&mut *bodies, b1, b2,j)
            },
            Prismatic {point_1: p1, axis_1: a1, point_2: p2, axis_2: a2} => {
                let j = PrismaticJoint::new(p1.into_rapier(),Unit::new_normalize(a1.into_rapier()),Default::default(),
                                           p2.into_rapier(),Unit::new_normalize(a2.into_rapier()), Default::default());
                joints.insert(&mut *bodies, b1, b2,j)
            },
        };
        commands.entity(entity).insert(handle);
    }
}

pub(crate) fn remove_invalids_after_components_removed(
    mut commands: Commands,
    mut joints: ResMut<JointSet>,
    mut islands: ResMut<IslandManager>,
    mut bodies: ResMut<RigidBodySet>,
    joints_removed: RemovedComponents<Joint>,
    joint_handles: Query<&JointHandle>,
) {
    for entity in joints_removed.iter() {
        if let Ok(handle)= joint_handles.get(entity) {
            joints.remove(*handle, &mut islands, &mut *bodies, true); //delete the rapier joint
            commands.entity(entity).remove::<JointHandle>(); //and the handle component
        } //else assume it was already deleted so it doesn't matter.
    }
}

pub(crate) fn remove_invalids_after_component_changed(
    mut commands: Commands,
    mut joints: ResMut<JointSet>,
    mut islands: ResMut<IslandManager>,
    mut bodies: ResMut<RigidBodySet>,
    changed: Query<(Entity, &JointHandle), Changed<Joint>>
) {
// If you change the joint data, we delete the joint,
// then the next time the create() system runs, you'll get a new joint
// with the updated data
    for (entity, handle) in changed.iter() {
        joints.remove(*handle, &mut islands, &mut *bodies, true);
        commands.entity(entity).remove::<JointHandle>();
    }
}