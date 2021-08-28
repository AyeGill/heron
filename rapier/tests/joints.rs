use bevy::core::CorePlugin;
use bevy::prelude::*;
use bevy::reflect::TypeRegistryArc;
use rstest::rstest;

use std::time::Duration;

use heron_core::*;
use heron_rapier::convert::IntoBevy;
use heron_rapier::RapierPlugin;

use utils::*;

mod utils;

fn test_app() -> App {
    let mut builder = App::build();
    builder
        .init_resource::<TypeRegistryArc>()
        .insert_resource(PhysicsSteps::every_frame(Duration::from_secs(1)))
        .add_plugin(CorePlugin)
        .add_plugin(RapierPlugin);
    builder.app
}

#[test]
fn joint_is_created_with_params() {
    let mut app = test_app();
    let e1 = app
        .world
        .spawn()
        .insert_bundle((
            Transform::default(),
            GlobalTransform::default(),
            RigidBody::Dynamic,
            CollisionShape::Sphere {radius: 1.0}
        ))
        .id();
    let e2 = app
        .world
        .spawn()
        .insert_bundle((
            Transform::from_translation(Vec3::new(1.0,0.0,0.0)),
            GlobalTransform::from_translation(Vec3::new(1.0,0.0,0.0)),
            RigidBody::Dynamic,
            CollisionShape::Sphere {radius: 1.0}
        ))
        .id();
    let joint = app
        .world
        .spawn()
        .insert(heron_core::Joint::ball_arm(e1, e2, Vec3::new(-1.0,0.0,0.0)))
        .id();

    app.update();


    let joint_handle : Option<&JointHandle> = app.world.get(joint);
    assert!(joint_handle.is_some());
    let joint_set : Option<&JointSet> = app.world.get_resource();
    assert!(joint_set.is_some());
    let rapier_joint = joint_set.unwrap().get(*joint_handle.unwrap());
    assert!(rapier_joint.is_some());
    if let utils::Joint {body1: b1, body2: b2, params: JointParams::BallJoint(ballparams), ..} = (rapier_joint.unwrap()) {
        let e1_body = app.world.get::<RigidBodyHandle>(e1).unwrap();
        let e2_body = app.world.get::<RigidBodyHandle>(e2).unwrap();
        assert_eq!(e1_body, b1);
        assert_eq!(e2_body, b2);
        assert_eq!(ballparams.local_anchor1.into_bevy(), Vec3::ZERO);
        assert_eq!(ballparams.local_anchor2.into_bevy(), Vec3::new(-1.0,0.0,0.0));
    } else {
        assert!(false, "Joint didn't match specification");
    }
}
