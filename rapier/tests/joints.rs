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

fn mk_body(pos: Vec3) -> (Transform,GlobalTransform,RigidBody,CollisionShape) {
    (Transform::from_translation(pos),
     GlobalTransform::from_translation(pos),
     RigidBody::Dynamic,
     CollisionShape::Sphere {radius: 1.0}
    )
}

#[test]
fn joint_is_created_with_params() {
    let mut app = test_app();
    let e1 = app
        .world
        .spawn()
        .insert_bundle(mk_body(Vec3::ZERO))
        .id();
    let e2 = app
        .world
        .spawn()
        .insert_bundle(mk_body(Vec3::new(1.0,0.0,0.0)))
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
    if let utils::Joint {
        body1: b1,
        body2: b2,
        params: JointParams::BallJoint(ballparams), ..} = rapier_joint.unwrap() {
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


#[test]
fn fixed_joints_work() {
    let mut app = test_app();
    let e1 = app.world.spawn().insert_bundle(mk_body(Vec3::ZERO)).id();
    let e2 = app.world.spawn().insert_bundle(mk_body(Vec3::X * 5.0)).id();
    let joint = app.world.spawn().insert(heron_core::Joint {
        entity_1: e1,
        entity_2: e2,
        spec: JointSpec::Fixed {
            iso_1: Isometry::from_translation(Vec3::X),
            iso_2: Isometry::from_translation(- Vec3::X * 4.0),
        },
    }).id();

    app.update();
}

#[test]
fn revolute_joints_work() {
    let mut app = test_app();
    let e1 = app.world.spawn().insert_bundle(mk_body(Vec3::ZERO)).id();
    let e2 = app.world.spawn().insert_bundle(mk_body(Vec3::X * 5.0)).id();
    let joint = app.world.spawn().insert(heron_core::Joint {
        entity_1: e1,
        entity_2: e2,
        spec: JointSpec::Revolute {
            point_1: Vec3::ZERO,
            axis_1: Vec3::X,
            point_2: Vec3::ZERO,
            axis_2: Vec3::X,
        },
    }).id();

    app.update();
}


// Testing the functions for creating joints using the current GlobalTransform
// We need to make this a bit more like an actual Bevy app so we can get Querys

struct ReadyEvent(Entity,Entity);
fn setup_system(
    mut commands: Commands,
    mut events: EventWriter<ReadyEvent>
) {
    let e1 = commands.spawn_bundle(mk_body(Vec3::ZERO)).id();
    let e2 = commands.spawn_bundle(mk_body(Vec3::X)).id();
    events.send(ReadyEvent(e1,e2));
}

fn joint_system_fixed(
    query: Query<'_, &GlobalTransform>,
    mut commands: Commands,
    mut events: EventReader<ReadyEvent>
) {
    for ev in events.iter() {
        if let Some(joint) = heron_core::Joint::fixed_current(&query,ev.0,ev.1) {
            commands.spawn().insert(joint);
        } else {
            assert!(false,
                    "fixed joint creation failed, most likely due to lack of GlobalTransform");
        }
    }
}
fn fixed_test_app() -> App {
    let mut builder = App::build();
    builder
        .init_resource::<TypeRegistryArc>()
        .insert_resource(PhysicsSteps::every_frame(Duration::from_secs(1)))
        .add_event::<ReadyEvent>()
        .add_plugin(CorePlugin)
        .add_plugin(RapierPlugin)
        .add_startup_system(setup_system.system())
        .add_system(joint_system_fixed.system());
    builder.app
}
#[test]
fn fixed_joint_current() {
    let mut app = fixed_test_app();

    app.update();
    app.update();
}
