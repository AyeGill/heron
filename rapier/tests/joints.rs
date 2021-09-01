#![cfg(any(dim2,dim3))]
#![allow(unused_variables)]
use bevy::core::CorePlugin;
use bevy::prelude::*;
use bevy::reflect::TypeRegistryArc;

use std::{assert_eq, println};
use std::time::Duration;

use heron_core::*;
use heron_rapier::convert::{IntoBevy,IntoRapier};
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
     CollisionShape::Sphere {radius: 0.1} //important to avoid collisions unless we want them
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

        #[cfg(dim3)]
        {
            assert_eq!(ballparams.local_anchor1.into_bevy(), Vec3::ZERO);
            assert_eq!(ballparams.local_anchor2.into_bevy(), Vec3::new(-1.0,0.0,0.0));
        }
        #[cfg(dim2)]
        {
            assert_eq!(ballparams.local_anchor1.into_bevy(), Vec2::ZERO);
            assert_eq!(ballparams.local_anchor2.into_bevy(), Vec2::new(-1.0,0.0));
        }
    } else {
        assert!(false, "Joint didn't match specification");
    }
    check_joint_entity(&app, joint);
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
        }
    }).id();

    app.update();
}

#[test]
#[should_panic]
fn without_update_comparison_fails() {

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
        }
    }).id();

    check_joint_entity(&app,joint);
}

// Testing the functions for creating joints using the current GlobalTransform
// We need to make this a bit more like an actual Bevy app so we can get Querys

struct ReadyEvent(Entity,Entity);
struct TheJoint(Option<Entity>);
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
    mut events: EventReader<ReadyEvent>,
    mut joint_res : ResMut<TheJoint>
) {
    for ev in events.iter() {
        if let Some(joint) = heron_core::Joint::fixed_current(&query,ev.0,ev.1) {
            let id = commands.spawn().insert(joint).id();
            *joint_res = TheJoint(Some(id));
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
        .insert_resource(TheJoint(None))
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

    if let Some(TheJoint(Some(joint_entity))) = app.world.get_resource::<TheJoint>() {
        check_joint_entity(&app, *joint_entity);

        //also, this shouldn't lead to any movement - the joint constraints are
        // by construction already satisfied

        let joint = app.world.get::<heron_core::Joint>(*joint_entity).unwrap();
        println!("{:?}", joint);
        let e1 = (*joint).entity_1;
        let e2 = (*joint).entity_2;
        let t1 = app.world.get::<GlobalTransform>(e1).unwrap();
        let t2 = app.world.get::<GlobalTransform>(e2).unwrap();
        assert_eq!(t1.translation, Vec3::ZERO);
        assert_eq!(t2.translation, Vec3::X);
    }
}

fn check_joint_entity (
    app : &App,
    joint_entity: Entity,
) {
    let core_joint = app.world.get::<heron_core::Joint>(joint_entity)
        .expect("in check_joint_entity, given entity had no heron joint component!");

    let rapier_joint_handle = app.world.get::<JointHandle>(joint_entity)
        .expect("in check_joint_entity, given entity had no rapier JointHandle component!");
    let rapier_jointset = app.world.get_resource::<JointSet>()
        .expect("in check_joint_entity, given app had no JointSet resrouce");
    let rapier_joint = rapier_jointset.get(*rapier_joint_handle)
        .expect("in check_joint_entity, the JointHandle didn't exist in the JointSet");
    assert_eq!(rapier_joint.body1,
               *app.world.get::<RigidBodyHandle>(core_joint.entity_1)
               .expect("First entity had no RigidBodyHandle component."));
    assert_eq!(rapier_joint.body2,
               *app.world.get::<RigidBodyHandle>(core_joint.entity_2)
               .expect("Second entity had no RigidBodyHandle component."));
    compare_joint_params(rapier_joint.params, core_joint.spec);
}

fn compare_joint_params(
    rapier_params: JointParams,
    core_params: JointSpec
) {
    use heron_core::JointSpec::*;
    match core_params {
        Fixed {iso_1: i1, iso_2: i2} => {
            if let JointParams::FixedJoint(fixed_params) = rapier_params {
                assert_eq!(i1.into_rapier(),fixed_params.local_frame1);
                assert_eq!(i2.into_rapier(),fixed_params.local_frame2);
            } else {assert!(false,
                            "Rapier joint {:?} and core joint {:?} don't match",
                            MyParams(rapier_params),
                            core_params)}
        }
        Ball {point_1: p1, point_2: p2} => {
            if let JointParams::BallJoint(ball_params) = rapier_params {
                compare_points(p1,ball_params.local_anchor1);
                compare_points(p2,ball_params.local_anchor2);
            } else {assert!(false,
                            "Rapier joint {:?} and core joint {:?} don't match",
                            MyParams(rapier_params),
                            core_params);}
        }
        Prismatic {point_1:p1, point_2: p2, axis_1: a1, axis_2: a2} => {
            if let JointParams::PrismaticJoint(prismatic_params) = rapier_params {
                compare_points(p1,prismatic_params.local_anchor1);
                compare_points(p2,prismatic_params.local_anchor2);
                compare_axes(a1,prismatic_params.local_axis1());
                compare_axes(a2,prismatic_params.local_axis2());
            } else {assert!(false,
                            "Rapier joint {:?} and core joint {:?} don't match",
                            MyParams(rapier_params),
                            core_params);}
        }
        Revolute {point_1:p1, point_2: p2, axis_1: a1, axis_2: a2} => {
            #[cfg(dim3)]
            {
                if let JointParams::RevoluteJoint(revolute_params) = rapier_params {
                    compare_points(p1,revolute_params.local_anchor1);
                    compare_points(p2,revolute_params.local_anchor2);
                    compare_axes(a1,revolute_params.local_axis1);
                    compare_axes(a2,revolute_params.local_axis2);
                } else {assert!(false,
                                "Rapier joint {:?} and core joint {:?} don't match",
                                MyParams(rapier_params),
                                core_params)}
            }
            #[cfg(dim2)]
            {
                if let JointParams::BallJoint(ball_params) = rapier_params {
                    compare_points(p1, ball_params.local_anchor1);
                    compare_points(p2, ball_params.local_anchor2);
                } else {assert!(false,
                                "Rapier joint {:?} and core joint {:?} don't match",
                                MyParams(rapier_params),
                                core_params)}
            }
        }
    }

}

struct MyParams(JointParams);

use std::fmt;
impl fmt::Debug for MyParams {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        use JointParams::*;
        match self.0 {
            BallJoint(_) => write!(f, "Ball"),
            FixedJoint(_) => write!(f, "Fixed"),
            RevoluteJoint(_) => write!(f, "Revolute"),
            PrismaticJoint(_) => write!(f, "Primatic"),
        }
    }
}

//We *don't* want to enforce the invariant that the third dimension
// of the Heron data is zero if we're in 2d at this point.
fn compare_points(
    heron_pt: Vec3,
    rapier_pt: Point<f32>,
) {
    assert_eq!(IntoRapier::<Point<f32>>::into_rapier(heron_pt),rapier_pt);
}

fn compare_axes(
    heron_axis: Vec3,
    rapier_axis: Unit<Vector<f32>>,
) {
    assert_eq!(Unit::new_normalize(IntoRapier::<Vector<f32>>::into_rapier(heron_axis)), rapier_axis);
}
