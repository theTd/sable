use crate::scene::PhysicsScene;
use rapier3d::glamx::DVec3;
use rapier3d::math::{Pose3, Real};

/// sim 域坐标不变式：|translation| ≤ 该阈值（f64 下 ULP ~4.6e-13，纯保险）
pub const REBASE_THRESHOLD: Real = 2048.0;

/// 把整个场景平移 c（f64）：动态 body 与 static_mount wrt 减 c，origin 加 c。
/// ground 不动（identity）。锁序 sable write → sim write，调用时不得持有任一锁。
pub fn rebase_by(scene: &PhysicsScene, c: DVec3) {
    let mut sable = scene.sable_data.write().unwrap();
    let mut sim = scene.sim_data.write().unwrap();

    for h in sable.rigid_bodies.values() {
        let t = sim.rigid_body_set.get(*h).unwrap().translation();
        sim.rigid_body_set
            .get_mut(*h)
            .unwrap()
            .set_translation(t - c, false);
    }
    for h in sable.rope_map.point_handles() {
        let t = sim.rigid_body_set.get(h).unwrap().translation();
        sim.rigid_body_set
            .get_mut(h)
            .unwrap()
            .set_translation(t - c, false);
    }
    for info in sable.level_colliders.values() {
        if info.static_mount.is_some() {
            let co = sim.collider_set.get_mut(info.collider).unwrap();
            // 必须读 wrt：position() 是缓存 world pose，连续 rebase 在 pipeline
            // 同步前会重复从旧 world 减 c，丢失前一次平移。
            let pose = *co.position_wrt_parent().unwrap();
            co.set_position_wrt_parent(Pose3 {
                translation: pose.translation - c,
                rotation: pose.rotation,
            });
        }
    }
    sable.origin += c;
    drop(sim);
    drop(sable);
    scene.reported_collisions.rebase_ground_points(c);
}

/// step() 内周期触发：动态 body / rope point / static_mount wrt 的 translation
/// 包围盒最远分量 > REBASE_THRESHOLD 时，按包围盒中心 rebase。
pub fn rebase_if_needed(scene: &PhysicsScene) {
    let sable = scene.sable_data.write().unwrap();
    let sim = scene.sim_data.write().unwrap();

    let mut min = rapier3d::math::Vec3::splat(f64::MAX);
    let mut max = rapier3d::math::Vec3::splat(f64::MIN);
    let mut consider = |t: &rapier3d::math::Vec3| {
        min = min.min(*t);
        max = max.max(*t);
    };
    for h in sable.rigid_bodies.values() {
        consider(&sim.rigid_body_set.get(*h).unwrap().translation());
    }
    for h in sable.rope_map.point_handles() {
        consider(&sim.rigid_body_set.get(h).unwrap().translation());
    }
    for info in sable.level_colliders.values() {
        if info.static_mount.is_some() {
            consider(
                &sim.collider_set
                    .get(info.collider)
                    .unwrap()
                    .position_wrt_parent()
                    .unwrap()
                    .translation,
            );
        }
    }

    // min_element 覆盖负向极值；max_element 覆盖正向极值
    let m = min
        .min_element()
        .abs()
        .max(max.max_element().abs());
    if m <= REBASE_THRESHOLD {
        return;
    }
    let c = (min + max) * 0.5;
    if c.length() < 1e-3 {
        return; // 已居中但跨度仍超阈值（多簇病理场景）：避免每步无效重定基
    }
    drop(sim);
    drop(sable);
    rebase_by(scene, c);
}

/// 绝对位姿 writer 的 f64 精度闭环：若 |W − origin| 超阈值，先以 c = W − origin
/// rebase（新体恰好落在 0 附近，origin := W，f64 精确），再写入 sim 域位姿。
/// 调用时不得持有任一锁（writer 用独立 with_handle 作用域先行调用）。
pub fn ensure_origin_near(scene: &PhysicsScene, w: &DVec3) {
    {
        let sable = scene.sable_data.write().unwrap();
        let d = *w - sable.origin;
        if d.x.abs() <= REBASE_THRESHOLD
            && d.y.abs() <= REBASE_THRESHOLD
            && d.z.abs() <= REBASE_THRESHOLD
        {
            return;
        }
    }
    let sable = scene.sable_data.write().unwrap();
    let c = *w - sable.origin;
    drop(sable);
    rebase_by(scene, c);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collider::LevelCollider;
    use crate::dispatcher::SableDispatcher;
    use crate::event_handler::SableEventHandler;
    use crate::groups::LEVEL_GROUP;
    use crate::hooks::SablePhysicsHooks;
    use crate::joints::SableJointSet;
    use crate::rope::RopeMap;
    use crate::scene::{
        PhysicsScene, ReportedCollisionBuffer, SableManifoldInfoMap, SableSceneData,
        SimulationSceneData,
    };
    use crate::ActiveLevelColliderInfo;
    use rapier3d::dynamics::{
        CCDSolver, ImpulseJointSet, IslandManager, MultibodyJointSet, RigidBodyBuilder,
        RigidBodySet,
    };
    use rapier3d::geometry::{ColliderBuilder, ColliderSet, DefaultBroadPhase, NarrowPhase};
    use rapier3d::math::Vec3;
    use rapier3d::parry::query::{DefaultQueryDispatcher, QueryDispatcher};
    use rapier3d::pipeline::PhysicsPipeline;
    use rapier3d::prelude::SharedShape;
    use std::collections::HashMap;
    use std::sync::{Arc, RwLock};

    fn make_scene() -> PhysicsScene {
        let sable_data = Arc::new(RwLock::new(SableSceneData {
            main_level_chunks: HashMap::new(),
            octree_chunks: HashMap::new(),
            joint_set: SableJointSet::new(),
            rope_map: RopeMap::default(),
            level_colliders: HashMap::new(),
            rigid_bodies: HashMap::new(),
            origin: DVec3::ZERO,
        }));
        let manifold_info_map = Arc::new(SableManifoldInfoMap::default());
        let reported_collisions = Arc::new(ReportedCollisionBuffer::new());
        let dispatcher = SableDispatcher {
            sable_data: Arc::clone(&sable_data),
            manifold_info_map: Arc::clone(&manifold_info_map),
        };

        let mut scene = PhysicsScene {
            sim_data: RwLock::new(SimulationSceneData {
                pipeline: PhysicsPipeline::new(),
                rigid_body_set: RigidBodySet::new(),
                collider_set: ColliderSet::new(),
                island_manager: IslandManager::new(),
                broad_phase: DefaultBroadPhase::new(),
                narrow_phase: NarrowPhase::with_query_dispatcher(
                    dispatcher.chain(DefaultQueryDispatcher),
                ),
                impulse_joint_set: ImpulseJointSet::new(),
                multibody_joint_set: MultibodyJointSet::new(),
                ccd_solver: CCDSolver::new(),
                physics_hooks: SablePhysicsHooks {
                    sable_data: Arc::clone(&sable_data),
                    manifold_info_map: Arc::clone(&manifold_info_map),
                    current_step_vm: None,
                },
                event_handler: SableEventHandler {
                    reported_collisions: Arc::clone(&reported_collisions),
                },
            }),
            sable_data,
            ground_handle: None,
            reported_collisions,
            current_step_vm: None,
            gravity: Vec3::new(0.0, -9.8, 0.0),
            universal_drag: 0.0,
            manifold_info_map,
        };

        {
            let mut sim = scene.sim_data.write().unwrap();
            let ground = RigidBodyBuilder::fixed().build();
            let ground_handle = sim.rigid_body_set.insert(ground);
            scene.ground_handle.replace(ground_handle);

            let collider = ColliderBuilder::new(SharedShape::new(LevelCollider::new(None, true)))
                .collision_groups(LEVEL_GROUP)
                .build();
            // 对齐生产 initialize：ground collider 无 parent，独立于 ground body。
            sim.collider_set.insert(collider);
        }

        scene
    }

    fn insert_dynamic(scene: &PhysicsScene, id: usize, t: Vec3) -> rapier3d::prelude::RigidBodyHandle {
        let mut sim = scene.sim_data.write().unwrap();
        let mut sable = scene.sable_data.write().unwrap();
        let handle = sim
            .rigid_body_set
            .insert(RigidBodyBuilder::dynamic().translation(t).build());
        sable.rigid_bodies.insert(id, handle);
        handle
    }

    fn insert_static_mount(scene: &PhysicsScene, id: usize, wrt: Vec3) {
        let mut sim = scene.sim_data.write().unwrap();
        let mut sable = scene.sable_data.write().unwrap();
        let sim = &mut *sim;
        let mount = sim
            .rigid_body_set
            .insert(RigidBodyBuilder::kinematic_position_based().build());
        let collider = ColliderBuilder::new(SharedShape::new(LevelCollider::new(
            Some(id),
            false,
        )))
        .collision_groups(LEVEL_GROUP)
        .build();
        let collider_handle = sim.collider_set.insert_with_parent(
            collider,
            mount,
            &mut sim.rigid_body_set,
        );
        sim.collider_set
            .get_mut(collider_handle)
            .unwrap()
            .set_position_wrt_parent(Pose3 {
                translation: wrt,
                rotation: Default::default(),
            });
        let mut info = ActiveLevelColliderInfo::new(collider_handle);
        info.static_mount = Some(mount);
        sable.level_colliders.insert(id, info);
    }

    #[test]
    fn ensure_origin_near_far_w_sets_origin_exactly() {
        let scene = make_scene();
        let w = DVec3::new(40000.137, 64.05, 40000.271);
        ensure_origin_near(&scene, &w);

        let sable = scene.sable_data.read().unwrap();
        assert_eq!(sable.origin, w);
        let residual = w - sable.origin;
        assert!(residual.length() < 1e-3);
    }

    #[test]
    fn rebase_by_shifts_dynamic_rope_and_static_mount_keeps_ground() {
        let scene = make_scene();
        let dyn_h = insert_dynamic(&scene, 1, Vec3::new(100.0, 20.0, 30.0));
        let (rope_p0, rope_p1) = {
            let mut sim = scene.sim_data.write().unwrap();
            let mut sable = scene.sable_data.write().unwrap();
            let p0 = sim
                .rigid_body_set
                .insert(RigidBodyBuilder::dynamic().translation(Vec3::new(10.0, 0.0, 0.0)).build());
            let p1 = sim
                .rigid_body_set
                .insert(RigidBodyBuilder::dynamic().translation(Vec3::new(11.0, 0.0, 0.0)).build());
            sable.rope_map.insert_strand_for_test(vec![p0, p1]);
            (p0, p1)
        };
        insert_static_mount(&scene, 2, Vec3::new(50.0, 0.0, 0.0));

        let origin_old = scene.sable_data.read().unwrap().origin;
        let t_dyn_old = {
            let sim = scene.sim_data.read().unwrap();
            sim.rigid_body_set.get(dyn_h).unwrap().translation()
        };
        let ground_old = {
            let sim = scene.sim_data.read().unwrap();
            sim.rigid_body_set
                .get(scene.ground_handle.unwrap())
                .unwrap()
                .translation()
        };
        let mount_old = {
            let sim = scene.sim_data.read().unwrap();
            let sable = scene.sable_data.read().unwrap();
            sim.collider_set
                .get(sable.level_colliders[&2].collider)
                .unwrap()
                .position_wrt_parent()
                .unwrap()
                .translation
        };

        let c = DVec3::new(5.0, 1.0, 2.0);
        rebase_by(&scene, c);

        let sable = scene.sable_data.read().unwrap();
        let sim = scene.sim_data.read().unwrap();
        assert_eq!(sable.origin, origin_old + c);

        let t_dyn_new = sim.rigid_body_set.get(dyn_h).unwrap().translation();
        let recovered = sable.origin + t_dyn_new;
        let expected = origin_old + t_dyn_old;
        assert!((recovered - expected).length() < 1e-3);

        let ground_new = sim
            .rigid_body_set
            .get(scene.ground_handle.unwrap())
            .unwrap()
            .translation();
        assert_eq!(ground_new, ground_old);
        assert_eq!(ground_new, Vec3::ZERO);

        let t0 = sim.rigid_body_set.get(rope_p0).unwrap().translation();
        let t1 = sim.rigid_body_set.get(rope_p1).unwrap().translation();
        assert!((t0 - DVec3::new(5.0, -1.0, -2.0)).length() < 1e-3);
        assert!((t1 - DVec3::new(6.0, -1.0, -2.0)).length() < 1e-3);

        let mount_new = sim
            .collider_set
            .get(sable.level_colliders[&2].collider)
            .unwrap()
            .position_wrt_parent()
            .unwrap()
            .translation;
        assert!((mount_new - (mount_old - c)).length() < 1e-3);
    }

    #[test]
    fn rebase_by_shifts_buffered_ground_collision_points() {
        use crate::ReportedCollision;
        let scene = make_scene();
        scene.reported_collisions.borrow_mut().push(ReportedCollision {
            body_a: None,
            body_b: Some(1),
            local_point_a: DVec3::new(10.0, 20.0, 30.0),
            local_point_b: DVec3::new(1.0, 2.0, 3.0),
            local_normal_a: DVec3::Y,
            local_normal_b: -DVec3::Y,
            force_amount: 1.0,
        });
        rebase_by(&scene, DVec3::new(5.0, 1.0, 2.0));
        let buf = scene.reported_collisions.borrow_mut();
        // ground 侧 O 帧点减 c；动态侧 COM 相对量不变
        assert!((buf[0].local_point_a - DVec3::new(5.0, 19.0, 28.0)).length() < 1e-9);
        assert_eq!(buf[0].local_point_b, DVec3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn consecutive_rebase_preserves_static_mount_wrt() {
        let scene = make_scene();
        insert_static_mount(&scene, 1, Vec3::new(100.0, 20.0, 30.0));

        rebase_by(&scene, DVec3::new(10.0, 0.0, 0.0));
        rebase_by(&scene, DVec3::new(5.0, 1.0, 2.0));

        let sim = scene.sim_data.read().unwrap();
        let sable = scene.sable_data.read().unwrap();
        let wrt = sim
            .collider_set
            .get(sable.level_colliders[&1].collider)
            .unwrap()
            .position_wrt_parent()
            .unwrap()
            .translation;
        // 100-10-5=85, 20-0-1=19, 30-0-2=28
        assert!((wrt - DVec3::new(85.0, 19.0, 28.0)).length() < 1e-3);
        assert_eq!(sable.origin, DVec3::new(15.0, 1.0, 2.0));
    }

    #[test]
    fn rebase_if_needed_triggers_and_guards() {
        let scene = make_scene();
        let h = insert_dynamic(&scene, 1, Vec3::new(3000.0, 0.0, 0.0));
        rebase_if_needed(&scene);
        {
            let sim = scene.sim_data.read().unwrap();
            let t = sim.rigid_body_set.get(h).unwrap().translation();
            assert!(t.length() <= REBASE_THRESHOLD);
            let sable = scene.sable_data.read().unwrap();
            assert!((sable.origin - DVec3::new(3000.0, 0.0, 0.0)).length() < 1e-3);
        }

        // at threshold: no trigger
        {
            let mut sim = scene.sim_data.write().unwrap();
            sim.rigid_body_set
                .get_mut(h)
                .unwrap()
                .set_translation(Vec3::new(REBASE_THRESHOLD, 0.0, 0.0), false);
            let mut sable = scene.sable_data.write().unwrap();
            sable.origin = DVec3::ZERO;
        }
        let origin_before = scene.sable_data.read().unwrap().origin;
        rebase_if_needed(&scene);
        assert_eq!(scene.sable_data.read().unwrap().origin, origin_before);

        // centered dual cluster ±3000: c≈0 → guard skips
        {
            let mut sim = scene.sim_data.write().unwrap();
            sim.rigid_body_set
                .get_mut(h)
                .unwrap()
                .set_translation(Vec3::new(3000.0, 0.0, 0.0), false);
        }
        let _ = insert_dynamic(&scene, 2, Vec3::new(-3000.0, 0.0, 0.0));
        {
            let mut sable = scene.sable_data.write().unwrap();
            sable.origin = DVec3::ZERO;
        }
        let origin_before = scene.sable_data.read().unwrap().origin;
        let t1_before = {
            let sim = scene.sim_data.read().unwrap();
            sim.rigid_body_set.get(h).unwrap().translation()
        };
        rebase_if_needed(&scene);
        assert_eq!(scene.sable_data.read().unwrap().origin, origin_before);
        let t1_after = {
            let sim = scene.sim_data.read().unwrap();
            sim.rigid_body_set.get(h).unwrap().translation()
        };
        assert_eq!(t1_before, t1_after);
    }
}
