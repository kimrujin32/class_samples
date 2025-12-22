# Copyright (c) 2022-2025
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations
import torch
import numpy as np

from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.torch.transformations import tf_combine, tf_inverse, tf_vector
from pxr import UsdGeom

import isaaclab.sim as sim_utils
from isaaclab.actuators.actuator_cfg import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import sample_uniform


@configclass
class RandomGraspOnTableEnvCfg(DirectRLEnvCfg):
    # ===== env =====
    episode_length_s = 10.0
    decimation = 2
    action_space = 10
    observation_space = 28
    state_space = 0

    # ===== sim =====
    sim: SimulationCfg = SimulationCfg(
        dt=1/120,
        render_interval=decimation,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )

    # ===== scene =====
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4096, env_spacing=3.0, replicate_physics=True, clone_in_fabric=True
    )

    # ===== robot: e0509 (6 + 4 DOF) =====
    robot = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path="/home/robot/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/e0509_random_grasp/usd/e0509_with_rh.usd",
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,              # 중력 켜서 테이블 위로 자연 안착
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=12,
                solver_velocity_iteration_count=1,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "joint_1": 0.0, "joint_2": 0.0, "joint_3": 0.0,
                "joint_4": 0.0, "joint_5": 0.0, "joint_6": 0.0,
                "rh_l1": 0.0, "rh_l2": 0.0, "rh_r1": 0.0, "rh_r2": 0.0,
            },
            # ★ 테이블 상면 z=0.7보다 2cm 위에서 스폰 → 충돌 없이 자연 안착
            pos=(0.307, 0.4, 0.75),
            rot=(0.0, 0.0, 0.0, 1.0),
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=["joint_[1-6]"],
                effort_limit_sim=184.0, stiffness=1000, damping=50,
            ),
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=["rh_l[12]", "rh_r[12]"],
                effort_limit_sim=200.0, stiffness=2000, damping=100,
            ),
        },
    )

    # ===== table: custom USD at z = 0.7 (static) =====
    table = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Table",
        spawn=sim_utils.UsdFileCfg(
            usd_path="/home/robot/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/e0509_random_grasp/usd/cerea_ws_Wls.usd",
            activate_contact_sensors=False,  # ★ 중요: RigidBodyAPI 자동 적용
            # ★ 테이블 월드 포즈: z=0.7 (상면 기준 모델이면 정확히 올라감)
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,     # 중력 영향 없음
                kinematic_enabled=True,   # 안전하게 고정(옵션)
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(),  # 충돌 감지 활성화
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.7),
            rot=(0.0, 0.0, 0.0, 1.0),
        ),
    )

    # ===== object: cube on table =====
    object = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Object",
        spawn=sim_utils.CuboidCfg(
            size=(0.05, 0.05, 0.05),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=False, disable_gravity=False,
                enable_gyroscopic_forces=True,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
                sleep_threshold=0.005,
                stabilization_threshold=0.0025,
                max_depenetration_velocity=1000.0,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.2, 0.2)),
        ),
        # ★ 테이블 위에 놓이도록 초기 z = 0.75 (상면 0.7 + 여유 5cm)
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0.0, 0.75)),
    )

    # ===== ground =====
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )

    # ===== randomization ranges =====
    object_pos_x_range = (0.3, 0.7)
    object_pos_y_range = (-0.3, 0.3)
    # ★ 리셋에서도 테이블 위로 스폰되도록 기본 z 고정
    object_pos_z = 0.75

    # ===== control scales =====
    action_scale = 7.5
    dof_velocity_scale = 0.1

    # ===== rewards =====
    dist_reward_scale = 2.0
    rot_reward_scale = 1.0
    grasp_reward_scale = 5.0
    lift_reward_scale = 10.0
    action_penalty_scale = 0.01
    finger_reward_scale = 2.0


class RandomGraspOnTableEnv(DirectRLEnv):
    cfg: RandomGraspOnTableEnvCfg

    def __init__(self, cfg: RandomGraspOnTableEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        def get_env_local_pose(env_pos: torch.Tensor, xformable: UsdGeom.Xformable, device: torch.device):
            world_transform = xformable.ComputeLocalToWorldTransform(0)
            world_pos = world_transform.ExtractTranslation()
            world_quat = world_transform.ExtractRotationQuat()
            px = world_pos[0] - env_pos[0]
            py = world_pos[1] - env_pos[1]
            pz = world_pos[2] - env_pos[2]
            qx = world_quat.imaginary[0]; qy = world_quat.imaginary[1]
            qz = world_quat.imaginary[2]; qw = world_quat.real
            return torch.tensor([px, py, pz, qw, qx, qy, qz], device=device)

        self.dt = self.cfg.sim.dt * self.cfg.decimation

        # DOF limits
        self.robot_dof_lower_limits = self._robot.data.soft_joint_pos_limits[0, :, 0].to(self.device)
        self.robot_dof_upper_limits = self._robot.data.soft_joint_pos_limits[0, :, 1].to(self.device)

        # speed scales: fingers slower
        self.robot_dof_speed_scales = torch.ones_like(self.robot_dof_lower_limits)
        for name in ["rh_l1", "rh_l2", "rh_r1", "rh_r2"]:
            idxs = self._robot.find_joints(name)[0]
            if len(idxs) > 0:
                self.robot_dof_speed_scales[idxs] = 0.1

        self.robot_dof_targets = torch.zeros((self.num_envs, self._robot.num_joints), device=self.device)

        # TCP transform (tool0 -> gripper center)
        stage = get_current_stage()
        hand_prim_path = self._find_body_prim_path(0, "tool0")
        if hand_prim_path is None:
            raise RuntimeError("EE prim 'tool0' not found.")
        lfinger_prim_path = self._find_body_prim_path(0, "rh_p12_rn_l2")
        rfinger_prim_path = self._find_body_prim_path(0, "rh_p12_rn_r2")
        if lfinger_prim_path is None or rfinger_prim_path is None:
            raise RuntimeError("Finger body prims not found: rh_p12_rn_l2 / rh_p12_rn_r2")

        hand_pose = get_env_local_pose(self.scene.env_origins[0], UsdGeom.Xformable(stage.GetPrimAtPath(hand_prim_path)), self.device)
        lfinger_pose = get_env_local_pose(self.scene.env_origins[0], UsdGeom.Xformable(stage.GetPrimAtPath(lfinger_prim_path)), self.device)
        rfinger_pose = get_env_local_pose(self.scene.env_origins[0], UsdGeom.Xformable(stage.GetPrimAtPath(rfinger_prim_path)), self.device)

        finger_pose = torch.zeros(7, device=self.device)
        finger_pose[0:3] = (lfinger_pose[0:3] + rfinger_pose[0:3]) / 2.0
        finger_pose[3:7] = lfinger_pose[3:7]

        hand_pose_inv_rot, hand_pose_inv_pos = tf_inverse(hand_pose[3:7], hand_pose[0:3])
        robot_local_grasp_rot, robot_local_grasp_pos = tf_combine(
            hand_pose_inv_rot, hand_pose_inv_pos, finger_pose[3:7], finger_pose[0:3]
        )
        # e0509 그리퍼 형상 오프셋 (여유)
        robot_local_grasp_pos += torch.tensor([0.0, 0.1, 0.0], device=self.device, dtype=torch.float32)

        self.robot_local_grasp_pos = robot_local_grasp_pos.repeat((self.num_envs, 1))
        self.robot_local_grasp_rot = robot_local_grasp_rot.repeat((self.num_envs, 1))

        self.gripper_forward_axis = torch.tensor([0.0, 0.0, 1.0], device=self.device, dtype=torch.float32).repeat((self.num_envs, 1))
        self.gripper_up_axis = torch.tensor([0.0, 1.0, 0.0], device=self.device, dtype=torch.float32).repeat((self.num_envs, 1))
        self.object_down_axis = torch.tensor([0.0, 0.0, -1.0], device=self.device, dtype=torch.float32).repeat((self.num_envs, 1))

        self.hand_link_idx = self._robot.find_bodies("tool0")[0][0]
        self.left_finger_link_idx = self._robot.find_bodies("rh_p12_rn_l2")[0][0]
        self.right_finger_link_idx = self._robot.find_bodies("rh_p12_rn_r2")[0][0]

        self.robot_grasp_rot = torch.zeros((self.num_envs, 4), device=self.device)
        self.robot_grasp_pos = torch.zeros((self.num_envs, 3), device=self.device)

        self.external_sensor_data = torch.zeros((self.num_envs, 2), device=self.device)
        self.initial_object_height = torch.zeros(self.num_envs, device=self.device)

    def _setup_scene(self):
        # spawn assets
        self._robot = Articulation(self.cfg.robot)
        self._object = RigidObject(self.cfg.object)

        # register
        self.scene.articulations["robot"] = self._robot
        self.scene.rigid_objects["object"] = self._object

        # terrain
        self.cfg.terrain.num_envs = self.scene.cfg.num_envs
        self.cfg.terrain.env_spacing = self.scene.cfg.env_spacing
        self._terrain = self.cfg.terrain.class_type(self.cfg.terrain)

        # clone
        self.scene.clone_environments(copy_from_source=False)

        # ★ 워크스테이션: clone 후 각 환경에 spawn (kinematic 고정)
        for env_idx in range(self.scene.cfg.num_envs):
            prim_path = f"/World/envs/env_{env_idx}/Table"
            self.cfg.table.spawn.func(prim_path, self.cfg.table.spawn)

        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[self.cfg.terrain.prim_path])

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    def _pre_physics_step(self, actions: torch.Tensor):
        self.actions = actions.clone().clamp(-1.0, 1.0)
        targets = self.robot_dof_targets + self.robot_dof_speed_scales * self.dt * self.actions * self.cfg.action_scale
        self.robot_dof_targets[:] = torch.clamp(targets, self.robot_dof_lower_limits, self.robot_dof_upper_limits)

    def _apply_action(self):
        self._robot.set_joint_position_target(self.robot_dof_targets)

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        object_height = self._object.data.root_pos_w[:, 2]
        lifted = (object_height - self.initial_object_height) > 0.10
        truncated = self.episode_length_buf >= self.max_episode_length - 1
        return lifted, truncated

    def _get_rewards(self) -> torch.Tensor:
        self._compute_intermediate_values()
        lpos = self._robot.data.body_pos_w[:, self.left_finger_link_idx]
        rpos = self._robot.data.body_pos_w[:, self.right_finger_link_idx]
        opos = self._object.data.root_pos_w
        orot = self._object.data.root_quat_w

        return self._compute_rewards(
            self.actions, self.robot_grasp_pos, opos, self.robot_grasp_rot, orot,
            lpos, rpos, self.gripper_forward_axis, self.object_down_axis, self.gripper_up_axis,
            self.num_envs, self.cfg.dist_reward_scale, self.cfg.rot_reward_scale,
            self.cfg.grasp_reward_scale, self.cfg.lift_reward_scale, self.cfg.action_penalty_scale,
            self.cfg.finger_reward_scale, self._robot.data.joint_pos, self.initial_object_height
        )

    def _reset_idx(self, env_ids: torch.Tensor | None):
        super()._reset_idx(env_ids)

        # robot reset
        joint_pos = self._robot.data.default_joint_pos[env_ids].clone()
        joint_pos += sample_uniform(-0.05, 0.05, (len(env_ids), self._robot.num_joints), self.device)
        joint_pos = torch.clamp(joint_pos, self.robot_dof_lower_limits, self.robot_dof_upper_limits)
        joint_vel = torch.zeros_like(joint_pos)
        self._robot.set_joint_position_target(joint_pos, env_ids=env_ids)
        self._robot.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)

        # object randomization (★ 항상 테이블 위 z로)
        object_pos = torch.zeros((len(env_ids), 3), device=self.device)
        object_pos[:, 0] = sample_uniform(self.cfg.object_pos_x_range[0], self.cfg.object_pos_x_range[1], (len(env_ids),), self.device)
        object_pos[:, 1] = sample_uniform(self.cfg.object_pos_y_range[0], self.cfg.object_pos_y_range[1], (len(env_ids),), self.device)
        object_pos[:, 2] = self.cfg.object_pos_z  # ★ 0.75
        
        object_pos += self.scene.env_origins[env_ids]

        object_rot = torch.zeros((len(env_ids), 4), device=self.device); object_rot[:, 3] = 1.0
        object_vel = torch.zeros((len(env_ids), 6), device=self.device)

        self._object.write_root_pose_to_sim(torch.cat([object_pos, object_rot], dim=-1), env_ids=env_ids)
        self._object.write_root_velocity_to_sim(object_vel, env_ids=env_ids)

        self.initial_object_height[env_ids] = object_pos[:, 2]
        self._compute_intermediate_values(env_ids)

    def _get_observations(self) -> dict:
        dof_pos_scaled = (
            2.0 * (self._robot.data.joint_pos - self.robot_dof_lower_limits)
            / (self.robot_dof_upper_limits - self.robot_dof_lower_limits) - 1.0
        )
        object_pos = self._object.data.root_pos_w
        to_target = object_pos - self.robot_grasp_pos
        obs = torch.cat((
            dof_pos_scaled,
            self._robot.data.joint_vel * self.cfg.dof_velocity_scale,
            to_target,
            object_pos,
            self.external_sensor_data,
        ), dim=-1)
        return {"policy": torch.clamp(obs, -5.0, 5.0)}

    def _compute_intermediate_values(self, env_ids: torch.Tensor | None = None):
        if env_ids is None:
            env_ids = self._robot._ALL_INDICES
        hand_pos = self._robot.data.body_pos_w[env_ids, self.hand_link_idx]
        hand_rot = self._robot.data.body_quat_w[env_ids, self.hand_link_idx]
        self.robot_grasp_rot[env_ids], self.robot_grasp_pos[env_ids] = tf_combine(
            hand_rot, hand_pos, self.robot_local_grasp_rot[env_ids], self.robot_local_grasp_pos[env_ids]
        )

    def _compute_rewards(
        self, actions, franka_grasp_pos, object_pos, franka_grasp_rot, object_rot,
        franka_lfinger_pos, franka_rfinger_pos, gripper_forward_axis, object_down_axis, gripper_up_axis,
        num_envs, dist_reward_scale, rot_reward_scale, grasp_reward_scale, lift_reward_scale,
        action_penalty_scale, finger_reward_scale, joint_positions, initial_object_height,
    ):
        d = torch.norm(franka_grasp_pos - object_pos, p=2, dim=-1)
        dist_reward = (1.0 / (1.0 + d**2))**2
        dist_reward = torch.where(d <= 0.02, dist_reward * 2, dist_reward)

        axis1 = tf_vector(franka_grasp_rot, gripper_forward_axis)
        axis2 = tf_vector(object_rot, object_down_axis)
        dot1 = torch.bmm(axis1.view(num_envs,1,3), axis2.view(num_envs,3,1)).squeeze(-1).squeeze(-1)
        rot_reward = torch.sign(dot1) * dot1**2

        action_penalty = torch.sum(actions**2, dim=-1)

        lfinger_dist = torch.norm(franka_lfinger_pos - object_pos, p=2, dim=-1)
        rfinger_dist = torch.norm(franka_rfinger_pos - object_pos, p=2, dim=-1)
        finger_dist_penalty = -(lfinger_dist + rfinger_dist)

        obj_h = object_pos[:, 2]
        lift_amount = torch.clamp(obj_h - initial_object_height, min=0.0)
        lift_reward = lift_amount

        rewards = (
            dist_reward_scale * dist_reward
            + rot_reward_scale * rot_reward
            + finger_reward_scale * finger_dist_penalty
            + lift_reward_scale * lift_reward
            - action_penalty_scale * action_penalty
        )

        avg_finger_dist = (lfinger_dist + rfinger_dist) / 2.0
        rewards = torch.where(avg_finger_dist < 0.03, rewards + grasp_reward_scale * 0.5, rewards)

        gripper_width = joint_positions[:, 6] + joint_positions[:, 7] + joint_positions[:, 8] + joint_positions[:, 9]
        grasped = (avg_finger_dist < 0.03) & (gripper_width < 0.08)
        rewards = torch.where(grasped, rewards + grasp_reward_scale, rewards)

        rewards = torch.where(lift_amount > 0.02, rewards + 2.0, rewards)
        rewards = torch.where(lift_amount > 0.05, rewards + 3.0, rewards)
        rewards = torch.where(lift_amount > 0.10, rewards + 5.0, rewards)

        self.extras["log"] = {
            "dist_reward": (dist_reward_scale * dist_reward).mean(),
            "rot_reward": (rot_reward_scale * rot_reward).mean(),
            "lift_reward": (lift_reward_scale * lift_reward).mean(),
            "action_penalty": (-action_penalty_scale * action_penalty).mean(),
            "left_finger_distance": lfinger_dist.mean(),
            "right_finger_distance": rfinger_dist.mean(),
            "finger_dist_penalty": (finger_reward_scale * finger_dist_penalty).mean(),
            "lift_amount": lift_amount.mean(),
            "grasp_success_rate": grasped.float().mean(),
        }
        return rewards

    def _find_body_prim_path(self, env_index: int, body_name: str) -> str | None:
        stage = get_current_stage()
        base = f"/World/envs/env_{env_index}/Robot"
        base_prim = stage.GetPrimAtPath(base)
        if not base_prim or not base_prim.IsValid():
            return None
        stack = [base_prim]
        while stack:
            p = stack.pop()
            if not p or not p.IsValid():
                continue
            if p.GetName() == body_name:
                return p.GetPath().pathString
            for c in p.GetChildren():
                stack.append(c)
        return None
