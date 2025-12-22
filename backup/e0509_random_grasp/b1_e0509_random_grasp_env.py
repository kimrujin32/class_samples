# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
e0509 로봇을 사용한 랜덤 그립 환경

이 환경은 6축 팔 + 4축 그리퍼(총 10 DOF)를 가진 e0509 로봇이
랜덤 위치에 놓인 물체를 잡아 들어올리는 것을 학습하는 강화학습 환경입니다.

주요 특징:
- 4096개의 병렬 환경 (GPU 가속)
- 매 에피소드마다 물체 위치 랜덤화
- 외부 센서 데이터(depth, pixel) 입력 지원
- Dense reward: 거리, 방향, 그립, 리프트 등 다양한 보상
"""

from __future__ import annotations

import torch
import numpy as np

# USD 스테이지 및 변환 유틸리티
from isaacsim.core.utils.stage import get_current_stage  # USD 씬 그래프 접근
from isaacsim.core.utils.torch.transformations import tf_combine, tf_inverse, tf_vector  # 좌표계 변환
from pxr import UsdGeom  # USD 지오메트리 API

# Isaac Lab 시뮬레이션 관련
import isaaclab.sim as sim_utils
from isaaclab.actuators.actuator_cfg import ImplicitActuatorCfg  # 액추에이터 설정
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg  # 로봇/물체 에셋
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg  # 강화학습 환경 베이스 클래스
from isaaclab.scene import InteractiveSceneCfg  # 씬 구성
from isaaclab.sim import SimulationCfg  # 시뮬레이션 파라미터
from isaaclab.terrains import TerrainImporterCfg  # 지형(바닥) 설정
from isaaclab.utils import configclass  # 설정 클래스 데코레이터
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR  # 기본 에셋 경로
from isaaclab.utils.math import sample_uniform  # 랜덤 샘플링


@configclass
class FrankaRandomGraspEnvCfg(DirectRLEnvCfg):
    """e0509 로봇 그립 환경 설정 클래스

    이 클래스는 강화학습 환경의 모든 파라미터를 정의합니다.
    """

    # ========================================
    # 환경 기본 설정
    # ========================================
    episode_length_s = 10.0  # 에피소드 길이 (초 단위) - 120Hz × 10초 = 1200 physics steps
    decimation = 2  # 물리 스텝 건너뛰기 - 2 physics steps마다 1번 제어 (60Hz 제어 주파수)
    action_space = 10  # 액션 차원: 6(팔) + 4(그리퍼) = 10 DOF
    # 관측 차원 (총 28):
    #   - robot_dof(10): 로봇 조인트 위치 (정규화됨)
    #   - robot_dof_vel(10): 로봇 조인트 속도
    #   - to_target(3): 그리퍼에서 물체까지의 벡터 (x, y, z)
    #   - object_pos(3): 물체의 절대 위치
    #   - external_sensor_data(2): 외부 센서 (depth, pixel_x)
    observation_space = 28
    state_space = 0  # 사용하지 않음 (관측과 상태가 동일)

    # ========================================
    # 시뮬레이션 설정
    # ========================================
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 120,  # 물리 시뮬레이션 타임스텝 (120Hz) - 높을수록 정확하지만 느림
        render_interval=decimation,  # 렌더링 주기 = decimation과 동일
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",  # 마찰력 계산 방식 (두 물체의 마찰계수를 곱함)
            restitution_combine_mode="multiply",  # 반발력 계산 방식
            static_friction=1.0,  # 정지 마찰계수 (1.0 = 미끄러지기 어려움)
            dynamic_friction=1.0,  # 운동 마찰계수
            restitution=0.0,  # 반발계수 (0 = 완전 비탄성 충돌, 튕기지 않음)
        ),
    )

    # ========================================
    # 씬(장면) 설정
    # ========================================
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4096,  # 병렬 환경 개수 (GPU 메모리에 따라 조절)
        env_spacing=3.0,  # 환경 간 간격 (미터) - 충돌 방지
        replicate_physics=True,  # 각 환경마다 독립적인 물리 시뮬레이션
        clone_in_fabric=True  # Fabric (병렬화 엔진) 사용
    )

    # ========================================
    # 로봇 설정: e0509 (6축 팔 + 4축 그리퍼 = 10 DOF)
    # ========================================
    robot = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",  # USD 씬에서 로봇 경로 (.*는 모든 환경)
        spawn=sim_utils.UsdFileCfg(
            usd_path="/home/user/Downloads/e0509_with_rh.usd",  # e0509 로봇 USD 파일 경로
            activate_contact_sensors=False,  # 접촉 센서 비활성화 (필요시 활성화)
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,  # 중력 적용
                max_depenetration_velocity=5.0,  # 물체가 서로 겹쳤을 때 분리 속도 (m/s)
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,  # 로봇 자체 충돌 비활성화 (팔과 팔 사이)
                solver_position_iteration_count=12,  # 위치 솔버 반복 횟수 (높을수록 정확)
                solver_velocity_iteration_count=1  # 속도 솔버 반복 횟수
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            # 초기 조인트 위치 (라디안) - 모든 조인트를 0 위치로 설정
            joint_pos={
                # 6축 팔 조인트
                "joint_1": 0.0,  # 베이스 회전
                "joint_2": 0.0,  # 어깨 pitch
                "joint_3": 0.0,  # 어깨 roll
                "joint_4": 0.0,  # 팔꿈치
                "joint_5": 0.0,  # 손목 pitch
                "joint_6": 0.0,  # 손목 roll
                # 4축 그리퍼 조인트 (rh = Robot Hand)
                "rh_l1": 0.0,  # 왼쪽 손가락 1번째 조인트
                "rh_l2": 0.0,  # 왼쪽 손가락 2번째 조인트
                "rh_r1": 0.0,  # 오른쪽 손가락 1번째 조인트
                "rh_r2": 0.0,  # 오른쪽 손가락 2번째 조인트
            },
            pos=(1.0, 0.0, 0.0),  # 로봇 베이스 위치 (x=1m 앞에 배치, 물체는 x=0.5m에 생성)
            rot=(0.0, 0.0, 0.0, 1.0),  # 로봇 베이스 회전 (쿼터니언: x, y, z, w) - 회전 없음
        ),
        actuators={
            # 팔 액추에이터: PD 제어기 파라미터
            "arm": ImplicitActuatorCfg(
                joint_names_expr=["joint_[1-6]"],  # 정규표현식: joint_1 ~ joint_6
                effort_limit_sim=184.0,  # 최대 토크 (N·m) - 실제 로봇 스펙에 맞춤
                stiffness=1000,  # P 게인 (위치 제어 강성) - 높을수록 목표 위치로 빠르게 이동
                damping=50,  # D 게인 (댐핑) - 높을수록 진동 감소
            ),
            # 그리퍼 액추에이터: 팔보다 더 높은 stiffness (정밀 제어)
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=["rh_l[12]", "rh_r[12]"],  # rh_l1, rh_l2, rh_r1, rh_r2
                effort_limit_sim=200.0,  # 최대 토크
                stiffness=2e3,  # 2000 - 팔의 2배 (그리퍼는 정밀해야 함)
                damping=1e2,  # 100 - 팔의 2배
            ),
        },
    )

    # ========================================
    # 잡을 물체 설정 (큐브)
    # ========================================
    object = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Object",  # 각 환경의 물체 경로
        spawn=sim_utils.CuboidCfg(
            size=(0.05, 0.05, 0.05),  # 큐브 크기 5cm × 5cm × 5cm
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=False,  # 물리 시뮬레이션 적용 (True면 움직이지 않음)
                disable_gravity=False,  # 중력 적용
                enable_gyroscopic_forces=True,  # 자이로스코프 효과 (회전 중 안정성)
                solver_position_iteration_count=8,  # 위치 솔버 반복 (로봇보다 낮게 설정)
                solver_velocity_iteration_count=0,  # 속도 솔버 (0 = 사용 안함)
                sleep_threshold=0.005,  # 물체가 멈춘 것으로 간주하는 속도 (m/s)
                stabilization_threshold=0.0025,  # 안정화 임계값
                max_depenetration_velocity=1000.0,  # 최대 분리 속도
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.1),  # 질량 100g (가벼운 물체)
            collision_props=sim_utils.CollisionPropertiesCfg(),  # 충돌 속성 기본값
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.2, 0.2)),  # 빨간색
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.5, 0.0, 0.05)),  # 초기 위치 (리셋시 랜덤화됨)
    )

    # ========================================
    # 지형(바닥) 설정
    # ========================================
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",  # 바닥 경로
        terrain_type="plane",  # 평평한 바닥
        collision_group=-1,  # 충돌 그룹 (모든 물체와 충돌)
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",  # 마찰력 계산 방식
            restitution_combine_mode="multiply",  # 반발력 계산 방식
            static_friction=1.0,  # 정지 마찰계수 (미끄러지지 않는 바닥)
            dynamic_friction=1.0,  # 운동 마찰계수
            restitution=0.0,  # 반발계수 0 (바닥에서 튕기지 않음)
        ),
    )

    # ========================================
    # 물체 랜덤화 범위
    # ========================================
    object_pos_x_range = (0.3, 0.7)  # X축: 로봇 앞쪽 30~70cm (로봇 도달 범위)
    object_pos_y_range = (-0.3, 0.3)  # Y축: 좌우 ±30cm
    object_pos_z = 0.05  # Z축: 바닥에서 5cm 높이 (테이블 위)

    # ========================================
    # 제어 스케일링 파라미터
    # ========================================
    action_scale = 7.5  # 액션 스케일 (클수록 빠르게 움직임, 너무 크면 불안정)
    dof_velocity_scale = 0.1  # 속도 관측 스케일 (속도를 정규화)

    # ========================================
    # 보상 가중치 (reward shaping)
    # ========================================
    dist_reward_scale = 2.0  # 거리 보상 가중치 (그리퍼-물체 거리)
    rot_reward_scale = 1.0  # 방향 보상 가중치 (그리퍼가 위에서 접근)
    grasp_reward_scale = 5.0  # 그립 보상 가중치 (물체를 잡았을 때)
    lift_reward_scale = 10.0  # 리프트 보상 가중치 (가장 중요! 물체를 들어올림)
    action_penalty_scale = 0.01  # 액션 페널티 (큰 움직임 억제, 에너지 절약)
    finger_reward_scale = 2.0  # 손가락 보상 가중치 (손가락이 물체에 가까이)


class FrankaRandomGraspEnv(DirectRLEnv):
    """e0509 로봇의 랜덤 그립 학습 환경

    이 환경은 e0509 로봇이 랜덤 위치의 물체를 잡아 들어올리는 것을 학습합니다.
    외부 센서 데이터(depth, pixel 좌표) 입력을 지원하며,
    매 에피소드마다 물체 위치를 랜덤화하여 일반화 능력을 향상시킵니다.
    """

    cfg: FrankaRandomGraspEnvCfg

    def __init__(self, cfg: FrankaRandomGraspEnvCfg, render_mode: str | None = None, **kwargs):
        """환경 초기화 함수

        Args:
            cfg: 환경 설정 객체
            render_mode: 렌더링 모드 ("human", "rgb_array", None 등)
            **kwargs: 추가 인자
        """
        # 부모 클래스 초기화 (씬 생성, 로봇/물체 스폰 등)
        super().__init__(cfg, render_mode, **kwargs)

        def get_env_local_pose(env_pos: torch.Tensor, xformable: UsdGeom.Xformable, device: torch.device):
            """USD 프림의 포즈를 환경 로컬 좌표계로 변환

            월드 좌표계의 포즈를 각 환경의 원점 기준 상대 좌표로 변환합니다.
            4096개 환경이 각각 다른 위치에 있으므로, 로컬 좌표계가 필요합니다.

            Args:
                env_pos: 환경 원점 위치 (3,)
                xformable: USD Xformable 객체 (포즈 정보 포함)
                device: 텐서 디바이스 (cpu or cuda)

            Returns:
                포즈 텐서 [px, py, pz, qw, qx, qy, qz] (7,) - 위치(3) + 쿼터니언(4)
            """
            # USD에서 월드 변환 행렬 가져오기
            world_transform = xformable.ComputeLocalToWorldTransform(0)
            world_pos = world_transform.ExtractTranslation()  # 위치 (x, y, z)
            world_quat = world_transform.ExtractRotationQuat()  # 쿼터니언 회전

            # 환경 원점 기준 상대 위치 계산
            px = world_pos[0] - env_pos[0]
            py = world_pos[1] - env_pos[1]
            pz = world_pos[2] - env_pos[2]

            # 쿼터니언 (w, x, y, z)
            qx = world_quat.imaginary[0]
            qy = world_quat.imaginary[1]
            qz = world_quat.imaginary[2]
            qw = world_quat.real

            return torch.tensor([px, py, pz, qw, qx, qy, qz], device=device)

        # 제어 타임스텝 계산: dt × decimation
        # 예: 1/120 × 2 = 1/60초 = 0.0167초 (60Hz 제어 주파수)
        self.dt = self.cfg.sim.dt * self.cfg.decimation

        # ========================================
        # 로봇 DOF (Degree of Freedom) 제한 설정
        # ========================================
        # USD 파일에서 각 조인트의 최소/최대 각도 가져오기 (라디안)
        # shape: (10,) - 10개 조인트 각각의 제한
        self.robot_dof_lower_limits = self._robot.data.soft_joint_pos_limits[0, :, 0].to(device=self.device)
        self.robot_dof_upper_limits = self._robot.data.soft_joint_pos_limits[0, :, 1].to(device=self.device)

        # DOF 속도 스케일: 조인트별로 얼마나 빨리 움직일지 결정
        # 기본값 1.0 (팔 조인트), 그리퍼는 0.1로 설정 (10배 느림)
        self.robot_dof_speed_scales = torch.ones_like(self.robot_dof_lower_limits)
        # 그리퍼 4축을 느리게 설정 (섬세한 제어 필요)
        for name in ["rh_l1", "rh_l2", "rh_r1", "rh_r2"]:
            idxs = self._robot.find_joints(name)[0]  # 조인트 인덱스 찾기
            if len(idxs) > 0:
                self.robot_dof_speed_scales[idxs] = 0.1  # 그리퍼는 10배 느리게

        # 조인트 목표 위치 버퍼: 각 환경의 10개 조인트 목표값
        # shape: (4096, 10)
        
        self.robot_dof_targets = torch.zeros((self.num_envs, self._robot.num_joints), device=self.device)

        # ========================================
        # TCP (Tool Center Point) 계산
        # ========================================
        # TCP = 그리퍼 중심점 (실제 작업이 일어나는 위치)
        # tool0 (EE)와 손가락 끝 사이의 변환을 계산해야 함

        stage = get_current_stage()  # USD 스테이지 가져오기

        # USD 트리에서 body 이름으로 프림 경로 동적 검색
        # (USD 구조가 복잡할 때 정확한 경로를 찾기 위함)
        hand_prim_path = self._find_body_prim_path(env_index=0, body_name="tool0")
        if hand_prim_path is None:
            raise RuntimeError("EE prim 'tool0' not found under /World/envs/env_0/Robot.")

        # 왼쪽/오른쪽 손가락 바디 경로 찾기
        lfinger_prim_path = self._find_body_prim_path(0, "rh_p12_rn_l2")
        rfinger_prim_path = self._find_body_prim_path(0, "rh_p12_rn_r2")
        if lfinger_prim_path is None or rfinger_prim_path is None:
            raise RuntimeError("Finger body prims not found: rh_p12_rn_l2 / rh_p12_rn_r2 under env_0/Robot.")

        # 각 바디의 초기 포즈를 환경 로컬 좌표계로 가져오기
        hand_pose = get_env_local_pose(
            self.scene.env_origins[0],  # 첫 번째 환경 원점
            UsdGeom.Xformable(stage.GetPrimAtPath(hand_prim_path)),  # tool0 프림
            self.device,
        )
        lfinger_pose = get_env_local_pose(
            self.scene.env_origins[0],
            UsdGeom.Xformable(stage.GetPrimAtPath(lfinger_prim_path)),
            self.device,
        )
        rfinger_pose = get_env_local_pose(
            self.scene.env_origins[0],
            UsdGeom.Xformable(stage.GetPrimAtPath(rfinger_prim_path)),
            self.device,
        )

        # 두 손가락의 중심점 계산 (그리퍼 중심 = TCP)
        finger_pose = torch.zeros(7, device=self.device)  # [위치(3), 쿼터니언(4)]
        finger_pose[0:3] = (lfinger_pose[0:3] + rfinger_pose[0:3]) / 2.0  # 중간점
        finger_pose[3:7] = lfinger_pose[3:7]  # 회전은 왼쪽 손가락 기준

        # tool0 좌표계 → 그리퍼 중심 좌표계 변환 계산
        # 1) tool0의 역변환 구하기 (월드 → tool0)
        hand_pose_inv_rot, hand_pose_inv_pos = tf_inverse(hand_pose[3:7], hand_pose[0:3])

        # 2) tool0 기준 그리퍼 중심의 상대 변환 계산
        robot_local_grasp_pose_rot, robot_local_pose_pos = tf_combine(
            hand_pose_inv_rot, hand_pose_inv_pos,  # tool0 → 로컬
            finger_pose[3:7], finger_pose[0:3]  # 그리퍼 중심 포즈
        )

        # 3) TCP 오프셋 추가 조정 (그리퍼 끝까지의 거리)
        # Y축으로 10cm 이동 (e0509 그리퍼의 형상에 맞춤)
        robot_local_pose_pos += torch.tensor([0, 0.1, 0], device=self.device)

        # 4) 모든 환경에 복제 (4096개 환경 모두 동일한 변환 사용)
        self.robot_local_grasp_pos = robot_local_pose_pos.repeat((self.num_envs, 1))  # (4096, 3)
        self.robot_local_grasp_rot = robot_local_grasp_pose_rot.repeat((self.num_envs, 1))  # (4096, 4)

        # ========================================
        # 방향 축 정의 (보상 계산용)
        # ========================================
        # 그리퍼와 물체의 방향을 비교하여 올바른 접근 방향에 보상
        # 그리퍼 앞방향 축 (로컬 좌표계 Z축 = 그리퍼가 향하는 방향)
        self.gripper_forward_axis = torch.tensor([0, 0, 1], device=self.device, dtype=torch.float32).repeat(
            (self.num_envs, 1)  # (4096, 3)
        )
        # 그리퍼 위방향 축 (로컬 좌표계 Y축)
        self.gripper_up_axis = torch.tensor([0, 1, 0], device=self.device, dtype=torch.float32).repeat(
            (self.num_envs, 1)  # (4096, 3)
        )
        # 물체 아래방향 축 (그리퍼가 위에서 접근하도록 유도)
        self.object_down_axis = torch.tensor([0, 0, -1], device=self.device, dtype=torch.float32).repeat(
            (self.num_envs, 1)  # (4096, 3)
        )

        # ========================================
        # 링크(바디) 인덱스 저장
        # ========================================
        # Isaac Lab은 내부적으로 바디를 인덱스로 관리
        # 매번 이름으로 찾는 것은 비효율적이므로 초기화 시 인덱스를 저장
        hand_idx = self._robot.find_bodies("tool0")[0]  # tool0 바디 인덱스 리스트
        self.hand_link_idx = hand_idx[0]  # 첫 번째 (유일한) 인덱스
        self.left_finger_link_idx = self._robot.find_bodies("rh_p12_rn_l2")[0][0]  # 왼쪽 손가락
        self.right_finger_link_idx = self._robot.find_bodies("rh_p12_rn_r2")[0][0]  # 오른쪽 손가락

        # ========================================
        # 그립 포즈 버퍼 (매 스텝 업데이트됨)
        # ========================================
        # 현재 그리퍼 중심(TCP)의 월드 좌표계 포즈
        self.robot_grasp_rot = torch.zeros((self.num_envs, 4), device=self.device)  # 회전 (쿼터니언)
        self.robot_grasp_pos = torch.zeros((self.num_envs, 3), device=self.device)  # 위치 (x, y, z)

        # ========================================
        # 외부 센서 데이터 버퍼
        # ========================================
        # 외부 비전 시스템에서 입력받을 데이터
        # Format: [depth, pixel_x] - 각 환경마다 2개 값
        # 사용법: env.set_external_sensor_data(depth=..., pixel_coords=...)
        self.external_sensor_data = torch.zeros((self.num_envs, 2), device=self.device)

        # ========================================
        # 초기 물체 높이 (리프트 감지용)
        # ========================================
        # 에피소드 시작 시 물체의 초기 Z 좌표 저장
        # 나중에 "얼마나 들어올렸는지" 계산할 때 사용
        self.initial_object_height = torch.zeros(self.num_envs, device=self.device)

    def _setup_scene(self):
        """씬 설정 함수 (로봇, 물체, 지형 생성)

        이 함수는 __init__ 전에 부모 클래스에서 자동으로 호출됩니다.
        USD 씬에 로봇과 물체를 스폰하고, 4096개 환경으로 복제합니다.
        """
        # 로봇과 물체 생성 (config 기반)
        self._robot = Articulation(self.cfg.robot)  # e0509 로봇
        self._object = RigidObject(self.cfg.object)  # 큐브 물체

        # 씬에 에셋 등록 (이름으로 접근 가능)
        self.scene.articulations["robot"] = self._robot
        self.scene.rigid_objects["object"] = self._object

        # 지형(바닥) 설정 업데이트
        self.cfg.terrain.num_envs = self.scene.cfg.num_envs  # 4096
        self.cfg.terrain.env_spacing = self.scene.cfg.env_spacing  # 3.0m
        self._terrain = self.cfg.terrain.class_type(self.cfg.terrain)  # 평면 생성

        # 환경 복제: env_0를 복제하여 env_1 ~ env_4095 생성
        # copy_from_source=False: 각 환경이 독립적 (True면 인스턴싱)
        self.scene.clone_environments(copy_from_source=False)

        # CPU 시뮬레이션에서는 충돌 필터링 명시적으로 설정 필요
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[self.cfg.terrain.prim_path])

        # 조명 추가 (렌더링용)
        light_cfg = sim_utils.DomeLightCfg(
            intensity=2000.0,  # 밝기
            color=(0.75, 0.75, 0.75)  # 흰색 조명
        )
        light_cfg.func("/World/Light", light_cfg)

    def set_external_sensor_data(self, depth: np.ndarray | None = None, pixel_coords: np.ndarray | None = None):
        """외부 센서 데이터 설정 (옵션)

        외부 비전 시스템(카메라, depth 센서 등)에서 데이터를 받아 관측에 포함시킵니다.
        예: 물체 감지 모델의 출력(depth, pixel 좌표)을 policy에 전달

        Args:
            depth: 각 환경의 depth 값, shape (num_envs,)
            pixel_coords: 각 환경의 픽셀 좌표 (x, y), shape (num_envs, 2)

        Example:
            >>> env.set_external_sensor_data(
            ...     depth=np.array([0.5, 0.6, ...]),  # 4096개
            ...     pixel_coords=np.array([[320, 240], [330, 250], ...])  # (4096, 2)
            ... )
        """
        if depth is not None:
            # Numpy → Torch 변환 후 GPU로 이동
            depth_tensor = torch.from_numpy(depth).to(self.device).float()
            self.external_sensor_data[:, 0] = depth_tensor  # 첫 번째 열에 depth 저장

        if pixel_coords is not None:
            pixel_tensor = torch.from_numpy(pixel_coords).to(self.device).float()
            # 픽셀 좌표를 [-1, 1] 범위로 정규화 (640×480 이미지 기준)
            # 중심(320, 240)을 0으로, 가장자리를 ±1로 매핑
            normalized_x = (pixel_tensor[:, 0] / 320.0) - 1.0  # X 좌표
            normalized_y = (pixel_tensor[:, 1] / 240.0) - 1.0  # Y 좌표 (현재 미사용)
            self.external_sensor_data[:, 1] = normalized_x  # 두 번째 열에 pixel_x 저장
            # 더 많은 센서 데이터가 필요하면 external_sensor_data 크기를 늘리세요

    # ========================================
    # 물리 시뮬레이션 전 호출 (액션 적용)
    # ========================================

    def _pre_physics_step(self, actions: torch.Tensor):
        """물리 스텝 전 액션 처리

        Policy에서 받은 액션을 조인트 목표 위치로 변환합니다.

        Args:
            actions: Policy 출력 액션, shape (num_envs, 10), 범위 [-1, 1]

        동작 원리:
            1. 액션을 [-1, 1]로 클램핑 (안전)
            2. 현재 목표에서 액션만큼 변화량 추가
            3. 조인트 제한 내로 클램핑
        """
        # 1. 액션 안전 범위로 제한
        self.actions = actions.clone().clamp(-1.0, 1.0)

        # 2. 조인트 목표 위치 계산
        # 변화량 = speed_scale × dt × action × action_scale
        # 예: 팔 조인트에서 action=1.0일 때
        #     Δangle = 1.0 × 0.0167 × 1.0 × 7.5 = 0.125 rad/step
        targets = self.robot_dof_targets + \
                  self.robot_dof_speed_scales * self.dt * self.actions * self.cfg.action_scale

        # 3. 조인트 제한 범위 내로 클램핑
        self.robot_dof_targets[:] = torch.clamp(
            targets,
            self.robot_dof_lower_limits,  # 최소 각도
            self.robot_dof_upper_limits  # 최대 각도
        )

    def _apply_action(self):
        """계산된 목표를 시뮬레이터에 적용

        _pre_physics_step에서 계산한 목표 위치를 실제 로봇에 전달합니다.
        시뮬레이터 내부의 PD 제어기가 이 목표를 추종합니다.
        """
        self._robot.set_joint_position_target(self.robot_dof_targets)

    # ========================================
    # 물리 시뮬레이션 후 호출 (보상, 종료 조건 등)
    # ========================================

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        """에피소드 종료 조건 확인

        Returns:
            terminated: 성공적으로 종료 (물체를 10cm 이상 들어올림), shape (num_envs,)
            truncated: 최대 길이 도달로 강제 종료, shape (num_envs,)

        종료 조건:
            - terminated=True: 물체를 초기 높이에서 10cm 이상 들어올림 (성공!)
            - truncated=True: 에피소드 최대 길이 도달 (실패 or 진행중)
        """
        # 현재 물체 높이 (Z 좌표)
        object_height = self._object.data.root_pos_w[:, 2]  # (4096,)

        # 성공 조건: 초기 높이 대비 10cm 이상 상승
        lifted = (object_height - self.initial_object_height) > 0.1  # (4096,) bool

        # 최대 길이 도달 (10초 = 600 제어 스텝)
        truncated = self.episode_length_buf >= self.max_episode_length - 1  # (4096,) bool

        return lifted, truncated

    def _get_rewards(self) -> torch.Tensor:
        """보상 계산 (매 스텝 호출)

        물리 시뮬레이션 후 현재 상태를 기반으로 보상을 계산합니다.
        Dense reward를 사용하여 학습을 안정화합니다.

        Returns:
            rewards: 각 환경의 보상, shape (num_envs,)
        """
        # 그리퍼 TCP 위치 업데이트 (tool0 위치 → 그리퍼 중심)
        self._compute_intermediate_values()

        # 필요한 상태 정보 수집
        robot_left_finger_pos = self._robot.data.body_pos_w[:, self.left_finger_link_idx]  # (4096, 3)
        robot_right_finger_pos = self._robot.data.body_pos_w[:, self.right_finger_link_idx]  # (4096, 3)
        object_pos = self._object.data.root_pos_w  # (4096, 3)
        object_rot = self._object.data.root_quat_w  # (4096, 4)

        # 실제 보상 계산 함수 호출
        return self._compute_rewards(
            self.actions,  # 이번 스텝 액션
            self.robot_grasp_pos,  # 그리퍼 중심 위치
            object_pos,  # 물체 위치
            self.robot_grasp_rot,  # 그리퍼 중심 회전
            object_rot,  # 물체 회전
            robot_left_finger_pos,  # 왼쪽 손가락 위치
            robot_right_finger_pos,  # 오른쪽 손가락 위치
            self.gripper_forward_axis,  # 그리퍼 앞방향 (로컬)
            self.object_down_axis,  # 물체 아래방향 (로컬)
            self.gripper_up_axis,  # 그리퍼 위방향 (로컬)
            self.num_envs,  # 환경 개수
            self.cfg.dist_reward_scale,  # 거리 보상 가중치
            self.cfg.rot_reward_scale,  # 방향 보상 가중치
            self.cfg.grasp_reward_scale,  # 그립 보상 가중치
            self.cfg.lift_reward_scale,  # 리프트 보상 가중치
            self.cfg.action_penalty_scale,  # 액션 페널티 가중치
            self.cfg.finger_reward_scale,  # 손가락 보상 가중치
            self._robot.data.joint_pos,  # 조인트 위치 (그리퍼 닫힘 감지용)
            self.initial_object_height,  # 초기 물체 높이 (리프트 량 계산용)
        )

    def _reset_idx(self, env_ids: torch.Tensor | None):
        """에피소드 리셋 (종료된 환경들만)

        종료된 환경들의 로봇과 물체를 초기 상태로 되돌립니다.
        물체 위치는 랜덤화하여 다양한 상황에서 학습하도록 합니다.

        Args:
            env_ids: 리셋할 환경 인덱스, shape (n,), n <= 4096
                     None이면 모든 환경 리셋
        """
        # 부모 클래스의 리셋 로직 (에피소드 카운터 등 초기화)
        super()._reset_idx(env_ids)

        # ========================================
        # 로봇 상태 리셋
        # ========================================
        # 기본 조인트 위치에서 시작
        joint_pos = self._robot.data.default_joint_pos[env_ids].clone()

        # 작은 랜덤 노이즈 추가: 매번 똑같은 자세에서 시작하면 과적합 위험
        # ±0.05 rad (약 ±3도) 랜덤 변화
        joint_pos += sample_uniform(
            -0.05,  # 최소값
            0.05,  # 최대값
            (len(env_ids), self._robot.num_joints),  # (n, 10)
            self.device,
        )

        # 조인트 제한 범위 내로 클램핑
        joint_pos = torch.clamp(joint_pos, self.robot_dof_lower_limits, self.robot_dof_upper_limits)

        # 초기 속도는 0
        joint_vel = torch.zeros_like(joint_pos)

        # 시뮬레이터에 적용
        self._robot.set_joint_position_target(joint_pos, env_ids=env_ids)  # 목표 위치 설정
        self._robot.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)  # 실제 상태 쓰기

        # ========================================
        # 물체 위치 랜덤화
        # ========================================
        object_pos = torch.zeros((len(env_ids), 3), device=self.device)

        # X축: 로봇 앞쪽 30~70cm 범위에서 랜덤
        object_pos[:, 0] = sample_uniform(
            self.cfg.object_pos_x_range[0],  # 0.3m
            self.cfg.object_pos_x_range[1],  # 0.7m
            (len(env_ids),),
            self.device,
        )

        # Y축: 좌우 ±30cm 범위에서 랜덤
        object_pos[:, 1] = sample_uniform(
            self.cfg.object_pos_y_range[0],  # -0.3m
            self.cfg.object_pos_y_range[1],  # 0.3m
            (len(env_ids),),
            self.device,
        )

        # Z축: 고정 (테이블 위 5cm)
        object_pos[:, 2] = self.cfg.object_pos_z  # 0.05m

        # 물체 회전: 회전 없음 (쿼터니언 identity)
        object_rot = torch.zeros((len(env_ids), 4), device=self.device)
        object_rot[:, 3] = 1.0  # w=1, x=y=z=0

        # 물체 속도: 0 (정지 상태)
        object_vel = torch.zeros((len(env_ids), 6), device=self.device)  # [선속도(3), 각속도(3)]

        # 시뮬레이터에 적용
        self._object.write_root_pose_to_sim(
            torch.cat([object_pos, object_rot], dim=-1),  # (n, 7)
            env_ids=env_ids
        )
        self._object.write_root_velocity_to_sim(object_vel, env_ids=env_ids)

        # 초기 높이 저장 (나중에 리프트 량 계산용)
        self.initial_object_height[env_ids] = object_pos[:, 2]

        # 그리퍼 TCP 위치 업데이트 (관측 계산에 필요)
        self._compute_intermediate_values(env_ids)

    def _get_observations(self) -> dict:
        """현재 관측 반환 (매 스텝 호출)

        Policy에 입력될 관측 벡터를 생성합니다.
        모든 값은 정규화되어 [-5, 5] 범위로 클램핑됩니다.

        Returns:
            관측 딕셔너리 {"policy": obs}, obs shape (num_envs, 28)
                - dof_pos_scaled (10): 조인트 위치 [-1, 1]
                - dof_vel_scaled (10): 조인트 속도 (스케일됨)
                - to_target (3): 그리퍼→물체 벡터
                - object_pos (3): 물체 절대 위치
                - external_sensor_data (2): depth, pixel_x
        """
        # ========================================
        # 1. 조인트 위치 정규화 [-1, 1]
        # ========================================
        # 원리: (값 - 최소) / (최대 - 최소) = [0, 1]
        #       × 2 - 1 = [-1, 1]
        dof_pos_scaled = (
            2.0
            * (self._robot.data.joint_pos - self.robot_dof_lower_limits)
            / (self.robot_dof_upper_limits - self.robot_dof_lower_limits)
            - 1.0
        )  # (4096, 10)

        # ========================================
        # 2. 그리퍼에서 물체까지의 벡터
        # ========================================
        object_pos = self._object.data.root_pos_w  # (4096, 3)
        to_target = object_pos - self.robot_grasp_pos  # (4096, 3)
        # 양수면 물체가 그리퍼보다 앞/위/오른쪽에 있음

        # ========================================
        # 3. 관측 벡터 조립
        # ========================================
        obs = torch.cat(
            (
                dof_pos_scaled,  # 10차원: 조인트 위치
                self._robot.data.joint_vel * self.cfg.dof_velocity_scale,  # 10차원: 조인트 속도 (×0.1 스케일)
                to_target,  # 3차원: 목표 방향
                object_pos,  # 3차원: 물체 위치
                self.external_sensor_data,  # 2차원: 외부 센서
            ),
            dim=-1,
        )  # (4096, 28)

        # 극단값 클램핑 (학습 안정화)
        return {"policy": torch.clamp(obs, -5.0, 5.0)}

    # ========================================
    # 보조 메서드 (Auxiliary Methods)
    # ========================================

    def _compute_intermediate_values(self, env_ids: torch.Tensor | None = None):
        """그리퍼 TCP 위치 계산 (tool0 → 그리퍼 중심)

        tool0 (EE)의 현재 포즈에서 그리퍼 중심(TCP)의 월드 좌표를 계산합니다.
        초기화 시 계산한 로컬 변환(robot_local_grasp_*)을 사용합니다.

        Args:
            env_ids: 계산할 환경 인덱스, None이면 모든 환경

        Note:
            이 함수는 보상 계산과 관측 생성 전에 반드시 호출되어야 합니다.
        """
        # 환경 인덱스가 없으면 모든 환경 처리
        if env_ids is None:
            env_ids = self._robot._ALL_INDICES  # [0, 1, 2, ..., 4095]

        # tool0 (EE)의 현재 월드 좌표
        hand_pos = self._robot.data.body_pos_w[env_ids, self.hand_link_idx]  # (n, 3)
        hand_rot = self._robot.data.body_quat_w[env_ids, self.hand_link_idx]  # (n, 4)

        # tool0 포즈 + 로컬 오프셋 = 그리퍼 중심 포즈 (월드 좌표)
        # tf_combine(rot1, pos1, rot2, pos2) = rot1 ⊗ rot2, rot1 * pos2 + pos1
        self.robot_grasp_rot[env_ids], self.robot_grasp_pos[env_ids] = tf_combine(
            hand_rot,  # tool0 회전
            hand_pos,  # tool0 위치
            self.robot_local_grasp_rot[env_ids],  # tool0 → 그리퍼 상대 회전
            self.robot_local_grasp_pos[env_ids],  # tool0 → 그리퍼 상대 위치
        )

    def _compute_rewards(
        self,
        actions,  # (num_envs, 10): 이번 스텝 액션
        franka_grasp_pos,  # (num_envs, 3): 그리퍼 중심 위치
        object_pos,  # (num_envs, 3): 물체 위치
        franka_grasp_rot,  # (num_envs, 4): 그리퍼 중심 회전 (쿼터니언)
        object_rot,  # (num_envs, 4): 물체 회전
        franka_lfinger_pos,  # (num_envs, 3): 왼쪽 손가락 위치
        franka_rfinger_pos,  # (num_envs, 3): 오른쪽 손가락 위치
        gripper_forward_axis,  # (num_envs, 3): 그리퍼 앞방향 축 (로컬 Z)
        object_down_axis,  # (num_envs, 3): 물체 아래방향 축 (로컬 -Z)
        gripper_up_axis,  # (num_envs, 3): 그리퍼 위방향 축 (로컬 Y)
        num_envs,  # int: 환경 개수
        dist_reward_scale,  # float: 거리 보상 가중치
        rot_reward_scale,  # float: 방향 보상 가중치
        grasp_reward_scale,  # float: 그립 보상 가중치
        lift_reward_scale,  # float: 리프트 보상 가중치
        action_penalty_scale,  # float: 액션 페널티 가중치
        finger_reward_scale,  # float: 손가락 보상 가중치
        joint_positions,  # (num_envs, 10): 조인트 위치 (그리퍼 닫힘 감지)
        initial_object_height,  # (num_envs,): 초기 물체 높이
    ):
        """보상 계산 함수 (Dense Reward)

        여러 보상 요소를 조합하여 최종 보상을 계산합니다.
        Dense reward를 사용하여 학습을 안정화하고 가이드합니다.

        보상 구성:
            1. 거리 보상: 그리퍼가 물체에 가까워지도록
            2. 방향 보상: 그리퍼가 위에서 접근하도록
            3. 손가락 보상: 손가락이 물체 양쪽에 가도록
            4. 리프트 보상: 물체를 들어올리도록 (가장 중요!)
            5. 액션 페널티: 불필요한 큰 움직임 억제
            6. 그립 보너스: 물체를 성공적으로 잡았을 때
            7. 리프트 보너스: 특정 높이 도달 시 추가 보상
        """

        # ========================================
        # 1️⃣ 거리 보상 (Distance Reward)
        # ========================================
        # 그리퍼 중심 - 물체 중심 유클리드 거리
        d = torch.norm(franka_grasp_pos - object_pos, p=2, dim=-1)  # (num_envs,)

        # 역제곱 함수: 1 / (1 + d²)
        # d=0 → 1.0, d=0.1 → 0.5, d=0.5 → 0.16
        dist_reward = 1.0 / (1.0 + d**2)
        dist_reward *= dist_reward  # 제곱해서 가까울수록 더 큰 보상

        # 2cm 이내면 2배 보너스 (매우 가까이 접근)
        dist_reward = torch.where(d <= 0.02, dist_reward * 2, dist_reward)

        # ========================================
        # 2️⃣ 방향 보상 (Orientation Reward)
        # ========================================
        # 그리퍼가 위에서 아래로 접근하도록 유도
        # 로컬 축을 월드 좌표로 변환
        axis1 = tf_vector(franka_grasp_rot, gripper_forward_axis)  # 그리퍼 앞방향 (월드)
        axis2 = tf_vector(object_rot, object_down_axis)  # 물체 아래방향 (월드)

        # 내적 계산: cos(θ) = -1 (반대) ~ 1 (같은 방향)
        dot1 = (
            torch.bmm(axis1.view(num_envs, 1, 3), axis2.view(num_envs, 3, 1)).squeeze(-1).squeeze(-1)
        )

        # 부호 유지하면서 제곱: sign(x) × x²
        # 평행하면 +1, 수직이면 0, 반대면 -1
        rot_reward = torch.sign(dot1) * dot1**2

        # ========================================
        # 3️⃣ 액션 페널티 (Action Penalty)
        # ========================================
        # 액션 크기의 제곱합 (L2 norm)
        # 큰 움직임을 억제하여 부드러운 동작 유도
        action_penalty = torch.sum(actions**2, dim=-1)  # (num_envs,)

        # ========================================
        # 4️⃣ 손가락 보상 (Finger Reward)
        # ========================================
        # 두 손가락이 모두 물체에 가까워지도록
        lfinger_dist = torch.norm(franka_lfinger_pos - object_pos, p=2, dim=-1)  # 왼쪽 손가락-물체 거리
        rfinger_dist = torch.norm(franka_rfinger_pos - object_pos, p=2, dim=-1)  # 오른쪽 손가락-물체 거리
        finger_dist_penalty = -(lfinger_dist + rfinger_dist)  # 마이너스: 가까울수록 높음

        # ========================================
        # 5️⃣ 리프트 보상 (Lift Reward) - 가장 중요!
        # ========================================
        # 물체를 초기 높이 대비 얼마나 들어올렸는지
        object_height = object_pos[:, 2]  # 현재 Z 좌표
        lift_amount = torch.clamp(object_height - initial_object_height, min=0.0)  # 음수 방지
        lift_reward = lift_amount  # 10cm 들면 +0.1 보상

        # ========================================
        # 기본 보상 조합
        # ========================================
        rewards = (
            dist_reward_scale * dist_reward  # 2.0 × [0~1] = 0~2
            + rot_reward_scale * rot_reward  # 1.0 × [-1~1] = -1~1
            + finger_reward_scale * finger_dist_penalty  # 2.0 × [-1~0] = -2~0
            + lift_reward_scale * lift_reward  # 10.0 × [0~0.1] = 0~1 (가장 큰 보상!)
            - action_penalty_scale * action_penalty  # 0.01 × [0~10] = 0~0.1 (작은 페널티)
        )

        # ========================================
        # 6️⃣ 그립 준비 보너스
        # ========================================
        # 평균 손가락 거리 < 3cm이면 보너스 (그립 준비 단계)
        avg_finger_dist = (lfinger_dist + rfinger_dist) / 2.0
        rewards = torch.where(
            avg_finger_dist < 0.03,
            rewards + grasp_reward_scale * 0.5,  # +2.5 보너스
            rewards
        )

        # ========================================
        # 7️⃣ 성공적인 그립 보너스
        # ========================================
        # 조건: 손가락이 물체에 가까이 AND 그리퍼가 닫혀있음
        # e0509: joint 6,7,8,9가 그리퍼 (rh_l1, rh_l2, rh_r1, rh_r2)
        gripper_width = joint_positions[:, 6] + joint_positions[:, 7] + joint_positions[:, 8] + joint_positions[:, 9]
        grasped = (avg_finger_dist < 0.03) & (gripper_width < 0.08)  # 두 조건 모두 만족
        rewards = torch.where(
            grasped,
            rewards + grasp_reward_scale,  # +5.0 보너스 (성공적인 그립!)
            rewards
        )

        # ========================================
        # 8️⃣ 단계별 리프트 보너스
        # ========================================
        # 특정 높이를 달성할 때마다 큰 보너스
        rewards = torch.where(lift_amount > 0.02, rewards + 2.0, rewards)  # 2cm → +2.0
        rewards = torch.where(lift_amount > 0.05, rewards + 3.0, rewards)  # 5cm → +3.0 (누적 +5.0)
        rewards = torch.where(lift_amount > 0.10, rewards + 5.0, rewards)  # 10cm → +5.0 (누적 +10.0, 성공!)

        # ========================================
        # 텐서보드 로깅 (학습 모니터링용)
        # ========================================
        # 각 보상 컴포넌트의 평균을 기록하여 학습 과정 분석
        self.extras["log"] = {
            "dist_reward": (dist_reward_scale * dist_reward).mean(),  # 거리 보상 평균
            "rot_reward": (rot_reward_scale * rot_reward).mean(),  # 방향 보상 평균
            "lift_reward": (lift_reward_scale * lift_reward).mean(),  # 리프트 보상 평균
            "action_penalty": (-action_penalty_scale * action_penalty).mean(),  # 액션 페널티 평균
            "left_finger_distance": lfinger_dist.mean(),  # 왼쪽 손가락 거리 평균
            "right_finger_distance": rfinger_dist.mean(),  # 오른쪽 손가락 거리 평균
            "finger_dist_penalty": (finger_reward_scale * finger_dist_penalty).mean(),  # 손가락 보상 평균
            "lift_amount": lift_amount.mean(),  # 평균 리프트 량 (미터)
            "grasp_success_rate": grasped.float().mean(),  # 그립 성공률 (0~1)
        }

        return rewards

    def _find_body_prim_path(self, env_index: int, body_name: str) -> str | None:
        """USD 씬에서 body 이름으로 프림 경로 찾기 (DFS 탐색)

        USD 트리 구조가 복잡할 때 body 이름만으로 정확한 경로를 찾습니다.
        예: "tool0" → "/World/envs/env_0/Robot/.../tool0"

        Args:
            env_index: 환경 인덱스 (0~4095)
            body_name: 찾을 바디 이름 (예: "tool0", "rh_p12_rn_l2")

        Returns:
            프림의 전체 경로 문자열, 없으면 None

        Note:
            이 함수는 초기화 시 한 번만 호출됩니다 (느린 연산).
            결과는 인덱스로 저장되어 매 스텝마다 빠르게 접근합니다.
        """
        stage = get_current_stage()  # USD 스테이지 가져오기
        base = f"/World/envs/env_{env_index}/Robot"  # 검색 시작 경로
        base_prim = stage.GetPrimAtPath(base)

        # 베이스 프림이 유효하지 않으면 None 반환
        if not base_prim or not base_prim.IsValid():
            return None

        # DFS (깊이 우선 탐색)로 트리 전체 탐색
        stack = [base_prim]
        while stack:
            p = stack.pop()
            if not p or not p.IsValid():
                continue

            # 프림 이름이 일치하면 경로 반환
            if p.GetName() == body_name:
                return p.GetPath().pathString

            # 자식 프림들을 스택에 추가 (재귀 탐색)
            for c in p.GetChildren():
                stack.append(c)

        # 찾지 못하면 None 반환
        return None
