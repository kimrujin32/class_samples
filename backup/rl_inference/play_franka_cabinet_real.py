#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Franka Cabinet (Doosan e0509) policy inference script for the real robot.

- Loads the RL-Games PPO checkpoint trained on `franka_cabinet_env.py` (obs_dim=25, act_dim=7).
- Builds observations from the real robot state and simple heuristics for the cabinet signals.
- Sends joint targets to the Doosan ROS2 driver using `movej`/`amovej`.

Example:
    python play_franka_cabinet_real.py \\
        --checkpoint last_franka_cabinet_direct_ep_500_rew_1296.2949.pth \\
        --robot-id dsr01 --robot-model e0509 \\
        --handle-pos -0.7 0.01 0.4 --rate 20 --max-steps 500
"""

from __future__ import annotations

import argparse
import importlib
import math
import os
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np
import torch

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from dsr_example.simple.gripper_drl_controller import GripperController

# -----------------------------------------------------------------------------
# Doosan ROS2 Python API 로딩
# -----------------------------------------------------------------------------


def _import_dr_init():
    """Doosan ROS2 Python API(DR_init) 모듈을 동적으로 임포트한다."""
    home_dir = Path.home()
    candidate_roots = [
        home_dir / "ros2_ws/install/dsr_common2/lib/python3.10/site-packages",
        home_dir / "ros2_ws/install/dsr_common2/lib/python3.10/dist-packages",
        home_dir / "ros2_ws/install/dsr_common2/lib",
        home_dir / "ros2_ws/install/lib/python3.10/site-packages",
        home_dir / "ros2_ws/install/lib/python3.10/dist-packages",
    ]
    for path in candidate_roots:
        if path.exists() and path.is_dir():
            path_str = str(path)
            if path_str not in sys.path:
                sys.path.insert(0, path_str)

    candidate_modules = (
        "dsr_common2.imp.DR_init",
        "dsr_common2.DR_init",
        "DR_init",
    )
    last_error: Optional[Exception] = None
    for name in candidate_modules:
        try:
            module = importlib.import_module(name)
            sys.modules.setdefault("dsr_common2.imp.DR_init", module)
            sys.modules["DR_init"] = module
            return module
        except ModuleNotFoundError as exc:  # pragma: no cover - 환경 의존
            last_error = exc
            continue
    raise RuntimeError(
        "Doosan ROS2 Python API(DR_init)를 찾을 수 없습니다. "
        "`source ~/ros2_ws/install/setup.bash` 실행 여부를 확인하세요."
    ) from last_error


DR_init = _import_dr_init()


# -----------------------------------------------------------------------------
# 정책 정의 (RL-Games 스타일)
# -----------------------------------------------------------------------------


class RunningMeanStd(torch.nn.Module):
    """RL-Games의 obs/value 정규화 버퍼."""

    def __init__(self, shape: int):
        super().__init__()
        self.register_buffer("running_mean", torch.zeros(shape, dtype=torch.float64))
        self.register_buffer("running_var", torch.ones(shape, dtype=torch.float64))
        self.register_buffer("count", torch.tensor(1e-4, dtype=torch.float64))

    def normalize(self, x: torch.Tensor) -> torch.Tensor:
        return (x - self.running_mean.to(dtype=x.dtype)) * torch.rsqrt(
            self.running_var.to(dtype=x.dtype).clamp(min=1e-6)
        )


class A2CNetwork(torch.nn.Module):
    """단일 헤드 actor-critic (mu/logstd/critic value)."""

    def __init__(self, obs_dim: int, act_dim: int, hidden_dims: list[int]):
        super().__init__()
        layers = []
        last_dim = obs_dim
        for dim in hidden_dims:
            layers.append(torch.nn.Linear(last_dim, dim))
            layers.append(torch.nn.ELU())
            last_dim = dim
        self.actor_mlp = torch.nn.Sequential(*layers)
        self.mu = torch.nn.Linear(last_dim, act_dim)
        self.value = torch.nn.Linear(last_dim, 1)
        self.sigma = torch.nn.Parameter(torch.zeros(act_dim))

    def forward(self, obs: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        x = self.actor_mlp(obs)
        mu = self.mu(x)
        sigma = torch.exp(self.sigma)
        value = self.value(x)
        return mu, sigma, value


class CabinetPolicy(torch.nn.Module):
    """rl_games PPO 정책을 재현."""

    def __init__(self, obs_dim: int, act_dim: int, hidden_dims: list[int]):
        super().__init__()
        self.running_mean_std = RunningMeanStd(obs_dim)
        self.value_mean_std = RunningMeanStd(1)
        self.a2c_network = A2CNetwork(obs_dim, act_dim, hidden_dims)

    @torch.no_grad()
    def act_inference(self, obs: torch.Tensor) -> torch.Tensor:
        norm_obs = self.running_mean_std.normalize(obs)
        mu, _, _ = self.a2c_network(norm_obs)
        # 학습 시 env.clip_actions=1.0 이므로 클램프하여 사용
        return torch.clamp(mu, -1.0, 1.0)


# -----------------------------------------------------------------------------
# 로봇/환경 파라미터 (FrankaCabinetEnv 기준)
# -----------------------------------------------------------------------------

SOFT_LIMIT_DEG_MIN = np.array([-180.0, -120.0, -180.0, -180.0, -180.0, -180.0], dtype=np.float32)
SOFT_LIMIT_DEG_MAX = np.array([+180.0, +120.0, +180.0, +180.0, +180.0, +180.0], dtype=np.float32)

GRIPPER_LIMIT_DEG_MIN = np.array([-60.0, -60.0, -60.0, -60.0], dtype=np.float32)
GRIPPER_LIMIT_DEG_MAX = np.array([+60.0, +60.0, +60.0, +60.0], dtype=np.float32)

LOWER_RAD_ARM = np.deg2rad(SOFT_LIMIT_DEG_MIN)
UPPER_RAD_ARM = np.deg2rad(SOFT_LIMIT_DEG_MAX)
LOWER_RAD_GRIPPER = np.deg2rad(GRIPPER_LIMIT_DEG_MIN)
UPPER_RAD_GRIPPER = np.deg2rad(GRIPPER_LIMIT_DEG_MAX)

OBS_LOWER_RAD = np.concatenate([LOWER_RAD_ARM, LOWER_RAD_GRIPPER], axis=0)
OBS_UPPER_RAD = np.concatenate([UPPER_RAD_ARM, UPPER_RAD_GRIPPER], axis=0)

ACT_LOWER_RAD = np.concatenate([LOWER_RAD_ARM, [LOWER_RAD_GRIPPER[2]]], axis=0)  # gripper control = rh_r1
ACT_UPPER_RAD = np.concatenate([UPPER_RAD_ARM, [UPPER_RAD_GRIPPER[2]]], axis=0)

DOF_VELOCITY_SCALE = 0.1
ACTION_SCALE = 7.5  # matches FrankaCabinetEnvCfg.action_scale
ACTION_DT_SIM = 1.0 / 60.0  # decimation*dt = 1/60 sec
ACTION_SMOOTHING_BETA = 0.6
BASE_COMMAND_RATE_HZ = 20.0
MIN_COMMAND_DELTA_DEG = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2], dtype=np.float32)

OBS_DIM = 25
ACT_DIM = 7

HOME_POSJ_DEG = np.array([0.0, 0.0, 90.0, 0.0, 90.0, 0.0], dtype=np.float32)

ROB_TO_ENV = np.arange(6, dtype=np.int64)
ENV_TO_ROB = np.arange(6, dtype=np.int64)

DEFAULT_GRIPPER_STROKE_MIN = 0.0
DEFAULT_GRIPPER_STROKE_MAX = 700.0
DEFAULT_GRIPPER_DEADBAND = 5.0


def deg2rad(arr_deg: np.ndarray) -> np.ndarray:
    return arr_deg * math.pi / 180.0


def rad2deg(arr_rad: np.ndarray) -> np.ndarray:
    return arr_rad * 180.0 / math.pi


def clamp_deg(q_deg: np.ndarray, qmin_deg: np.ndarray, qmax_deg: np.ndarray) -> np.ndarray:
    return np.minimum(np.maximum(q_deg, qmin_deg), qmax_deg)


# -----------------------------------------------------------------------------
# 추론 노드
# -----------------------------------------------------------------------------


class DoosanCabinetInference(Node):
    """RL-Games PPO 체크포인트를 이용해 e0509 팔+그리퍼 policy를 실행."""

    def __init__(self, args: argparse.Namespace):
        robot_id = args.robot_id.strip("/") or "dsr01"
        robot_namespace = f"/{robot_id}"
        super().__init__("doosan_cabinet_inference", namespace=robot_namespace)

        self._robot_id = robot_id
        self._robot_model = args.robot_model.strip("/") if args.robot_model else ""
        self.args = args

        self._load_dsr_bindings()
        try:
            self._set_robot_mode(self._robot_mode_autonomous)
            print("[INFO] 로봇 모드: AUTONOMOUS")
        except Exception as exc:  # pragma: no cover - 하드웨어 의존
            print(f"[WARNING] 로봇 모드 설정 실패: {exc}")

        self.device = torch.device("cuda:0" if torch.cuda.is_available() and not args.cpu else "cpu")
        self.policy = self._load_policy(args.checkpoint, self.device)

        self.rate_hz = args.rate
        self.dt_loop = 1.0 / self.rate_hz
        self.move_vel = args.vel
        self.move_acc = args.acc

        self.drawer_handle_pos = np.array(args.handle_pos, dtype=np.float32)
        self.tcp_offset = np.array(args.tcp_offset, dtype=np.float32)
        self.drawer_pos_est = float(args.drawer_pos)
        self.drawer_vel_est = 0.0

        self.initial_joint_deg: Optional[np.ndarray] = None
        self.gripper_joint_rad = np.zeros(4, dtype=np.float32)
        self.gripper_joint_rad[2] = float(args.initial_gripper)
        self.prev_gripper_rad = self.gripper_joint_rad.copy()
        self.step_counter = 0
        self.max_episode_steps = max(1, int(args.max_steps))
        self.command_interval = max(1, int(round(self.rate_hz / BASE_COMMAND_RATE_HZ)))

        self.prev_joint_rad_env: Optional[np.ndarray] = None
        self.prev_joint_sample_time: Optional[float] = None
        self.prev_target_rad_env: Optional[np.ndarray] = None
        self.last_command_deg_env: Optional[np.ndarray] = None
        self.tcp_pos_estimate = self.drawer_handle_pos - np.array([0.3, 0.0, 0.0], dtype=np.float32)

        # Actuated joint target (6 arm + rh_r1)
        self.actuated_target_rad = np.zeros(ACT_DIM, dtype=np.float32)
        self.prev_filtered_target_rad: Optional[np.ndarray] = None


        print("[INFO] Gripper, Enter를 누르세요...")
        input()

        self.gripper_enabled = True #not args.disable_gripper
        self.gripper_min_stroke = float(args.gripper_min_stroke)
        self.gripper_max_stroke = float(args.gripper_max_stroke)
        self.gripper_deadband = float(args.gripper_deadband)
        self.last_gripper_stroke: Optional[float] = None
        self.gripper: Optional[GripperController] = None

        if self.gripper_enabled:
            try:
                self.gripper = GripperController(node=self, namespace=robot_id)
                self.gripper.initialize()
                init_stroke = self._gripper_rad_to_stroke(self.gripper_joint_rad[2])
                self.gripper.move(int(init_stroke))
                self.last_gripper_stroke = init_stroke
                print(f"[INFO] Gripper initialized (stroke ~ {init_stroke:.1f})")
            except Exception as exc:  # pragma: no cover - 하드웨어 의존
                print(f"[WARNING] 그리퍼 초기화 실패: {exc}")
                self.gripper_enabled = False

        print(f"[INFO] 제어 주기: {self.rate_hz:.1f} Hz (dt={self.dt_loop:.4f}s)")
        print(f"[INFO] 추론 디바이스: {self.device}")
        print(f"[INFO] ROS namespace: {self.get_namespace()}")
        print(f"[INFO] 최대 스텝: {self.max_episode_steps}")
        print(f"[INFO] 핸들 위치(base): {self.drawer_handle_pos}")
        print(f"[INFO] TCP 옵셋(base): {self.tcp_offset}")

    # ------------------------------------------------------------------
    # 초기화/로딩
    # ------------------------------------------------------------------

    def _load_dsr_bindings(self):
        setattr(DR_init, "__dsr__node", self)
        setattr(DR_init, "__dsr__id", self._robot_id)
        setattr(DR_init, "__dsr__model", self._robot_model)
        try:
            if "DSR_ROBOT2" in sys.modules:
                dsr_mod = importlib.reload(sys.modules["DSR_ROBOT2"])
            else:
                dsr_mod = importlib.import_module("DSR_ROBOT2")
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "DSR_ROBOT2 모듈을 찾을 수 없습니다. `colcon build` 및 setup.bash 실행 여부를 확인하세요."
            ) from exc

        self._dsr_mod = dsr_mod
        self._get_current_posj = dsr_mod.get_current_posj
        self._get_current_posx = getattr(dsr_mod, "get_current_posx", None)
        self._movej = dsr_mod.movej
        self._amovej = getattr(dsr_mod, "amovej", None)
        self._wait = getattr(dsr_mod, "wait", None)
        self._set_robot_mode = dsr_mod.set_robot_mode
        self._robot_mode_autonomous = dsr_mod.ROBOT_MODE_AUTONOMOUS

    def _load_policy(self, checkpoint_path: str, device: torch.device) -> CabinetPolicy:
        if not os.path.exists(checkpoint_path):
            raise FileNotFoundError(f"체크포인트를 찾을 수 없습니다: {checkpoint_path}")

        checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
        state_dict = checkpoint.get("model", checkpoint)

        policy = CabinetPolicy(obs_dim=OBS_DIM, act_dim=ACT_DIM, hidden_dims=[256, 128, 64])
        incompatible = policy.load_state_dict(state_dict, strict=False)
        if incompatible.missing_keys:
            print(f"[WARNING] 누락된 키: {incompatible.missing_keys}")
        if incompatible.unexpected_keys:
            print(f"[WARNING] 예기치 않은 키: {incompatible.unexpected_keys}")
        policy.to(device)
        policy.eval()
        print("[INFO] 정책 네트워크 로드 완료 (RL-Games style PPO)")
        return policy

    def _gripper_rad_to_stroke(self, rad: float) -> float:
        """joint rad -> 그리퍼 stroke(0~700) 선형 맵핑."""
        rad_clamped = float(np.clip(rad, LOWER_RAD_GRIPPER[2], UPPER_RAD_GRIPPER[2]))
        stroke = np.interp(
            rad_clamped,
            [LOWER_RAD_GRIPPER[2], UPPER_RAD_GRIPPER[2]],
            [self.gripper_min_stroke, self.gripper_max_stroke],
        )
        return float(np.clip(stroke, self.gripper_min_stroke, self.gripper_max_stroke))

    # ------------------------------------------------------------------
    # 관측/행동
    # ------------------------------------------------------------------

    def _compute_observation(self) -> np.ndarray:
        q_deg_robot = np.array(self._get_current_posj(), dtype=np.float32)
        try:
            q_deg_env = q_deg_robot[ROB_TO_ENV]
        except IndexError:
            q_deg_env = q_deg_robot

        if self.initial_joint_deg is None:
            self.initial_joint_deg = q_deg_env.copy()
            self.actuated_target_rad[:6] = deg2rad(q_deg_env)

        q_rad_env = deg2rad(q_deg_env)
        full_q_rad = np.concatenate([q_rad_env, self.gripper_joint_rad], axis=0)

        dof_pos_scaled = 2.0 * (full_q_rad - OBS_LOWER_RAD) / (OBS_UPPER_RAD - OBS_LOWER_RAD) - 1.0
        dof_pos_scaled = np.clip(dof_pos_scaled, -1.0, 1.0)

        now = time.time()
        if self.prev_joint_rad_env is None or self.prev_joint_sample_time is None:
            dof_vel_scaled_arm = np.zeros(6, dtype=np.float32)
        else:
            dt = max(now - self.prev_joint_sample_time, 1e-3)
            q_diff = q_rad_env - self.prev_joint_rad_env
            vel_rad = q_diff / dt
            dof_vel_scaled_arm = np.clip(vel_rad * DOF_VELOCITY_SCALE, -5.0, 5.0)
        self.prev_joint_rad_env = q_rad_env.copy()
        self.prev_joint_sample_time = now

        # 그리퍼 속도는 마지막 추정값을 사용
        dt_grip = self.dt_loop
        grip_vel = (self.gripper_joint_rad - self.prev_gripper_rad) / max(dt_grip, 1e-3)
        self.prev_gripper_rad = self.gripper_joint_rad.copy()
        dof_vel_scaled_grip = np.clip(grip_vel * DOF_VELOCITY_SCALE, -5.0, 5.0)

        # TCP position (mm -> m)
        if self._get_current_posx is not None:
            try:
                posx = self._get_current_posx()
                tcp_pos = np.array(posx[0:3], dtype=np.float32) / 1000.0
                self.tcp_pos_estimate = tcp_pos
            except Exception:
                tcp_pos = self.tcp_pos_estimate
        else:
            tcp_pos = self.tcp_pos_estimate

        tcp_grasp_pos = tcp_pos + self.tcp_offset
        to_target = self.drawer_handle_pos - tcp_grasp_pos

        obs = np.concatenate(
            (
                dof_pos_scaled,  # 10
                np.concatenate([dof_vel_scaled_arm, dof_vel_scaled_grip], axis=0),  # 10
                to_target,  # 3
                np.array([self.drawer_pos_est], dtype=np.float32),  # 1
                np.array([self.drawer_vel_est], dtype=np.float32),  # 1
            ),
            axis=0,
        )
        return obs.astype(np.float32)

    def _apply_action(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.float32).flatten()
        if action.shape[0] != ACT_DIM:
            raise ValueError(f"액션 차원이 {ACT_DIM}이어야 합니다. 입력: {action.shape}")
        if not np.all(np.isfinite(action)):
            print("[WARNING] 비유효 액션이 감지되어 명령을 건너뜁니다.")
            return

        delta_rad = action * ACTION_SCALE * ACTION_DT_SIM
        target_rad_env = self.actuated_target_rad + delta_rad
        target_rad_env = np.clip(target_rad_env, ACT_LOWER_RAD, ACT_UPPER_RAD)

        if self.prev_filtered_target_rad is None:
            filtered_rad_env = target_rad_env
        else:
            beta = np.clip(ACTION_SMOOTHING_BETA, 0.0, 1.0)
            filtered_rad_env = beta * self.prev_filtered_target_rad + (1.0 - beta) * target_rad_env
        self.prev_filtered_target_rad = filtered_rad_env.copy()
        self.actuated_target_rad = filtered_rad_env.copy()

        # gripper 추정 상태 업데이트 (rh_r1)
        self.gripper_joint_rad[2] = filtered_rad_env[6]

        if self.step_counter % self.command_interval != 0:
            return

        target_deg_env = rad2deg(filtered_rad_env[:6])
        target_deg_env = clamp_deg(target_deg_env, SOFT_LIMIT_DEG_MIN, SOFT_LIMIT_DEG_MAX)

        if self.last_command_deg_env is not None:
            if np.all(np.abs(target_deg_env - self.last_command_deg_env) < MIN_COMMAND_DELTA_DEG):
                return

        try:
            target_deg_robot = target_deg_env[ENV_TO_ROB]
        except IndexError:
            target_deg_robot = target_deg_env

        try:
            if self._amovej is not None:
                self._amovej(target_deg_robot.tolist(), vel=self.move_vel, acc=self.move_acc)
            else:
                self._movej(target_deg_robot.tolist(), vel=self.move_vel, acc=self.move_acc)
            self.last_command_deg_env = target_deg_env.copy()
        except Exception as exc:  # pragma: no cover - 하드웨어 의존
            print(f"[ERROR] movej 실패: {exc}")

        if self.gripper_enabled and self.gripper is not None:
            stroke = self._gripper_rad_to_stroke(self.gripper_joint_rad[2])
            if self.last_gripper_stroke is None or abs(stroke - self.last_gripper_stroke) > self.gripper_deadband:
                try:
                    self.gripper.move(int(stroke))
                    self.last_gripper_stroke = stroke
                except Exception as exc:  # pragma: no cover - 하드웨어 의존
                    print(f"[WARNING] 그리퍼 명령 실패: {exc}")

    # ------------------------------------------------------------------
    # 메인 루프
    # ------------------------------------------------------------------

    def spin(self, max_steps: Optional[int] = None):
        print("\n[INFO] === Cabinet 추론 시작 (실 로봇) ===")
        print("[INFO] 로봇을 안전 자세로 준비 후 Enter를 누르세요...")
        input()

        torch.set_grad_enabled(False)
        self.prev_joint_rad_env = None
        self.prev_joint_sample_time = None
        self.prev_target_rad_env = None
        self.last_command_deg_env = None
        self.step_counter = 0

        try:
            while rclpy.ok():
                loop_start = time.time()

                obs_np = self._compute_observation()
                obs_tensor = torch.from_numpy(obs_np).unsqueeze(0).to(self.device)

                with torch.inference_mode():
                    action_tensor = self.policy.act_inference(obs_tensor)
                action = action_tensor.squeeze(0).detach().cpu().numpy()

                self._apply_action(action)

                self.step_counter += 1

                if self.step_counter % 50 == 0:
                    print(f"[INFO] Step {self.step_counter}/{self.max_episode_steps}")

                if max_steps is not None and self.step_counter >= max_steps:
                    print(f"[INFO] 최대 스텝({max_steps}) 도달. 홈 포즈로 복귀합니다.")
                    break

                elapsed = time.time() - loop_start
                sleep_time = self.dt_loop - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif self.step_counter % 50 == 0:
                    print(f"[WARNING] 제어 주기를 초과했습니다: {elapsed:.3f}s > {self.dt_loop:.3f}s")

        except KeyboardInterrupt:
            print("\n[INFO] 사용자 중단")
        except Exception as exc:  # pragma: no cover - 디버깅용
            print(f"\n[ERROR] 추론 중 예외 발생: {exc}")
            import traceback

            traceback.print_exc()
        finally:
            try:
                print("[INFO] 로봇을 홈 포즈로 복귀시키는 중...")
                self._movej(HOME_POSJ_DEG.tolist(), vel=self.move_vel, acc=self.move_acc)
                if self._wait is not None:
                    self._wait()
                if self.gripper_enabled and self.gripper is not None:
                    safe_stroke = self._gripper_rad_to_stroke(0.0)
                    self.gripper.move(int(safe_stroke))
                    self.gripper.terminate()
            except Exception as exc:  # pragma: no cover - 하드웨어 의존
                print(f"[WARNING] 홈 포즈로 복귀 실패: {exc}")
            self.prev_joint_rad_env = None
            self.prev_joint_sample_time = None
            self.prev_target_rad_env = None
            self.last_command_deg_env = None
            print("[INFO] 추론 종료")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Franka Cabinet (Doosan e0509) 정책 추론")
    parser.add_argument("--checkpoint",type=str,required=True,help="RL-Games PPO 체크포인트 경로 (.pth)",)
    parser.add_argument("--robot-id", type=str, default="dsr01", help="ROS2 로봇 네임스페이스")
    parser.add_argument("--robot-model", type=str, default="e0509", help="로봇 모델명")
    parser.add_argument("--rate", type=float, default=20.0, help="제어 주기 (Hz)")
    parser.add_argument("--vel", type=float, default=30.0, help="movej 속도 (deg/s)")
    parser.add_argument("--acc", type=float, default=30.0, help="movej 가속도 (deg/s^2)")
    parser.add_argument("--max-steps", type=int, default=500, help="최대 반복 스텝 (기본 500)")
    parser.add_argument("--handle-pos", nargs=3, type=float, default=[-0.7, 0.01, 0.4], help="캐비닛 핸들 위치 (base, m)")
    parser.add_argument("--tcp-offset",nargs=3,type=float, default=[0.0, 0.04, 0.0], help="TCP->그리퍼 중심 오프셋 (base, m)",)
    parser.add_argument("--drawer-pos", type=float, default=0.0, help="서랍 열림 추정값 (m, 센서 없으면 0)")
    parser.add_argument("--initial-gripper", type=float, default=0.0, help="rh_r1 초기 각도(rad) 추정치")
    parser.add_argument("--gripper-min-stroke", type=float, default=DEFAULT_GRIPPER_STROKE_MIN, help="그리퍼 최소 stroke")
    parser.add_argument("--gripper-max-stroke", type=float, default=DEFAULT_GRIPPER_STROKE_MAX, help="그리퍼 최대 stroke")
    parser.add_argument("--gripper-deadband", type=float, default=DEFAULT_GRIPPER_DEADBAND, help="stroke 변화 허용 오차")
    parser.add_argument("--disable-gripper", action="store_true", help="그리퍼 명령 비활성화")
    parser.add_argument("--cpu", action="store_true", help="강제로 CPU에서 추론 실행")
    return parser.parse_args()


def main():
    args = _parse_args()

    rclpy.init()
    os.environ.setdefault("ROS_DOMAIN_ID", "0")

    node = DoosanCabinetInference(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        node.spin(max_steps=args.max_steps)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
