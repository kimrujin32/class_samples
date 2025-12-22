# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
"""
e0509 Random Grasp environment.
"""

import gymnasium as gym

from . import agents

##
# Register Gym environments.
##

gym.register(
    id="Isaac-E0509-Random-Grasp-Direct-v0",
    entry_point=f"{__name__}.e0509_random_grasp_env:RandomGraspOnTableEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.e0509_random_grasp_env:RandomGraspOnTableEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:FrankaRandomGraspPPORunnerCfg",
        "sb3_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
        "sb3_sac_cfg_entry_point": f"{agents.__name__}:sb3_sac_cfg.yaml",
    },
)
