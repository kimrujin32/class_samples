# Franka Random Grasp Environment

This environment trains a Franka Emika Panda robot to grasp objects at random positions. Unlike the cabinet environment where the target is fixed, this environment randomizes the object position for each episode, making it suitable for learning generalized grasping policies.

## Features

- **Randomized Object Positions**: Objects spawn at random positions within a defined workspace for each episode
- **External Sensor Data Support**: Interface for receiving position, depth, and pixel coordinate data from external nodes
- **Initial Robot Configuration**: Robot starts at posj(0, 0, 90, 0, 90, 0) position
- **Comprehensive Rewards**: Distance rewards, orientation rewards, grasp detection, and lift rewards

## Environment Details

- **Task**: Grasp a cube object and lift it 10cm above its initial position
- **Episode Length**: 10 seconds (600 timesteps)
- **Object**: Red cube (5cm × 5cm × 5cm, 0.1kg)
- **Workspace**:
  - X: 0.3m to 0.7m (in front of robot)
  - Y: -0.3m to 0.3m (left-right)
  - Z: 0.05m (table height)

## Observation Space (26 dimensions)

1. Robot joint positions (9) - scaled to [-1, 1]
2. Robot joint velocities (9) - scaled
3. Vector from gripper to object (3)
4. Object position (3)
5. External sensor data (2) - depth and normalized pixel coordinates

## Reward Components

1. **Distance Reward** (scale: 2.0): Encourages gripper to move toward object
2. **Rotation Reward** (scale: 1.0): Encourages proper gripper orientation (downward approach)
3. **Finger Distance Penalty** (scale: 2.0): Both fingers should be close to object center (3D distance)
4. **Grasp Reward** (scale: 5.0): Bonus when fingers are close to object AND gripper is closed
5. **Lift Reward** (scale: 10.0): Proportional to how high object is lifted
6. **Action Penalty** (scale: 0.01): L2 regularization on actions
7. **Progressive Lift Bonuses**:
   - +2.0 for lifting > 2cm
   - +3.0 for lifting > 5cm
   - +5.0 for lifting > 10cm (episode success)

## Training

### With Stable Baselines3 (SB3)

#### PPO
```bash
./isaaclab.sh -p scripts/reinforcement_learning/sb3/train.py \
    --task Isaac-Franka-Random-Grasp-Direct-v0 \
    --num_envs 4096 \
    --max_iterations 2000
```

#### SAC
```bash
./isaaclab.sh -p scripts/reinforcement_learning/sb3/train.py \
    --task Isaac-Franka-Random-Grasp-Direct-v0 \
    --num_envs 4096 \
    --max_iterations 150000 \
    --agent sb3_sac_cfg_entry_point
```

### With RSL-RL

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Franka-Random-Grasp-Direct-v0 \
    --num_envs 4096
```

### With SKRL

```bash
./isaaclab.sh -p scripts/reinforcement_learning/skrl/train.py \
    --task Isaac-Franka-Random-Grasp-Direct-v0 \
    --num_envs 4096 \
    --algorithm PPO
```

## Evaluation

### With SB3
```bash
./isaaclab.sh -p scripts/reinforcement_learning/sb3/play.py \
    --task Isaac-Franka-Random-Grasp-Direct-v0 \
    --num_envs 32 \
    --checkpoint /path/to/checkpoint.zip
```

### With RSL-RL
```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Franka-Random-Grasp-Direct-v0 \
    --num_envs 32 \
    --load_run /path/to/run/folder
```

## External Sensor Data Integration

To provide external sensor data (e.g., from camera/depth sensors), you can use the `set_external_sensor_data` method:

```python
import gymnasium as gym
import numpy as np

# Create environment
env = gym.make("Isaac-Franka-Random-Grasp-Direct-v0", num_envs=4096)

# Example: Set depth and pixel coordinates from external source
depth_data = np.random.rand(4096)  # Shape: (num_envs,)
pixel_coords = np.random.rand(4096, 2)  # Shape: (num_envs, 2) - (x, y) coordinates

env.unwrapped.set_external_sensor_data(depth=depth_data, pixel_coords=pixel_coords)

# Continue with normal RL loop
obs, info = env.reset()
for _ in range(1000):
    action = policy(obs)
    obs, reward, terminated, truncated, info = env.step(action)
```

## Key Differences from Franka-Cabinet

1. **Object Position**: Randomized each episode vs. fixed cabinet
2. **Task Objective**: Grasp and lift vs. open drawer
3. **Initial Robot Pose**: posj(0,0,90,0,90,0) vs. custom cabinet-facing pose
4. **Observation Space**: Includes external sensor data placeholders
5. **Termination**: Success when object lifted 10cm vs. drawer opened 39cm

## Configuration

### Modifying Object Spawn Range

Edit `franka_random_grasp_env.py`:

```python
# In FrankaRandomGraspEnvCfg class
object_pos_x_range = (0.3, 0.7)  # Front-back range
object_pos_y_range = (-0.3, 0.3)  # Left-right range
object_pos_z = 0.05  # Table height
```

### Modifying Reward Scales

Edit the configuration class:

```python
dist_reward_scale = 2.0
rot_reward_scale = 1.0
grasp_reward_scale = 5.0
lift_reward_scale = 10.0
action_penalty_scale = 0.01
finger_reward_scale = 2.0
```

### Changing Object Properties

Edit the `object` configuration:

```python
object = RigidObjectCfg(
    prim_path="/World/envs/env_.*/Object",
    spawn=sim_utils.CuboidCfg(
        size=(0.05, 0.05, 0.05),  # Change size
        mass_props=sim_utils.MassPropertiesCfg(mass=0.1),  # Change mass
        # ... other properties
    ),
)
```

## Logs and Metrics

During training, the following metrics are logged:

- `dist_reward`: Distance-based reward
- `rot_reward`: Orientation alignment reward
- `lift_reward`: Lift height reward
- `action_penalty`: Action regularization penalty
- `left_finger_distance`: Distance from left finger to object
- `right_finger_distance`: Distance from right finger to object
- `finger_dist_penalty`: Combined finger distance penalty
- `lift_amount`: Current lift height
- `grasp_success_rate`: Percentage of environments with successful grasp

## Troubleshooting

### Objects falling through ground
- Check physics material friction coefficients
- Verify object collision properties
- Ensure solver iteration counts are sufficient

### Robot not reaching objects
- Verify object spawn range is within robot workspace
- Check initial robot joint configuration
- Adjust reward scales to encourage exploration

### Training instability
- Reduce learning rate
- Increase number of environments
- Adjust reward scales
- Enable observation normalization

## File Structure

```
franka_random_grasp/
├── __init__.py                      # Environment registration
├── franka_random_grasp_env.py      # Main environment implementation
├── README.md                        # This file
└── agents/
    ├── __init__.py
    ├── sb3_ppo_cfg.yaml            # SB3 PPO hyperparameters
    ├── sb3_sac_cfg.yaml            # SB3 SAC hyperparameters
    └── rsl_rl_ppo_cfg.py           # RSL-RL PPO configuration
```
