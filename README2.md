# Go2 IsaacLab No-Lin-Vel Policy Progress

This file tracks the work done to prepare a new IsaacLab policy that is safer to deploy toward the real Unitree Go2.

## Project Goal

The long-term goal is to train a walking/balancing Go2 policy in IsaacLab, run it through the ROS2/Gazebo workflow first, and then deploy the same policy path to the real robot.

The key lesson from the previous `isaaclab48` real-robot test is that a policy that works in Gazebo is not automatically real-deployable. The actor observation contract must match what the real robot can actually provide.

## Problem With Current `isaaclab48` Policy

The `isaaclab48` policy uses 48 observations:

```text
lin_vel      3
ang_vel      3
gravity_vec  3
commands     3
dof_pos     12
dof_vel     12
actions     12
total       48
```

The problem is `lin_vel`.

In Gazebo/IsaacLab, base linear velocity is available cleanly from simulation. On the real Go2 deployment path, the low-level robot state gives IMU and motor states, but it does not directly provide true base linear velocity. During the real test, the deployed policy effectively received zero or invalid `lin_vel`, while the policy had been trained expecting real base velocity feedback.

This is a major sim-to-real observation mismatch.

## Chosen Real Solution

We chose the clean engineering path:

```text
Train a new IsaacLab flat Go2 policy without base linear velocity in the actor observation.
```

The new policy should use only signals that both IsaacLab/Gazebo and the real Go2 deployment path can provide:

```text
ang_vel      3
gravity_vec  3
commands     3
dof_pos     12
dof_vel     12
actions     12
total       45
```

This is why the new policy is called `isaaclab45_real` in this repo.

## IsaacLab Files Added Or Modified

The IsaacLab source tree is here:

```text
/home/rinderudon/Projects/ws/IsaacLab
```

### Added

```text
/home/rinderudon/Projects/ws/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/go2/flat_nolinvel_env_cfg.py
```

This file creates a new Go2 flat environment config derived from the normal Go2 flat config, but removes:

```python
self.observations.policy.base_lin_vel = None
self.observations.policy.height_scan = None
```

`base_lin_vel` is removed because the real Go2 deployment path does not provide direct base linear velocity.

`height_scan` is also disabled for this flat-policy path so the actor observation stays compact and deployable.

The command range is inherited from the normal Go2 velocity task. This keeps full command-following walking:

```text
forward/backward
sideways
yaw turning
```

### Modified

```text
/home/rinderudon/Projects/ws/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/go2/agents/rsl_rl_ppo_cfg.py
```

Added a PPO runner config:

```python
class UnitreeGo2FlatNoLinVelPPORunnerCfg(UnitreeGo2FlatPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()
        self.max_iterations = 500
        self.experiment_name = "unitree_go2_flat_nolinvel"
```

This gives the new training run its own experiment folder:

```text
logs/rsl_rl/unitree_go2_flat_nolinvel
```

### Modified

```text
/home/rinderudon/Projects/ws/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/go2/__init__.py
```

Registered two new task IDs:

```text
Isaac-Velocity-Flat-Unitree-Go2-NoLinVel-v0
Isaac-Velocity-Flat-Unitree-Go2-NoLinVel-Play-v0
```

Use the first one for training and the second one for play/testing.

## Deployment Config Added In This Repo

Added:

```text
/home/rinderudon/Projects/go2_isaac_gazebo/policy/go2/isaaclab45_real/config.yaml
```

This config expects:

```yaml
num_observations: 45
observations: ["ang_vel", "gravity_vec", "commands", "dof_pos", "dof_vel", "actions"]
model_name: "policy.pt"
```

The folder does not have `policy.pt` yet. That file should be produced by training/exporting the new IsaacLab policy.

## Checks Already Done

Static checks passed:

```text
Python syntax check passed for the new IsaacLab Python files.
YAML parse check passed for policy/go2/isaaclab45_real/config.yaml.
The deployment observation dimension is 45.
Whitespace/diff checks passed for the touched files.
```

One runtime registry check was attempted with plain Conda Python:

```bash
conda run -n env_isaaclab python -c "import gymnasium as gym; import isaaclab_tasks"
```

That failed because plain Python could not import Isaac Sim's `pxr` module:

```text
ModuleNotFoundError: No module named 'pxr'
```

This does not mean the new task is broken. It means IsaacLab should be launched through `./isaaclab.sh -p`, because that script sets the Isaac Sim Python environment.

## Training Command

This is the command to train the new policy:

```bash
cd ~/Projects/ws/IsaacLab
conda activate env_isaaclab

./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Velocity-Flat-Unitree-Go2-NoLinVel-v0 \
  --num_envs 4096 \
  --headless
```

If GPU memory is too tight, reduce `--num_envs`:

```bash
--num_envs 2048
```

or:

```bash
--num_envs 1024
```

## Play/Test Command In IsaacLab

After training, test the policy in IsaacLab first:

```bash
cd ~/Projects/ws/IsaacLab
conda activate env_isaaclab

./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
  --task Isaac-Velocity-Flat-Unitree-Go2-NoLinVel-Play-v0 \
  --num_envs 50 \
  --checkpoint logs/rsl_rl/unitree_go2_flat_nolinvel/<run_name>/model_<iteration>.pt
```

Replace `<run_name>` and `<iteration>` with the actual trained run folder and checkpoint number.

## Exported Policy Location

After a successful play/export step, the TorchScript policy should appear around:

```text
/home/rinderudon/Projects/ws/IsaacLab/logs/rsl_rl/unitree_go2_flat_nolinvel/<run_name>/exported/policy.pt
```

Copy that file into:

```text
/home/rinderudon/Projects/go2_isaac_gazebo/policy/go2/isaaclab45_real/policy.pt
```

Then the deployment folder should contain:

```text
policy/go2/isaaclab45_real/config.yaml
policy/go2/isaaclab45_real/policy.pt
```

## Gazebo Test After Export

Start Gazebo:

```bash
cd ~/Projects/go2_isaac_gazebo
./scripts/run_gazebo_go2.sh
```

Start the RL controller with the new policy config:

```bash
cd ~/Projects/go2_isaac_gazebo
RL_SAR_POLICY_CONFIG_NAME=isaaclab45_real ./scripts/run_rl_sim_go2.sh
```

Gazebo must be stable before trying the real robot.

## Real Robot Test Rule

Do not test the new policy directly on the floor first.

Use this order:

```text
1. IsaacLab play test
2. Gazebo ROS2 test
3. Real robot on a stand
4. Real robot with tiny joystick commands
5. Real walking only after debug logs look sane
```

For real testing, the important logs are:

```text
policy observations
policy actions
target joint positions
lowcmd q/dq/kp/kd/tau
lowstate motor q/dq/tau_est
IMU angular velocity
gravity/projected gravity
commanded velocity
```

## Current Status

Prepared:

```text
New IsaacLab no-lin-vel env config
New IsaacLab no-lin-vel PPO runner config
New IsaacLab train/play task IDs
New deployment config: policy/go2/isaaclab45_real/config.yaml
```

Still needed:

```text
Train the policy
Play/test it in IsaacLab
Export policy.pt
Copy policy.pt into policy/go2/isaaclab45_real/
Test in Gazebo
Only then test carefully on the real robot
```
