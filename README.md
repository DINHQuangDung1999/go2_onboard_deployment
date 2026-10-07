# Go2 onboard policy deployment

This ROS 2 workspace runs exported Go2 locomotion policies in Gazebo Classic
and on a real Go2. Both runners use the `rl_sar` policy runtime; Gazebo reads
simulated state and sends joint commands, while the real runner uses Unitree
LowState and LowCmd. Isaac Lab is needed for training and export, not inference.

Policies live in `policy/go2/<policy_name>/` with a `config.yaml` and model file.
`policy/go2/base.yaml` selects the default policy. Set
`RL_SAR_POLICY_CONFIG_NAME` when launching a runner to choose another folder.
The current default is `benchmarkv8.4mlp`.

## Build

On Ubuntu 22.04 with ROS 2 Humble, Gazebo Classic, and the project dependencies
installed:

```bash
cd /home/dung-admin/go2_ws/go2_onboard_deployment
source /opt/ros/humble/setup.bash
colcon build --merge-install --symlink-install \
  --packages-select robot_msgs robot_joint_controller go2_description rl_sar \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

The launch scripts source ROS and this workspace's `install/` directory.
Rebuild after C++ or package changes. Policy model and YAML changes only need
a runner restart.

## Run in Gazebo

Start Gazebo in Terminal 1:

```bash
cd /home/dung-admin/go2_ws/go2_onboard_deployment
./scripts/run_gazebo_go2.sh
```

After the Go2 model spawns, start the policy runner in Terminal 2:

```bash
cd /home/dung-admin/go2_ws/go2_onboard_deployment
RL_SAR_POLICY_CONFIG_NAME=benchmarkv8.4mlp ./scripts/run_rl_sim_go2.sh
```

In the runner terminal, press `0` to get up, wait for the transition to finish,
then press `1` to enter policy mode. Use small `W/S/A/D/Q/E` commands to test
walking; `Space` zeros the movement command. Press `9` to get down.


Terminal 3, apply joint fault

```bash
cd /home/dung-admin/go2_ws/go2_onboard_deployment
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub --once /rl_sar/gain_alpha std_msgs/msg/Float32MultiArray \
  "{data: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}"
```

## Run on the real Go2

Copy the exported policy folder to the onboard computer and build the runtime
there for its architecture. Use the network interface connected to the robot
in place of `<robot-interface>`. With the Go2 supported for an initial test,
start the real runner onboard:

```bash
cd /path/to/go2_onboard_deployment
RL_SAR_POLICY_CONFIG_NAME=benchmarkv8.4mlp \
  ./install/lib/rl_sar/rl_real_go2 <robot-interface>
```

Press `0` to get up and wait for the transition, then press `1` to enter policy
mode. Check stability with the hand controller centered before trying small
joystick commands. Keep the hand controller available for emergency stop.
Gazebo validation does not establish hardware safety: check the policy's
observation order, joint mapping, action scale, gains, and limits first.

## More documentation

- [Deployment details](docs/deployment_reference.md): configuration, controls,
  logging, troubleshooting, and policy export.
- [History policy deployment](docs/history_policy_deployment.md): model input
  signatures and validation.
- [Legacy policy run guide](docs/legacy_policy_run_guide.md): earlier policy and
  fault test procedures; verify names and paths before using them.
- [No-linear-velocity policy notes](docs/no_linear_velocity_policy_progress.md):
  historical training and deployment notes.
