# Go2 onboard policy deployment

This ROS 2 workspace runs exported Go2 locomotion policies in Gazebo Classic
and on a real Go2. Both runners use the `rl_sar` policy runtime; Gazebo reads
simulated state and sends joint commands, while the real runner uses Unitree
LowState and LowCmd. Isaac Lab is needed for training and export, not inference.

Policies live in `policy/go2/<policy_name>/` with a `config.yaml` and model file.
`policy/go2/base.yaml` selects the default policy. Set
`RL_SAR_POLICY_CONFIG_NAME` when launching a runner to choose another folder.
The current default is `benchmarkv8.4mlp`.

## Install on the Go2 onboard computer

Use the standalone CMake build on the robot. The onboard target used here is
Ubuntu 20.04, ARM64 (`aarch64`), Jetson L4T R35.3.1, and system Python 3.8.
ROS Foxy may already be installed, but this hardware build does not require
ROS, Gazebo, `go2_description`, or LibTorch for ONNX policies. Do not source
`/opt/ros/humble/setup.bash` on this machine; Humble is not installed here.

### 1. Install build dependencies and clone

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git curl file unzip \
  python3-dev python3-numpy libtbb-dev libyaml-cpp-dev gcc-10 g++-10

git clone https://github.com/DINHQuangDung1999/go2_onboard_deployment.git
cd ~/go2_onboard_deployment
```

If already cloned, use the existing checkout. Build in its final location:
CMake embeds the checkout's policy and runtime paths. Rebuild after moving it;
do not copy `cmake_build/` from another checkout or architecture.

### 2. Set up ONNX Runtime and build

```bash
cd ~/go2_onboard_deployment
./build.sh --cmake
```

This installs/checks ONNX Runtime and builds into `cmake_build/bin/`, using
Release mode, `/usr/bin/python3`, and two compile jobs. It does not download
robot descriptions or install PyTorch. Override the job count with
`BUILD_JOBS=4 ./build.sh --cmake`. For a TorchScript policy, explicitly request
`INFERENCE_RUNTIME=libtorch` (or `all`); that requires a compatible LibTorch
installation and is separate from the default ONNX setup.

On Jetson, the runtime installer builds ONNX Runtime 1.20.1 with
`onnxruntime_ENABLE_CPUINFO=OFF` to avoid the CPU-topology initialization failure
encountered on this target. A fresh runtime build downloads source and build
tools and can take substantial time and disk space. `JETSON_BUILD_JOBS=2`
controls that build separately.

To reuse a runtime from a previous installation **on the same robot**, copy its
complete `onnxruntime/` directory before running the build. For this robot's
archived installation:

```bash
cd ~/go2_onboard_deployment
# Only do this when the destination does not already exist.
test ! -e library/inference_runtime/onnxruntime && \
  cp -a ~/Documents/backup_071026/go2_isaac_gazebo/library/inference_runtime/onnxruntime \
    library/inference_runtime/onnxruntime
./build.sh --cmake
```

The runtime must contain `include/`, `lib/`, and the
`JETSON_CPUINFO_DISABLED` marker from the compatible build. Do not create the
marker manually. Runtime libraries and build products are ignored by Git and
are not included in a fresh clone.

### 3. Validate the policy offline

These checks load the policy without connecting to the robot or sending motor
commands:

```bash
./cmake_build/bin/validate_policy_config go2 benchmarkv8.4mlp
./cmake_build/bin/validate_policy_model \
  policy/go2/benchmarkv8.4mlp/policy.onnx obs_history 45 30 12
```

The dimensions above belong to `benchmarkv8.4mlp`: 45 observations, 30 history
frames, and 12 actions. Use the selected policy's configuration for other
models. Both checks must pass before running that policy on hardware; they do
not establish physical stability or validate the robot's calibration.

### 4. Run on the real Go2

Identify the interface connected to the robot with `ip -br addr`, then replace
`<robot-interface>` below with its name:

```bash
cd ~/go2_onboard_deployment
RL_SAR_POLICY_CONFIG_NAME=benchmarkv8.4mlp \
  ./cmake_build/bin/rl_real_go2 <robot-interface>
```

Support the Go2 for the initial test and keep the hand controller available for
emergency stop. Check the policy's observation order, joint mapping, action
scale, gains, and limits before enabling it. Press `0` to get up, wait for the
transition, then press `1` to enter policy mode. Check stability with the hand
controller centered before trying small joystick commands. Press `9` to get
down.

After C++ changes, rerun `./build.sh --cmake`. Model and YAML changes only need
a runner restart, but rerun the offline validation for each changed policy.

## Build for Gazebo on a development computer

The simulation workflow uses Ubuntu 22.04, ROS 2 Humble, Gazebo Classic, and
the project's ROS dependencies. It is separate from the robot's standalone
build above.

```bash
cd ~/go2_onboard_deployment
source /opt/ros/humble/setup.bash
./build.sh robot_msgs robot_joint_controller go2_description rl_sar
```

The wrapper downloads the robot descriptions and creates `package.xml` links
from `package.ros2.xml` before calling colcon. A raw colcon command on a fresh
clone skips those steps, causing the missing-manifest error and unknown
`go2_description` warning. The `rosidl_interface_packages` group is already
present in the message package's manifest.

The launch scripts source Humble and this workspace's `install/` directory.
Rebuild after C++ or package changes. Policy model and YAML changes only need
a runner restart.

## Run in Gazebo

Start Gazebo in Terminal 1:

```bash
cd ~/go2_onboard_deployment
./scripts/run_gazebo_go2.sh
```

After the Go2 model spawns, start the policy runner in Terminal 2:

```bash
cd ~/go2_onboard_deployment
RL_SAR_POLICY_CONFIG_NAME=benchmarkv8.4mlp ./scripts/run_rl_sim_go2.sh
```

In the runner terminal, press `0` to get up, wait for the transition to finish,
then press `1` to enter policy mode. Use small `W/S/A/D/Q/E` commands to test
walking; `Space` zeros the movement command. Press `9` to get down.


Terminal 3, apply joint fault

```bash
cd ~/go2_onboard_deployment
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub --once /rl_sar/gain_alpha std_msgs/msg/Float32MultiArray \
  "{data: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}"
```

## More documentation

- [Deployment details](docs/deployment_reference.md): configuration, controls,
  logging, troubleshooting, and policy export.
- [History policy deployment](docs/history_policy_deployment.md): model input
  signatures and validation.
- [Legacy policy run guide](docs/legacy_policy_run_guide.md): earlier policy and
  fault test procedures; verify names and paths before using them.
- [No-linear-velocity policy notes](docs/no_linear_velocity_policy_progress.md):
  historical training and deployment notes.
