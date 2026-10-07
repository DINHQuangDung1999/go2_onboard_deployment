# Go2 Policy Run Guide

Historical policy and fault-test procedures. Policy names, onboard paths, and
build commands in this guide may differ from the current checkout; use the
[workspace README](../README.md) for current startup commands.

This guide shows how to run the four retained policies in Gazebo and on the
real Go2. It covers deployment only. IsaacLab training is maintained in the
separate IsaacLab workspace.

## Policies

| Policy name | Model contract | Intended use | Hardware status |
| --- | --- | --- | --- |
| `isaaclab45_real` | 45 observations, single frame | Older flat-terrain real-deploy policy | Legacy; harness testing only |
| `policy_history` | 45 observations plus 30-frame history | Dual-input history policy | Experimental; harness testing only |
| `fault_history_b5` | 45 observations x 30 frames | Scripted tucked-leg B5 experiment | Guarded fault test only |
| `adaptive_fault_d22` | 45 observations x 30 frames | Residual actuator-loss adaptation | Experimental adaptive policy |

All four ONNX models pass model and YAML contract validation on the Go2
Jetson. This does not mean that every policy has completed physical validation.

Policy selection always uses:

```bash
RL_SAR_POLICY_CONFIG_NAME=<policy_name>
```

`policy/go2/base.yaml` currently selects `adaptive_fault_d22` when the variable is not
provided.

## Gazebo

### Compare benchmarkv8.3 and benchmarkv8.4 with an RL hip fault

Run each policy in a fresh Gazebo session. Start Gazebo with
`./scripts/run_gazebo_go2.sh`, then start the policy in another terminal:

```bash
RL_SAR_POLICY_CONFIG_NAME=benchmarkv8.3 ./scripts/run_rl_sim_go2.sh
```

Press `0` to stand, wait for get-up, then press `1` to enter policy mode.
Apply the same command sequence in each trial. In a third terminal, inject a
gradual RL hip actuator loss (20% torque remains after a 2 second ramp):

```bash
./scripts/run_gazebo_joint_fault.sh RL_hip_joint 0.20 10 2
```

The 10 second delay starts when this command is run. Use
`./scripts/run_gazebo_joint_fault.sh clear` to restore full torque. Stop and
restart Gazebo for the next trial, selecting `benchmarkv8.4` in the policy
command. Run the fault command again with the same arguments. For a complete
torque loss, use `0.00` instead of `0.20`.

To record a trial, start this before injecting the fault, using a different
output directory for each policy:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag record -o debug_logs/benchmarkv8.3_rlhip \
  /gazebo/model_states /robot_joint_controller/state \
  /robot_joint_controller/command /robot_joint_controller/fault_alpha
```

The fault scales actuator torque in Gazebo's joint controller. The policies
stay selected throughout the trial; `fault_policy_switch_enabled` is false in
`policy/go2/base.yaml`.

### Start Gazebo

Terminal 1:

```bash
cd ~/Projects/go2_isaac_gazebo
./scripts/run_gazebo_go2.sh
```

Wait until the Go2 model and controllers finish spawning.

### Start A Policy

Open Terminal 2 and run exactly one command from this list.

Flat real-deploy policy:

```bash
cd ~/Projects/go2_isaac_gazebo
RL_SAR_POLICY_CONFIG_NAME=isaaclab45_real ./scripts/run_rl_sim_go2.sh
```

Dual-input history policy:

```bash
cd ~/Projects/go2_isaac_gazebo
RL_SAR_POLICY_CONFIG_NAME=policy_history ./scripts/run_rl_sim_go2.sh
```

B5 tucked-leg policy:

```bash
cd ~/Projects/go2_isaac_gazebo
RL_SAR_POLICY_CONFIG_NAME=fault_history_b5 ./scripts/run_rl_sim_go2.sh
```

D2.2 adaptive-fault policy:

```bash
cd ~/Projects/go2_isaac_gazebo
RL_SAR_POLICY_CONFIG_NAME=adaptive_fault_d22 ./scripts/run_rl_sim_go2.sh
```

### Gazebo Controls

Use the keyboard in the policy terminal:

```text
0      get up
1      enter policy locomotion
9      get down
P      enter passive immediately
W/S    increase/decrease forward command
A/D    increase/decrease lateral command
Q/E    increase/decrease yaw command
Space  reset x, y, and yaw commands to zero
```

Normal sequence:

```text
press 0
wait until get-up reaches 100%
press 1
begin with small movement commands
press 9 before ending the simulation
```

### Test D2.2 Actuator Loss

Start Gazebo and `adaptive_fault_d22`, enter policy mode, then use Terminal 3:

```bash
cd ~/Projects/go2_isaac_gazebo
./scripts/run_gazebo_joint_fault.sh FL_calf_joint 0.20 10 2
```

Arguments:

```text
JOINT_NAME ALPHA DELAY_SECONDS RAMP_SECONDS
```

`alpha` is the remaining actuator effectiveness:

```text
1.00  healthy actuator
0.20  20% authority remains
0.05  5% authority remains
0.00  complete simulated torque loss
```

List valid joints:

```bash
./scripts/run_gazebo_joint_fault.sh list
```

Clear a latched fault:

```bash
./scripts/run_gazebo_joint_fault.sh clear
```

The 12 accepted joints are:

```text
FR_hip_joint    FR_thigh_joint    FR_calf_joint
FL_hip_joint    FL_thigh_joint    FL_calf_joint
RR_hip_joint    RR_thigh_joint    RR_calf_joint
RL_hip_joint    RL_thigh_joint    RL_calf_joint
```

### Test The B5 Tucked-Leg Scenario

B5 was trained around a deterministic tucked front-right leg. Start
`fault_history_b5`, enter policy mode, then run:

```bash
cd ~/Projects/go2_isaac_gazebo
source /opt/ros/humble/setup.bash
source install/setup.bash

/usr/bin/python3 src/rl_sar/scripts/run_fault_scenario.py \
  FR_calf_joint \
  --mode tucked-leg-lock \
  --delay 10 \
  --hold 50 \
  --restore-on-exit
```

This is a scripted tuck-and-lock experiment, not the general adaptive behavior
targeted by D2.2.

## Real Go2

The recommended architecture is:

```text
laptop: SSH, monitoring, and emergency supervision
Jetson onboard Go2: policy inference and 200 Hz LowCmd loop
hand controller: movement commands
```

The onboard workspace is:

```text
/home/unitree/go2_isaac_gazebo
```

### Connect To The Jetson

From the laptop:

```bash
ssh unitree@192.168.0.192
```

The Go2 and laptop must be connected to the same Wi-Fi network. The Jetson
must also have its internal `eth0` robot connection active.

### Read-Only Preflight

Run this before every physical policy session:

```bash
cd ~/go2_isaac_gazebo
./cmake_build/bin/go2_state_probe eth0 10
```

Move the hand-controller sticks during the ten-second probe. Continue only if
the result says:

```text
PASS: LowState and embedded hand-controller data are live.
```

Do not continue if LowState, IMU, joint state, or controller values are stale.

### Start A Real Policy

Keep the Go2 on its overhead harness. Run exactly one command.

Flat real-deploy policy:

```bash
cd ~/go2_isaac_gazebo
RL_SAR_POLICY_CONFIG_NAME=isaaclab45_real \
  ./cmake_build/bin/rl_real_go2 eth0
```

Dual-input history policy:

```bash
cd ~/go2_isaac_gazebo
RL_SAR_POLICY_CONFIG_NAME=policy_history \
  ./cmake_build/bin/rl_real_go2 eth0
```

B5 policy without activating its fault test:

```bash
cd ~/go2_isaac_gazebo
unset RL_SAR_ENABLE_REAL_TUCK_TEST
RL_SAR_POLICY_CONFIG_NAME=fault_history_b5 \
  ./cmake_build/bin/rl_real_go2 eth0
```

D2.2 adaptive-fault policy:

```bash
cd ~/go2_isaac_gazebo
RL_SAR_POLICY_CONFIG_NAME=adaptive_fault_d22 \
  ./cmake_build/bin/rl_real_go2 eth0
```

Running D2.2 does not inject a hardware fault. It only loads the policy that
was trained to infer and respond to actuator degradation from observation
history.

### Guarded Real Benchmark v8.3 RL Hip Gain-Fault Test

For `benchmarkv8.3`, the guarded test is restricted to `RL_hip_joint` by
`policy/go2/benchmarkv8.3/config.yaml`. The onboard Jetson must have this
policy folder and a current `rl_real_go2` build with the guarded gain-fault
code. Verify the policy on the Jetson before starting the robot:

```bash
cd ~/go2_isaac_gazebo
./cmake_build/bin/validate_policy_config go2 benchmarkv8.3
./cmake_build/bin/go2_state_probe eth0 10
```

With the Go2 supported by an overhead harness, an operator at the physical
controller, and the sticks centered, launch on the Jetson:

```bash
cd ~/go2_isaac_gazebo
unset RL_SAR_ENABLE_REAL_TUCK_TEST
RL_SAR_ENABLE_REAL_GAIN_FAULT_TEST=1 \
RL_SAR_POLICY_CONFIG_NAME=benchmarkv8.3 \
  ./cmake_build/bin/rl_real_go2 eth0
```

Press `0` to get up, wait for completion, then `1` for policy mode. While
standing stably with sticks centered, press `F` in the same terminal; the
program must announce `ARMED RL_hip_joint`. Press `T` within five seconds to
trigger. The program ramps RL hip `kp` and `kd` to 20% over three seconds,
holds for at most 15 seconds, then restores full gains and requests GetDown.
Press `P` to abort the test and request GetDown early. Use `Ctrl+C` for normal
prone-first shutdown. This scales control gains, rather than imposing the
Gazebo torque multiplier or a mechanical joint lock.

The enable gate, policy allowlist, centered-command check, fresh LowState,
tilt limits, joint-state limits, timeout, and GetDown request are implemented
in `src/rl_sar/src/rl_real_go2.cpp`. If arming or triggering is rejected, read
the reason printed by the program; do not bypass the gate for a hardware run.

### Guarded Real Adaptive Gain-Fault Test

This harness-only test randomly chooses one joint from the FL, RR, or RL leg.
It excludes the FR leg, ramps that joint's PD gains to 50% over three seconds,
holds the derating for at most 15 seconds, and then restores full gains and
requests GetDown. Safety violations do the same immediately.

On the Jetson SSH terminal:

```bash
cd ~/go2_isaac_gazebo
unset RL_SAR_ENABLE_REAL_TUCK_TEST
RL_SAR_ENABLE_REAL_GAIN_FAULT_TEST=1 \
RL_SAR_POLICY_CONFIG_NAME=adaptive_fault_d2 \
  ./cmake_build/bin/rl_real_go2 eth0
```

After normal get-up and policy entry, keep the physical controller centered
and type these keys in the laptop's SSH terminal:

```text
F  randomly select and announce one non-FR joint; arm for five seconds
T  begin the three-second gain-derating ramp
P  restore full gains and abort immediately
```

After the ramp completes, use only very small hand-controller commands. Use
`Ctrl+C` for the normal prone-first shutdown. This test models residual
actuator authority; it does not mechanically lock the selected joint.

### Real-Robot Control Sequence

```text
1. Leave the hand-controller sticks centered.
2. Press 0 on the SSH terminal, or A on the controller, to get up.
3. Wait until get-up reaches 100%.
4. Press 1 on the SSH terminal, or RB+DPadUp, to enter policy mode.
5. Confirm stable standing before applying a small joystick command.
6. Use Ctrl+C to end the deployment through the controlled prone shutdown.
```

Do not use `P` as the normal shutdown command. It requests passive mode
immediately and does not perform the controlled GetDown trajectory.

### Prone-First Ctrl+C Shutdown

`Ctrl+C` now performs this sequence:

```text
stop policy inference and movement commands
run the two-second GetDown position trajectory
reach the recorded prone pose
enter Passive
gradually reduce stiffness, torque, and damping
place motors in stop/standby mode
exit
```

Expected output includes:

```text
Safe shutdown: moving to prone before releasing motor stiffness.
Getting down ... 100%
Controlled prone posture reached.
RL_Real safe shutdown complete
```

If LowState becomes stale or GetDown cannot finish within five seconds, the
runtime uses its gradual motor-release fallback. Keep the harness supporting
the robot until the process has fully exited.

### Guarded Real B5 Fault Test

This test is not part of normal deployment. Use the overhead harness and keep
an emergency stop available.

```bash
cd ~/go2_isaac_gazebo
RL_SAR_ENABLE_REAL_TUCK_TEST=1 \
RL_SAR_POLICY_CONFIG_NAME=fault_history_b5 \
  ./cmake_build/bin/rl_real_go2 eth0
```

After normal get-up and policy entry:

```text
F  arm the test for five seconds
T  trigger the tucked front-right leg
P  abort immediately to passive mode
```

Use `Ctrl+C`, not `P`, for the normal controlled prone shutdown.

Real actuator derating for D2.2 is intentionally not exposed as a casual
command. Physical fault injection requires a separately reviewed safety gate,
limits, watchdogs, and harness protocol.

## Validate A Policy

On the Jetson:

```bash
cd ~/go2_isaac_gazebo
./cmake_build/bin/validate_policy_config go2 isaaclab45_real
./cmake_build/bin/validate_policy_config go2 policy_history
./cmake_build/bin/validate_policy_config go2 fault_history_b5
./cmake_build/bin/validate_policy_config go2 adaptive_fault_d22
```

Each command must end with:

```text
Policy configuration validation passed
```

Validation confirms model shape, input count, observation construction, and
12-action output. It does not replace Gazebo testing or harness validation.

## Troubleshooting

`Failed to load model`:

- run `validate_policy_config` for the selected policy;
- confirm `model_name` in that policy's `config.yaml` exists;
- confirm the Jetson build reports `USE_ONNX: ON`.

No joystick commands:

- run `go2_state_probe` and move the controller sticks;
- confirm `joystick_changes` increases and the axes reach nonzero values;
- confirm the runtime uses internal interface `eth0`.

Gazebo package or ROS environment errors:

```bash
cd ~/Projects/go2_isaac_gazebo
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Fault remains active in Gazebo:

```bash
./scripts/run_gazebo_joint_fault.sh clear
```

Never move forcefully against a real joint that remains stiff after shutdown.
Keep the robot supported, stop the process, and investigate LowCmd/gain state
before another physical run.
