# FTNet Contact-History Deployment Debug Notes

This note explains what was broken before, what we changed, and why the robot behavior improved.

## Goal

We want to deploy the FTNet policy in Gazebo.

This policy is different from `isaaclab48` because it uses:

- 49 current observations
- 30 frames of observation history
- foot contact booleans instead of body linear velocity

So the policy needs two important things during deployment:

1. Correct current observation.
2. Correct 30-frame observation history.

If either one is wrong, the policy can look unstable even if the exported `policy.pt` is valid.

## Bug 1: Foot Contact Topics Existed, But Had No Contact Data

### Before

Gazebo showed these topics:

```text
/FL_foot_contact
/FR_foot_contact
/RL_foot_contact
/RR_foot_contact
```

But checking one foot showed:

```bash
ros2 topic echo --once /FL_foot_contact
```

Result:

```text
states: []
```

That means the topic existed, but Gazebo was not detecting any collision contact for that sensor.

### Why It Happened

The robot foot links are fixed to the calf links. Gazebo Classic merges fixed links into the parent link.

So the real spawned collision name was not a simple foot collision name. It looked like:

```text
FL_calf_fixed_joint_lump__FL_foot_collision_collision_3
```

But the contact sensor was listening to the wrong collision name, so it always returned empty contact states.

### After

We patched the contact sensors in:

```text
src/rl_sar_zoo/go2_description/xacro/gazebo.xacro
```

The important idea is:

```xml
<gazebo reference="FL_calf">
  <sensor name="FL_foot_contact" type="contact">
    <contact>
      <collision>FL_calf_fixed_joint_lump__FL_foot_collision_collision_3</collision>
    </contact>
  </sensor>
</gazebo>
```

Now Gazebo publishes real contact force data.

## Bug 2: FTNet Needed Foot Contact Booleans

### Before

The FTNet policy expects foot contact booleans in this order:

```text
[FL, FR, RL, RR]
```

But `/rl_sar/foot_contacts` had no real publisher, or the values were not reliable.

That means the policy might see:

```text
[0, 0, 0, 0]
```

even when the robot was standing on the ground.

For a contact-history policy, this is very bad. The policy thinks the robot is flying or falling.

### After

We created:

```text
src/rl_sar/scripts/run_foot_contact_bridge.py
```

It reads Gazebo contact topics:

```text
/FL_foot_contact
/FR_foot_contact
/RL_foot_contact
/RR_foot_contact
```

Then it publishes:

```text
/rl_sar/foot_contacts
```

as:

```text
[FL_contact, FR_contact, RL_contact, RR_contact]
```

Each value is:

```text
1.0 = foot force is above threshold
0.0 = foot force is below threshold
```

Example debug output:

```text
force_norm_N [FL FR RL RR] = [755.08 754.41 838.67 832.71] contacts = [1 1 1 1]
```

This means all four feet are touching the ground.

## Bug 3: The 30-Frame History Started Empty

### Before

FTNet does not only use the current observation. It also uses the previous 30 observations.

At the beginning of deployment, those 30 frames did not really exist yet.

So the policy could receive something like:

```text
[zero, zero, zero, ..., current_observation]
```

That is not what it saw during training. This can cause violent behavior right after pressing `1`.

### After

We added warm-start history logic in:

```text
src/rl_sar/src/rl_sim.cpp
```

The idea:

```cpp
if (warm_start_history && episode_length_buf <= 1)
{
    history_obs_buf.reset({0}, clamped_obs);
}
```

Meaning:

```text
At the first policy step, fill the whole 30-frame history with the current observation.
```

So instead of empty history, the policy starts with:

```text
[current, current, current, ..., current]
```

This is much safer.

## Bug 4: Zero Command Behavior

The config contains:

```yaml
hold_zero_command: true
```

When this is true, the controller holds the default standing pose when the command is zero.

This helps because FTNet seems weaker at quiet standing in Gazebo than `isaaclab48`.

So the robot does not immediately ask FTNet to solve standing still. It only uses the policy when you give a movement command like `W`.

## Recommended Test Flow

Use this order:

```bash
cd ~/Projects/go2_isaac_gazebo
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Terminal 1:

```bash
./scripts/run_gazebo_go2.sh
```

Let the robot stand up first.

Terminal 2:

```bash
ros2 run rl_sar run_foot_contact_bridge.py --threshold 1.0 --debug
```

Wait until you see contact values like:

```text
contacts = [1 1 1 1]
```

Terminal 3:

```bash
./scripts/run_rl_sim_go2_ftnet_contact_history.sh
```

Then press `1`, and test movement gently.

## What Is Fixed Now

Fixed:

- Foot contact sensor collision names.
- Foot contact bridge from Gazebo force data to FTNet booleans.
- Correct foot order: `[FL, FR, RL, RR]`.
- History warm start for the first 30-frame input.
- Safer zero-command behavior with `hold_zero_command: true`.

Still not fully solved:

- FTNet walking still does not look as clean as `isaaclab48`.
- This likely means there is still a sim-to-sim mismatch.
- Possible remaining mismatch areas:
  - observation scaling
  - joint order
  - action order
  - command scaling
  - foot contact threshold/timing
  - policy was trained with slightly different dynamics than Gazebo

## Simple Summary

Before, FTNet was running without reliable foot contact information and with bad startup history.

After, FTNet receives real foot contact booleans and starts with a safer 30-frame history.

That is why the robot improved, even though the gait is still not as clean as `isaaclab48` yet.
