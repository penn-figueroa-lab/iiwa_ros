# CLAUDE.md — iiwa_ros

This file gives Claude Code the context needed to understand, edit, and extend the `iiwa_ros` metapackage without breaking the kinematics conventions or hardware safety properties that downstream packages depend on.

Fork of the original [epfl-lasa/iiwa_ros](https://github.com/epfl-lasa/iiwa_ros) with additions for the Figueroa Robotics Lab @ Penn.

---

## Package overview

`iiwa_ros` is a ROS (Noetic / catkin) metapackage. It is the **hardware and kinematics foundation** for the workspace: the driver connects to the physical robot over FRI, the Gazebo plugin provides a physics-accurate simulation, and `iiwa_tools` provides FK/IK/Jacobian used by `iiwa_interactive_controller` at 500 Hz.

---

## Sub-packages

| Sub-package | Role |
|---|---|
| `iiwa_ros` | Metapackage (declares dependencies on all others) |
| `iiwa_driver` | FRI hardware interface — bridges KUKA FRI to `ros_control` |
| `iiwa_description` | URDF/xacro models for iiwa7 and iiwa14; lab-specific gripper and sensor attachments |
| `iiwa_tools` | RBDyn-based FK, IK, Jacobian, gravity library; also exposed as ROS services |
| `iiwa_gazebo` | Gazebo simulation with gravity compensation (`GravityCompensationHWSim`) |
| `iiwa_control` | `ros_control` controller configs and `CustomEffortController` plugin |
| `iiwa_moveit` | MoveIt! SRDF, kinematics, and planning configuration |

---

## iiwa_driver

FRI-based `hardware_interface::RobotHW` implementation. Connects to the robot via UDP and runs the `ros_control` loop at **500 Hz**.

### Connection parameters (`config/iiwa.yaml`)

```yaml
fri:
  port: 30200
  robot_ip: 192.170.10.2
  robot_description: /robot_description

hardware_interface:
  control_freq: 500   # Hz
  joints: [iiwa_joint_1 ... iiwa_joint_7]
```

### Published topics

| Topic | Type | Notes |
|---|---|---|
| `/iiwa/joint_states` | `JointState` | Position, velocity, effort for all 7 joints (via joint_state_controller) |
| `/iiwa/commanding_status` | `Bool` | True when FRI is in commanding state |
| `/iiwa/additional_outputs` | `AdditionalOutputs` | External torques and other FRI monitoring data |

### Subscribed topics

| Topic | Type | Notes |
|---|---|---|
| `/iiwa/TorqueController/command` | `Float64MultiArray` [7] | Joint torque commands (from `iiwa_interactive_controller`) |

### Launch

```bash
# Real robot only
roslaunch iiwa_driver iiwa_bringup.launch [controller:=TorqueController] [model:=14]
```

**Real-robot startup sequence:**
1. Confirm Linux laptop has IP `192.170.10.1`, mask `255.255.255.0` on the KONI port.
2. On Smartpad: activate `AUT` mode → select app in `[Application]` → press `Play ▶`.
3. Within 10 s: run `roslaunch iiwa_driver iiwa_bringup.launch`.
4. Verify `/iiwa/joint_states` is publishing and reflects the real robot state.

---

## iiwa_tools

RBDyn + mc_rbdyn_urdf based kinematics. Two usage modes:

### 1. C++ library (direct include — used by `iiwa_interactive_controller`)

```cpp
#include <iiwa_tools/iiwa_tools.h>

iiwa_tools::IiwaTools tools;
tools.init_rbdyn(urdf_string, end_effector);      // call once at init

iiwa_tools::RobotState state;
state.position = ...;  state.velocity = ...;

auto ee = tools.perform_fk(state);                // → EefState {translation, orientation}
auto [J, J_dot] = tools.jacobians(state);         // → {6×7, 6×7} matrices
auto g = tools.gravity({0,0,-9.81}, state);       // → 7-vector of gravity torques
Eigen::VectorXd q = tools.perform_ik(target, seed); // → 7-vector
```

Each call copies the internal `_rbdyn_urdf` struct for thread-safety.

### 2. ROS services

```bash
roslaunch iiwa_tools iiwa_service.launch
```

| Service | Type | Request | Response |
|---|---|---|---|
| `/iiwa/iiwa_fk_server` | `GetFK` | `Float64MultiArray joints` | `Pose[] poses` |
| `/iiwa/iiwa_ik_server` | `GetIK` | `Pose[] poses`, optional seed | `Float64MultiArray joints`, `bool[] is_valid` |
| `/iiwa/iiwa_jacobian_server` | `GetJacobian` | `float64[] joint_angles`, `float64[] joint_velocities` | `Float64MultiArray jacobian` |
| `/iiwa/iiwa_jacobians_server` | `GetJacobians` | same | jacobian + jacobian_deriv |
| `/iiwa/iiwa_gravity_server` | `GetGravity` | joint state + gravity vector | `float64[] compensation_torques` |

### **Critical: Jacobian convention**

`IiwaTools::jacobian()` returns a **6×7** matrix where:
- Rows 0–2: **angular** velocity components
- Rows 3–5: **linear** velocity components

This is the RBDyn spatial Jacobian convention (`[ang, lin]`). **All consumers must account for this.** `iiwa_interactive_controller` publishes this Jacobian as-is on `/iiwa/jacobian`, and `rga_qpik_node` swaps rows to `[lin, ang]` before solving.

---

## iiwa_gazebo

Gazebo simulation using the `GravityCompensationHWSim` plugin, which extends `DefaultRobotHWSim` to add gravity compensation identical to what the real robot's internal controller provides.

**The plugin calls `/iiwa/iiwa_gravity_server` on every simulation step.** Therefore `iiwa_tools iiwa_service.launch` must be running before Gazebo starts.

```bash
# Standalone Gazebo (launches iiwa_service automatically via iiwa_gazebo.launch)
roslaunch iiwa_gazebo iiwa_gazebo.launch [gui:=true] [controller:=TorqueController] [model:=14]
```

`iiwa_interactive_controller`'s `modular_passive_gazebo.launch` launches its own Gazebo with the same plugin — verify the gravity service is active if simulation behaves unexpectedly.

---

## iiwa_control

Provides `ros_control` controller configurations and the `CustomEffortController` plugin.

### Controller instances (`config/iiwa_control.yaml`)

| Controller name | Type | Command topic |
|---|---|---|
| `TorqueController` | `effort_controllers/JointGroupEffortController` | `/iiwa/TorqueController/command` (Float64MultiArray [7]) |
| `PositionTorqueController` | `effort_controllers/JointGroupPositionController` | — |
| `PositionController` | `position_controllers/JointGroupPositionController` | — |
| `CustomControllers` | `iiwa_control/CustomEffortController` | `~command` (Float64MultiArray) |
| `PositionTrajectoryController` | `position_controllers/JointTrajectoryController` | — |

**`iiwa_interactive_controller` uses `TorqueController` exclusively.** It publishes `Float64MultiArray[7]` torques directly to `/iiwa/TorqueController/command` — it does not go through `CustomEffortController`.

### CustomEffortController

A `ros_control` plugin that bridges `robot_controllers`-based control logic to the hardware interface. Supports joint-space and task-space modes (`params/space: joint|task`). In task-space mode it uses `IiwaTools` for FK and Jacobian. Accepts commands on `~command` (size depends on configured I/O types). Not used by `iiwa_interactive_controller` — provided for alternative controller configurations.

---

## iiwa_description

URDF/xacro model library.

| File | Contents |
|---|---|
| `urdf/iiwa14.urdf.xacro` | Main iiwa 14 model; includes optional gripper/finger macros |
| `urdf/iiwa7.urdf.xacro` | iiwa 7 model |
| `urdf/iiwa14.xacro` | Core iiwa14 link/joint definitions |
| `urdf/iiwa7.xacro` | Core iiwa7 link/joint definitions |
| `urdf/finger_with_xela.xacro` | **Lab addition**: Xela-instrumented finger assembly (taxel TF frames) |
| `urdf/robotiq_2f85_umi_gripper.xacro` | **Lab addition**: Robotiq 2F-85 / UMI gripper attachment |
| `urdf/iiwa.gazebo.xacro` | Gazebo-specific plugins and friction settings |
| `urdf/iiwa.transmission.xacro` | `ros_control` transmission interfaces |
| `meshes/iiwa14/` | Visual and collision STL meshes for iiwa 14 |
| `meshes/iiwa7/` | Visual and collision STL meshes for iiwa 7 |
| `meshes/finger/` | Xela finger meshes |

**The URDF loaded at runtime determines which TF frames exist.** If `finger_with_xela.xacro` is included, taxel frames `left_finger_taxel_frame_{1..24}` and `right_finger_taxel_frame_{1..24}` are published by `robot_state_publisher` — required by `rga_assist_node` for Xela wrench compensation.

---

## iiwa_moveit

MoveIt! configuration for path planning with iiwa 14. Includes SRDF (`config/iiwa14.srdf`), OMPL planner settings, and kinematics solver configuration. Not used at runtime by `iiwa_interactive_controller` or `assistive_robot_grasp`.

---

## Key invariants — do not break

1. **Jacobian row order is `[ang(3), lin(3)]`.** This is an RBDyn convention baked into `IiwaTools::jacobian()`. Every consumer that uses this Jacobian must swap rows explicitly. Changing `IiwaTools` to output `[lin, ang]` would silently break `iiwa_interactive_controller`.

2. **`init_rbdyn()` must be called before any FK/Jacobian/IK call.** The `_rbdyn_urdf`, `_rbd_indices`, and `_ef_index` fields are only set during `init_rbdyn()`. Calling `perform_fk()` before `init_rbdyn()` will crash.

3. **`IiwaTools` makes a full copy of `_rbdyn_urdf` on each FK/Jacobian call** (for thread-safety). This is intentional — do not remove the copy or replace it with a reference without adding a mutex.

4. **Gazebo gravity compensation requires `iiwa_gravity_server` to be running.** If the service is not available, `GravityCompensationHWSim` will stall on every simulation step. Always start Gazebo via a launch file that includes the gravity service.

5. **Joint names are `iiwa_joint_1` through `iiwa_joint_7` (not `joint_1`).** All controller configs, URDF, and driver use this naming. Changing a name in one place requires changing it everywhere.

6. **FRI connects within a 10-second timeout window.** After pressing Play on the Smartpad, `iiwa_bringup.launch` must be launched within 10 s. If the timeout expires, uncheck the app in `[Application]` on the Smartpad and retry.

---

## Build notes

- C++14 standard.
- Key external dependencies (system-installed): SpaceVecAlg, RBDyn, mc_rbdyn_urdf, corrade, robot_controllers.
- FRI library (private KUKA code) is required only for `iiwa_driver`. Gazebo and tools build without it.
- Build with `catkin build` — `catkin_make` may not correctly handle the sub-package structure.

---

## Common failure modes

| Symptom | Likely cause | Fix |
|---|---|---|
| Gazebo freezes at startup | `iiwa_gravity_server` not running | Start `iiwa_service.launch` before Gazebo |
| `perform_fk` / `jacobians` crash (segfault) | `init_rbdyn()` not called | Call `init_rbdyn(urdf_string, end_effector)` first |
| `/iiwa/joint_states` not published | `joint_state_controller` not loaded | Check `iiwa_control.launch` started correctly |
| FRI timeout at startup | Too slow launching driver | Retry: uncheck + recheck app on Smartpad, then `roslaunch` within 10 s |
| Wrong EE position in FK | Wrong `end_effector` string in `init_rbdyn()` | Must match a body name in the URDF (e.g., `"iiwa_link_ee"`) |
| Taxel TF frames missing | `finger_with_xela.xacro` not included in URDF, or `robot_state_publisher` not running | Check launch file includes finger xacro and `robot_state_publisher` |

---

## Relationship with `iiwa_interactive_controller`

- `iiwa_interactive_controller` includes `<iiwa_tools/iiwa_tools.h>` and calls `IiwaTools::init_rbdyn()`, `perform_fk()`, `jacobians()` directly — no ROS service calls at runtime.
- The torque loop: `iiwa_interactive_controller` → `/iiwa/TorqueController/command` → `TorqueController` (ros_control) → `iiwa_driver` (real) or `GravityCompensationHWSim` (Gazebo) → robot joints.
- The Jacobian convention (`[ang, lin]`) originates in `IiwaTools::jacobians()` and propagates to `/iiwa/jacobian` — see `iiwa_interactive_controller/CLAUDE.md` invariant #5 for how `rga_qpik_node` handles this.
- `robot_state_publisher` (started by `iiwa_driver/launch/iiwa_setup.launch`) publishes TF for all URDF frames — required for the taxel frames used by `rga_assist_node`.