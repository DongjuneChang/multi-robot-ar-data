# Network Robots Configuration

Single Source of Truth for all robot configurations used by Unity, ROS2, and HTML panels.

## Unit Conventions

| Parameter | Unit | Notes |
|-----------|------|-------|
| Joint angles | **DEGREES** | Human-readable, ROS2 converts to radians at runtime |
| Positions | METERS | All position values |
| Orientations | Quaternion [x, y, z, w] | ROS2 convention |

## File Structure

Each robot has a single YAML file containing all configuration:

```
network_robots/
├── README.md           # This file
├── lite6.yaml          # UFACTORY Lite6 (6 DOF)
├── xarm5.yaml          # UFACTORY xArm5 (5 DOF)
├── fr3.yaml            # Franka Research 3 (7 DOF)
├── ur3e.yaml           # Universal Robots UR3e (6 DOF)
├── ur5e.yaml           # Universal Robots UR5e (6 DOF)
├── ur16e.yaml          # Universal Robots UR16e (6 DOF)
├── iiwa7.yaml          # KUKA iiwa7 (7 DOF)
├── med7.yaml           # KUKA MED7 (7 DOF)
├── med14.yaml          # KUKA MED14 (7 DOF)
└── kinova_gen3_7dof.yaml  # Kinova Gen3 (7 DOF)
```

## Configuration Sections

### 1. robot
Basic robot information.
```yaml
robot:
  model: "lite6"
  manufacturer: "UFACTORY"
  dof: 6
  id_pattern: "{model}_{anchor}"  # lite6_Q1
```

### 2. joints
Joint configuration with angles in **DEGREES**.
```yaml
joints:
  names: ["joint1", "joint2", ...]
  zero: [0, 0, 0, 0, 0, 0]        # Zero configuration (deg)
  home: [0, 35, 60, 0, 25, -20]   # Home position (deg)
  limits:
    joint1: [-360, 360]
    ...
```

### 3. frame_hierarchy
Unity ICoordinateFrame / NetworkTF configuration.
```yaml
frame_hierarchy:
  qr_frame_pattern: "QR_{anchor}"
  root_frame:
    type: "NetworkTF"
    id: "{robot_id}"
    parent: "QR_{anchor}"
  network_frames:
    "{robot_id}_base": ...
    "{robot_id}_eef": ...
    "{robot_id}_work": ...
  local_frames: [...]
```

### 4. ros2
ROS2 communication settings.
```yaml
ros2:
  namespace: ""
  topics:
    tf: "/tf"
    tf_static: "/tf_static"
    joint_states: "/joint_states"
  prefab_links: {...}
  external_frames: {...}
  subscribe_frames: [...]
```

### 5. named_frames
ROS2 FrameManager configuration. References `joints.zero` and `joints.home`.
```yaml
named_frames:
  version: "2.0"
  coordinate_system: "ROS2"
  frames:
    world: ...
    link_base: ...
    robot_origin:
      use_joint_angles: true
      joint_angles_ref: "joints.zero"  # Reference to joints section
    Home:
      use_joint_angles: true
      joint_angles_ref: "joints.home"
    WorkFrame: ...
    WorkIK: ...
    EndEffectorOrientation: ...
```

### 6. visualization
Unity ScriptableObject and ROS2 marker settings.
```yaml
visualization_config: "lite6_visualization"
ros2_visualization:
  reference_frame: "link_base"
  features: {...}
  markers: {...}
  colors: {...}
```

### 7. admittance (Optional)
Admittance controller parameters.
```yaml
admittance:
  dynamics:
    inertia: [10.0, 10.0, 10.0]      # kg
    damping: [3.0, 3.0, 3.0]         # Ns/m
    stiffness_p: [0.0, 0.0, 0.0]     # N/m (position)
    stiffness_d: [0.0, 0.0, 0.0]     # Ns/m (velocity)
  control: {...}
  safety: {...}
  equilibrium: {...}
  initial_state: {...}
```

## Usage

### Unity (C#)
```csharp
var config = YamlLoader.Load($"network_robots/{robotModel}.yaml");
var jointAngles = config.joints.home;  // degrees, use directly
```

### ROS2 (Python)
```python
import math
config = yaml.safe_load(open(f"network_robots/{robot_model}.yaml"))
# Convert degrees to radians for ROS2
joint_angles_rad = [math.radians(deg) for deg in config['joints']['home']]
```

### HTML Panel (JavaScript)
```javascript
const config = await fetch(`/config/network_robots/${robotModel}.yaml`);
// Use degrees directly for UI display
```

## Adding a New Robot

1. Create `{robot_model}.yaml` using `lite6.yaml` as template
2. Update robot-specific values:
   - Robot info (model, manufacturer, dof)
   - Joint names, limits, zero/home positions
   - Frame names (ros_frame, prefab_links)
   - FK positions for named_frames
3. Optionally add admittance parameters

## Migration Notes

- **ROS2 config folder**: Goal is to eliminate `ros2/demo/config/robots/` folder
- **ROS2 named_frames**: Should read from this shared_data location
- **Degrees vs Radians**: All files use DEGREES; ROS2 code converts at runtime
