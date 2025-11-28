# Shared Configuration

Single Source of Truth for Unity, ROS2, and HTML panel configurations.

## Directory Structure

```
shared_data/config/
├── README.md                    # This file
├── network_config.yaml          # Network/ROS2 connection settings
├── device_config.yaml           # Device settings
├── map_data.yaml               # Map/environment data
│
├── network_robots/             # Robot configurations
│   ├── README.md               # Robot config documentation
│   ├── lite6.yaml              # UFACTORY Lite6 (6 DOF)
│   ├── xarm5.yaml              # UFACTORY xArm5 (5 DOF)
│   ├── fr3.yaml                # Franka Research 3 (7 DOF)
│   ├── ur3e.yaml               # Universal Robots UR3e (6 DOF)
│   ├── ur5e.yaml               # Universal Robots UR5e (6 DOF)
│   ├── ur16e.yaml              # Universal Robots UR16e (6 DOF)
│   ├── iiwa7.yaml              # KUKA iiwa7 (7 DOF)
│   ├── med7.yaml               # KUKA MED7 (7 DOF)
│   ├── med14.yaml              # KUKA MED14 (7 DOF)
│   └── kinova_gen3_7dof.yaml   # Kinova Gen3 (7 DOF)
│
├── unit_patterns/              # Movement patterns (unit-based)
│   ├── README.md               # Pattern documentation
│   ├── patterns_registry.yaml  # Pattern registry & adjustable params
│   ├── square_pattern.yaml     # Square shape
│   ├── diamond_pattern.yaml    # Diamond shape
│   ├── triangle_pattern.yaml   # Triangle shape
│   ├── circle_pattern.yaml     # Circle (generated)
│   ├── spiral_pattern.yaml     # Spiral (generated)
│   └── hemisphere_pattern.yaml # Hemisphere (generated)
│
└── policies/                   # Strategy/policy configs
    └── robot_visualization_strategy.yaml
```

## Unit Conventions

| Category | Parameter | Unit | Notes |
|----------|-----------|------|-------|
| Robots | Joint angles | **DEGREES** | ROS2 converts to radians at runtime |
| Robots | Positions | METERS | All position values |
| Robots | Orientations | Quaternion [x,y,z,w] | ROS2 convention |
| Patterns | Positions | **UNIT (-1 to 1)** | Scaled by `defaults.size` |
| Patterns | Size/Radius | METERS | Scaling factor |
| Patterns | Duration | SECONDS | Time per waypoint |

## Consumers

| Consumer | Config Files | Notes |
|----------|-------------|-------|
| Unity | `network_robots/*.yaml` | Uses degrees directly |
| ROS2 | `network_robots/*.yaml`, `unit_patterns/*.yaml` | Converts degrees→radians |
| HTML Panel | All | For UI display and adjustment |

## Key Features

### network_robots/

- **Single file per robot** containing all config (Unity + ROS2 + HTML)
- Unified structure: `robot`, `joints`, `frame_hierarchy`, `ros2`, `named_frames`, `visualization`, `admittance`
- Joint angles in DEGREES (human-readable)
- See [network_robots/README.md](network_robots/README.md) for details

### unit_patterns/

- **Unit-based coordinates** (-1 to 1 range)
- `defaults` section for adjustable parameters via HTML
- Actual position = `unit_value * defaults.size`
- See [unit_patterns/README.md](unit_patterns/README.md) for details

## Usage Examples

### Unity (C#)

```csharp
// Robot config
var robot = YamlLoader.Load("network_robots/lite6.yaml");
var homeJoints = robot.joints.home;  // degrees

// Pattern
var pattern = YamlLoader.Load("unit_patterns/square_pattern.yaml");
var size = pattern.defaults.size;
var actualX = pose.position.x * size;
```

### ROS2 (Python)

```python
import yaml
import math

# Robot config - convert degrees to radians
robot = yaml.safe_load(open("network_robots/lite6.yaml"))
home_rad = [math.radians(d) for d in robot['joints']['home']]

# Pattern - scale unit to actual
pattern = yaml.safe_load(open("unit_patterns/square_pattern.yaml"))
size = pattern['defaults']['size']
actual_x = pose['position']['x'] * size
```

### HTML Panel (JavaScript)

```javascript
// Load configs
const robot = await loadYaml('/config/network_robots/lite6.yaml');
const pattern = await loadYaml('/config/unit_patterns/square_pattern.yaml');

// Adjust via UI
pattern.defaults.size = 0.15;  // User changes size to 15cm
```

## Migration Goals

- [ ] Eliminate `ros2/demo/config/robots/` folder
- [ ] Eliminate `ros2/demo/config/*_pattern.yaml` files
- [ ] ROS2 reads all config from `shared_data/config/`
