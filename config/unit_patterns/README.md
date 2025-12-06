# Unit Patterns Configuration

Unit-based waypoint and generated pattern configurations for robot motion planning.

## Unit-based System

All position values are **normalized (-1 to 1)** and scaled by `defaults` parameters:

```yaml
# Example: square_pattern.yaml
defaults:
  size: 0.20        # meters (scaling factor)
  height: 0.0       # meters (z offset)
  duration: 2.0     # seconds per waypoint

named_target_poses:
  corner1:
    position:
      x: -1.0       # unit value
      y: -1.0       # unit value
      z: 0.0
```

**Actual position calculation:**
```
actual_x = unit_x * defaults.size  # -1.0 * 0.20 = -0.20m
actual_y = unit_y * defaults.size  # -1.0 * 0.20 = -0.20m
actual_z = unit_z + defaults.height
```

## Pattern Types

### Waypoint-based Patterns
Pre-defined waypoints loaded from YAML files:
- `square_pattern.yaml` - Square shape (4 corners)
- `diamond_pattern.yaml` - Diamond shape (center + 4 cardinal points)
- `triangle_pattern.yaml` - Equilateral triangle (3 vertices)

### Generated Patterns
Parameters for runtime generation:
- `circle_pattern.yaml` - Circular motion
- `spiral_pattern.yaml` - Outward spiral
- `hemisphere_pattern.yaml` - 3D hemisphere surface

## Common Configuration

All patterns use:
```yaml
coordinate_system: ENU           # East-North-Up
reference_frame: world           # Planning reference
target_reference_frame: WorkFrame # Work coordinate frame
```

## Coordinate System (ENU)
- **E (East)**: +X direction
- **N (North)**: +Y direction
- **U (Up)**: +Z direction

## HTML Panel Integration

The `defaults` section can be adjusted via HTML panel:

```javascript
// Load pattern
const pattern = await fetch('/config/unit_patterns/square_pattern.yaml');

// Adjust defaults via UI sliders
pattern.defaults.size = 0.15;  // Change size to 15cm

// Calculate actual positions
const actualX = pose.position.x * pattern.defaults.size;
const actualY = pose.position.y * pattern.defaults.size;
```

## Pattern Registry

See `patterns_registry.yaml` for:
- Available patterns and their types
- Adjustable parameter ranges (min/max/unit)
- Pattern groups (basic_2d, advanced, 3d_patterns, etc.)
- Speed profiles (slow, normal, fast, test)
- Safety constraints

## Usage

### ROS2 (Python)
```python
import yaml

registry = yaml.safe_load(open("unit_patterns/patterns_registry.yaml"))
pattern = yaml.safe_load(open("unit_patterns/square_pattern.yaml"))

# Scale unit positions to actual
size = pattern['defaults']['size']
for pose in pattern['named_target_poses'].values():
    actual_x = pose['position']['x'] * size
    actual_y = pose['position']['y'] * size
```

### HTML Panel (JavaScript)
```javascript
const registry = await fetch('/config/unit_patterns/patterns_registry.yaml');
const pattern = await fetch('/config/unit_patterns/square_pattern.yaml');
```

## Adding a New Pattern

1. Create `{name}_pattern.yaml` with:
   - `defaults` section with scaling parameters
   - Unit positions (-1 to 1 range)
2. For waypoint patterns: Add `named_target_poses` and `target_sequence`
3. For generated patterns: Add `pattern_type` and generator parameters
4. Register in `patterns_registry.yaml` under `patterns` section
5. Add `adjustable` ranges for HTML panel controls
6. Add to appropriate `pattern_groups`
