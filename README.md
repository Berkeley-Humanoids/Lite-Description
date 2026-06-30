# Lite Description

[![Python](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/)
[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E.svg)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/license-MIT-yellow.svg)](LICENSE)

Robot description for the **Berkeley Humanoid Lite V2** — a low-cost, open-source 
humanoid built on [Robstride](https://robstride.com) actuators. This repository is the
single source of truth for the robot's geometry: it holds the **URDF**, **MJCF**, and
**xacro** (with a `ros2_control` block) descriptions and meshes, all generated from the
Onshape CAD.

<!-- TODO: add a render / photo of the Lite robot here. -->

The same source serves both worlds:

- **Simulation / RL** — Mujoco Lab and Isaac Lab, via the `robot_assets` Python loader.
- **ROS 2** — `ros2_control` + `robot_state_publisher`, as the `lite_description`
  ament package.

## Variants

| Variant | Description | DoF | Root | `ros2_control` |
|---|---|---|---|---|
| `lite` | full-body Lite humanoid (legs, 1-DoF waist yaw, arms, neck, 5-finger hands) | 72 | `pelvis` | model-only |
| `lite_pro` | full-body Lite **Pro** humanoid (legs, 3-DoF waist, arms, neck, 5-finger hands) | 74 | `pelvis` | model-only |
| `lite_dummy` | V1 bimanual upper body (arms + neck) — the configuration `humanoid_control` deploys | 17 | `chest` | yes — Robstride on two CAN buses |
| `lite_bimanual` | V2 bimanual arms (no neck) | 14 | `chest` | yes — Robstride on two CAN buses |
| `lite_biped` | V2 legs (hip ×3 / knee / ankle ×3 per leg) | 14 | `pelvis` | model-only |

All variants are generated from the same Onshape document (different `Configuration=…`); the
`<robot>.urdf.xacro` of a `ros2_control` variant selects sim / mock / real hardware via xacro args.

## CAD source

Every description is generated from an Onshape assembly (document
`e9ee61a2e2678af2088d9f31`) by the `robot_assets` tool — see
[Re-generating from CAD](#re-generating-from-cad). The files under `robots/<variant>/`
are build artifacts: **do not hand-edit them**; change the `cad/` inputs and regenerate.

## Usage

### Simulation / RL (Python, no ROS toolchain)

```bash
uv add git+https://github.com/Berkeley-Humanoids/Lite-Description.git
```

```python
from robot_assets import load

urdf_path = load("robots/lite/urdf/lite.urdf")               # Isaac Lab
mjcf_path = load("robots/lite_dummy/mjcf/lite_dummy.xml")    # MuJoCo
```

`load()` fetches and caches the requested variant's subtree from this GitHub repo; no ROS
install required.

### ROS 2

`lite_description` is a standard `ament_cmake` package (its `package.xml` is at the repo
root). Build it in a ROS 2 workspace — or pull it via `vcs` / `humanoid_control.repos` from `humanoid_control`
— and `colcon build`. Downstream, `robot_state_publisher` runs xacro on
`robots/<variant>/xacro/<variant>.urdf.xacro`, and
`package://lite_description/robots/<variant>/meshes/visual/...` resolves after install.
The `<ros2_control>` block selects the hardware backend via xacro args
(`use_sim` / `use_fake_hardware`), following the `franka_ros2` / Universal Robots
convention.

## Repository layout

```
Lite-Description/                 # repo root == ament package "lite_description"
  package.xml  CMakeLists.txt      # ament (colcon); installs robots/<variant>/...
  pyproject.toml                   # pip/uv: builds the robot_assets Python module
  robot_assets/                    # Python module: the CAD->assets generator + load()
  robots/                          # per-variant assets (franka_description-style subdir)
    <variant>/
      xacro/                       #   ROS entry (GENERATED)
        <variant>.urdf.xacro       #     top assembly: args + includes + instantiation
        <variant>.description.xacro #    <xacro:macro> model: kinematics, ${mesh_root}, base_link
        <variant>.ros2_control.xacro #   hardware macros: sim / mock / real, MIT interfaces, CAN ids
      urdf/<variant>.urdf          #   flat URDF (GENERATED; the kinematic HUB; base_link-free)
      mjcf/<variant>.xml           #   MJCF (GENERATED; MuJoCo training + deployment sim)
      meshes/visual/*.stl          #   one shared mesh copy
      cad/                         #   generation INPUTS (not installed):
        config.json                #     Onshape document + export options
        joint_properties.json      #     sim tuning: armature / friction / effort_limit
        physics.json               #     MJCF <option> (optional; deployment-sim tuning)
        ros2_control.json          #     ROS hardware map (optional; CAN ids / models / buses / modes)
        scad/                      #     collider sources
```

The committed `urdf/<variant>.urdf` is the single kinematic hub: the `mjcf` and `xacro`
stages both derive from it, so the three formats cannot drift. It is `base_link`-free
(`base_link` is a ROS/KDL concern injected only into the description xacro), which keeps
the finalize stage idempotent and the MJCF rooted at the CAD root link.

## Re-generating from CAD

```bash
uv sync
sudo apt install openscad        # for collider editing (onshape-to-robot)
```

One command produces all three formats from a variant's `cad/` inputs:

```bash
# Full pipeline (Onshape -> URDF + MJCF + xacro). Auto-skips the Onshape stage
# when a committed urdf/<variant>.urdf hub is already present.
uv run robot-assets-generate lite_dummy

# Skip the (expensive, ~1000-request) Onshape export and reuse the committed URDF hub:
uv run robot-assets-generate lite_dummy --from-cache

# Re-emit only some stages after editing physics.json / ros2_control.json:
uv run robot-assets-generate lite_dummy --only mjcf,xacro
```

Stages (the committed flat `urdf/<variant>.urdf` is the hub; MJCF and xacro both derive
from it, so the three formats share one kinematic origin):

| Stage | Reads | Writes |
|---|---|---|
| `onshape` | `cad/config.json`, `cad/scad/` | `urdf/<variant>.urdf`, `meshes/visual/` |
| `urdf` | `urdf/<variant>.urdf`, `joint_properties.json` | finalized `urdf/<variant>.urdf` (effort harmonised; base_link-free; idempotent) |
| `mjcf` | `urdf/<variant>.urdf`, `joint_properties.json`, `physics.json` | `mjcf/<variant>.xml` (`<option>`, actuators; no `<sensor>` block — joint state is read from sim data, and `mujoco_ros2_control` aborts on non-site sensors) |
| `xacro` | `urdf/<variant>.urdf`, `ros2_control.json` | `xacro/<variant>.*.xacro` (`base_link` injected here) |
| `package` | — | registers the variant in the repo-root `CMakeLists.txt` |

A variant without a `ros2_control.json` (`lite`, `lite_biped`) generates a **model-only**
package: a description macro + a thin assembly, no `<ros2_control>`.

### Editing colliders (OpenSCAD)

```bash
uv run robot-assets-onshape-to-urdf lite_dummy --keep-assets
cd robots/lite_dummy/cad/assets/
uv run onshape-to-robot-edit-shape ./chest.stl
```

## Tests

```bash
uv run pytest        # left/right symmetry, URDF<->MJCF parity, inertial plausibility,
                     # mesh existence, xacro well-formedness, generator-determinism
```

The `xacro` + `check_urdf` expansion checks run only where those tools are installed
(ROS / RoboStack-pixi); they are skipped in the plain `uv` environment and exercised by
the ROS CI job.

## License

MIT — see [LICENSE](LICENSE).
