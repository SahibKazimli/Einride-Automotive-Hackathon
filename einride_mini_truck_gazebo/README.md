# einride_mini_truck_gazebo

This subfolder holds example source files and a corresponding `CMakeLists.txt` file, as a starting point for compiling Gazebo implementations in a personal repository (i.e. not part of the official Gazebo source repositories).

The provided `CMakeLists.txt` file contains the directives to compile two example Gazebo systems: `BasicSystem` and `FullSystem`.

For more information on Gazebo Sim systems, see following [Gazebo Sim tutorials](https://gazebosim.org/api/sim/7/tutorials.html):

- [Create System Plugins](https://gazebosim.org/api/sim/7/createsystemplugins.html)
- [Migration from Gazebo Classic: Plugins](https://gazebosim.org/api/sim/7/migrationplugins.html)


## `BasicSystem` and `FullSystem`

`BasicSystem` is an example system that implements only the `ISystemPostUpdate` interface:

```c++
 class BasicSystem:
    public gz::sim::System,
    public gz::sim::ISystemPostUpdate
```

`FullSystem` is an example system that implements all of the system interfaces:

```c++
class FullSystem:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemUpdate,
    public gz::sim::ISystemPostUpdate,
    public gz::sim::ISystemReset
```

See the comments in the source files for further documentation.

## `CMakeLists.txt`

The provided `CMakeLists.txt` file contains comments that clarify the different sections and commands, and how to apply these to your project.
## Worlds and models

`worlds/einride_mini_truck.sdf` is the `demo` world the bringup launch files
load. It includes the rover from `einride_mini_truck_description` and the
delivery dock below.

`worlds/arena.sdf` is a 6 x 6 m walled arena with eight docks around the
perimeter, two per wall at +-1.5 m from each wall's midpoint - which puts every
consecutive pair exactly 3.000 m apart along the 24 m perimeter, corners
included - plus thirteen props that break the symmetry (see **Props** below).
Run it with:

```bash
ros2 launch einride_mini_truck_bringup simulation.launch.py world:=arena
```

Nothing else changes: the launch file already takes the world basename, and no
bridged topic is scoped to the world name. The dock layout table is in the
header of `arena.sdf`.

`warehouse_dock_a` .. `warehouse_dock_h` are the docks: a shallow trapezoidal
bay with a 100 mm tag36h11 AprilTag on the back wall, built to the geometry in
`DOCK_PLAN.md`. Arriving at a dock's `dock_approach_pose` frame *is* the
delivery - nothing is physically transferred. Dock A carries tag id 0 through to
dock H with id 7, and each dock's letter decal matches its tag. The dock is
three structural panels - middle (back wall), left wing, right wing - all
350 mm tall; the letter lives on the back wall's own free zone (z 269-341)
rather than on a separate pylon, so it costs no extra panel.

**They are generated, not checked in eight times.** `dock_template/model.sdf.in`
holds every dimension once and `CMakeLists.txt` configures it per dock from the
`DOCK_ASSIGNMENTS` list. Edit the template, never a generated `model.sdf` - the
generated ones live in the build tree and say so at the top. Adding a dock means
adding one entry to that list plus two PNGs in `dock_template/textures/`.

They install under `worlds/` rather than a sibling `models/` on purpose: the
environment hook already puts `share/einride_mini_truck_gazebo/worlds` on
`GZ_SIM_RESOURCE_PATH`, so `model://warehouse_dock_a` resolves with no change to
`CMakeLists.txt` or the hooks.

The tag texture is the official `apriltag-imgs` 10x10 PNG upscaled 100x with
nearest-neighbour interpolation. Do not re-export it with a smooth filter -
blurred cell edges stop the detector cold. `tag36h11_00000.png` .. `tag36h11_00007.png` ship
for each dock, alongside `letter_a.png` .. `letter_h.png`; CMake copies only
the two a given dock references, so each generated model stays self-contained
the way Gazebo expects.

The dock's two styling elements - the dark accent band (z 200-260) and the
letter decal (z 269-341) - both sit clear of the 80-180 mm scan band and of
the tag. `DOCK_PLAN.md` section 3.3 states that envelope as a contract; check
any new decoration against it, because the sensing behaviour of this model
depends on those two bands staying plain. `dock_template/textures/wordmark.png`
is a leftover asset from an earlier revision that had a wordmark crown above
the back wall; that panel no longer exists, so the texture is unused and safe
to delete.


## Props, and why they are there

The dock ring alone is **exactly D4-symmetric**: 4-fold rotation plus both mirrors
and both diagonals, eight group elements. Under an exact symmetry a 2D scan taken at
pose `p` is indistinguishable from one taken at `g(p)`, so a scan matcher has eight
equally good answers and no way to choose. The arena was also visually near-blank
between the docks, which is close to the worst case for a feature-based visual front
end.

`props/` holds three vendored Gazebo Fuel assets - a construction barrel, a jersey
barrier and a traffic cone, all CC0 - placed as thirteen instances in five clusters:
a different arrangement in each of the four (dock-free) corners, plus one off-centre
interior cluster that occludes and forces a drive-around. `props/README.md` carries
provenance, licence and the measured geometry; `worlds/arena.sdf`'s header carries
the layout and the reasoning behind each cluster.

Like the docks, the prop models are **generated**: `prop_template/model.sdf.in` plus
one scale table in `props/props.cmake`. Edit those, never the copies in the build
tree. Only upstream `meshes/` and `materials/` are vendored byte-for-byte - the
`model.sdf` is ours, because upstream's carries a 500 kg non-static barrel and a
jersey barrier whose collision is six hand-written boxes at full scale. Both traps
are documented in `prop_template/model.sdf.in`.

### Measured effect

Measured 2026-09-12 against the running simulator. The tooling that produced these
has since been removed, so treat them as a **record of what was true when the props
were placed**, not as something that re-runs.

| | before props | after props |
|---|---|---|
| minimum non-identity `D(g)` over the eight D4 elements | **0.0000** | **0.3381** |
| symmetric pose twins a 2D scan can tell apart | 0 of 38 by construction | **38 of 38**, worst 230 mm RMS on a 4.8 mm noise floor |
| ORB features per frame, median / fraction usable | - | **843** / **95.7 %** at >= 150, no position blind in every direction |
| all eight dock tags detected at `dock_approach_pose` | yes | yes, decision margin ratio **1.00** |

`D(g)` is the symmetric difference between the arena's obstacle footprint at the
z = 145 mm scan plane and its own image under `g`, over their union. Per element:

| D4 element | before | after |
|---|---|---|
| rot 90 / 180 / 270 | 0.0000 | 0.4349 / 0.4386 / 0.4349 |
| mirror x / y | 0.0000 | 0.4426 / 0.4282 |
| mirror diag | 0.0000 | 0.4467 |
| mirror anti-diag | 0.0000 | **0.3381** |

`0.0000` on all eight is not a rounding - the pre-prop arena is bit-exactly
D4-symmetric, which is both the problem statement and the proof the metric was
implemented correctly. The weakest element after props is the anti-diagonal mirror,
which maps NE to SW: the two clusters most alike, three objects each. The LiDAR twin
test agreed independently - its closest pairs were also the anti-diagonal ones.

Three findings worth keeping even though the scripts are gone:

- **The cones are marginal LiDAR landmarks.** All three sit at exactly 3 returns at
  the scan plane, the minimum to detect but not to fit. The beam budget says why: a
  59 mm cross-section gives 4.2 returns at 1 m but 1.1 at 4 m. They earn their place
  as camera features.
- **The arena's remaining visual weak spot is directional, not positional.** The only
  sub-threshold camera frames were at (+-1.8, +-0.6) facing 0 or 180 deg - close to a
  wall, looking at the bare span *between* two docks with no dock or prop in frame.
  Every one of those positions is feature-rich in other directions. Closing it would
  mean putting something on the wall midspans.
- **`<ambient>1.0 1.0 1.0</ambient>` in `arena.sdf` flattens shading**, so ORB counts
  there are almost entirely texture-driven rather than geometry-driven.

## Two hazards this arena will hand you

Both cost time here, and neither announces itself.

**A missing mesh loads a working world with an invisible prop.** An unresolvable
`model://` URI does not crash Gazebo - you get a loaded arena, a prop that is
invisible and intangible, and any measurement you take then silently describes a
prop-free arena. "The sim started" is not evidence that the props are there; check
the scene graph for the mesh path, or that the LiDAR actually returns beams where the
prop should be.

**`gz set_pose` preserves the model's velocities**, and anything publishing
`/cmd_vel` will drive the rover out from under whatever you are capturing. If you
teleport the robot to measure something, read the pose back afterwards and confirm it
stuck - in yaw as well as position.
