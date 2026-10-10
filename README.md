[![Code: Apache-2.0](https://img.shields.io/badge/code-Apache--2.0-blue.svg)](LICENSE)
[![Assets: CC BY 4.0](https://img.shields.io/badge/assets-CC%20BY%204.0-lightgrey.svg)](assets/LICENSE)
![USD](https://img.shields.io/badge/format-OpenUSD-6a5acd.svg)
![URDF](https://img.shields.io/badge/format-URDF-22314e.svg)
![MJCF](https://img.shields.io/badge/format-MJCF-ff6f00.svg)
![Blender](https://img.shields.io/badge/format-Blender-e87d0d.svg?logo=blender&logoColor=white)
![SimReady](https://img.shields.io/badge/SimReady-Robot--Body%203.0.0-76b900.svg?logo=nvidia&logoColor=white)
![Newton](https://img.shields.io/badge/Newton-1.6-76b900.svg)

# Apollo Lunar Roving Vehicle (LRV-3, Apollo 17)

![Apollo 17 Lunar Roving Vehicle, Cycles render](docs/images/hero.jpg)

This is a simulation-ready model of the Apollo 17 Lunar Roving Vehicle. It is built procedurally in Blender from the LRV-3 Systems Handbook drawings, and its materials are colour-matched to the Smithsonian NASM close-up photographs and Apollo 17 Hasselblad frames. It ships as a SimReady/Isaac-style **USD** asset, plus **URDF**, **MJCF** and a **Blender** file, with working joints for:

- wheels
- double-Ackermann steering
- four-bar suspension
- the T-handle
- the high-gain antenna (HGA)
- the TV camera
- fenders and more

We also include decals for the US flag on the front fenders, the Control and Display Console faceplate (heading and speed indicators, power and temperature meters, attitude indicator, sun-shadow-device scale and switch legends), the operating instructions on the hand-controller housing, the LCRU communications panel legend and its HGA and LGA cable tags, the part numbers and seat-belt markings on the seats, and the stowage placards on the rear tool pallet.

## Quick Start

Download the format you need (`lrv-usd`, `lrv-urdf`, `lrv-mjcf` or `lrv-blend`) from the [latest release](https://github.com/alexmaietta/apollo-lrv-asset/releases/latest). To pin a version, use `download/v1.0.0` instead of `latest/download`; each release includes `SHA256SUMS`.

```sh
curl -LO https://github.com/alexmaietta/apollo-lrv-asset/releases/latest/download/lrv-usd.zip
unzip lrv-usd.zip   # -> lrv/lrv.usda
```

Or clone the repo for the source, the examples and the same assets in `assets/`, which are stored with Git LFS (set `GIT_LFS_SKIP_SMUDGE=1` to skip them). Then run the Newton example with [uv](https://docs.astral.sh/uv/):

```sh
git clone git@github.com:alexmaietta/apollo-lrv-asset.git && cd apollo-lrv-asset && git lfs pull
uv venv && source .venv/bin/activate
uv pip install "newton[examples]"
python examples/newton/example_robot_lrv.py   # i/k throttle, j/l steer, space stop
```

Newton's viewers simplify the materials. For a nicer image, use the ray-traced RTX viewer:

```sh
uv pip install "newton[rtx]"
python examples/newton/example_robot_lrv.py --viewer rtx
```

To explore the configurations (missions, the Apollo 17 fender repair, fender colours, flags and equipment), open `assets/lrv/lrv.usda` in Isaac Sim or another Omniverse app and switch variants in the Variant Presenter.

## Gallery

Cycles renders of the Smithsonian `nasm` configuration, framed like the museum's photographs. The last three show the flight configurations: the Apollo 17 right-rear fender repair (`a17_repair`), Apollo 17 on the surface (`a17`: LCRU radiator open, TV camera, SEP and traverse gravimeter, flags on the front fenders only) and Apollo 15 (`a15`: flags on all four fenders):

| | |
|---|---|
| ![front three-quarter](docs/images/front_three_quarter.jpg) | ![rear three-quarter](docs/images/rear_three_quarter.jpg) |
| ![side](docs/images/side.jpg) | ![rear left](docs/images/rear_left.jpg) |
| ![wheel and fender](docs/images/wheel_fender.jpg) | ![console and T-handle](docs/images/console_t_handle.jpg) |
| ![seats](docs/images/seats.jpg) | ![HGA](docs/images/hga.jpg) |
| ![LCRU](docs/images/lcru.jpg) | ![Apollo 17 fender repair](docs/images/a17_repair.jpg) |
| ![Apollo 17 surface configuration](docs/images/a17_surface.jpg) | ![Apollo 15 configuration](docs/images/a15.jpg) |

Each comparison panel puts a reference photo on the left and the model on the right (photos and credits in [`docs/references/`](docs/references/)):

![compare front](docs/images/compare_front_three_quarter.jpg)

![compare wheel](docs/images/compare_wheel_fender.jpg)

![compare console](docs/images/compare_console_t_handle.jpg)

![compare Apollo 17 fender repair](docs/images/compare_a17_repair.jpg)

<details><summary>More comparisons</summary>

![compare side](docs/images/compare_side.jpg)

![compare rear](docs/images/compare_rear_three_quarter.jpg)

![compare rear left](docs/images/compare_rear_left.jpg)

![compare seats](docs/images/compare_seats.jpg)

![compare LCRU](docs/images/compare_lcru.jpg)

![compare HGA](docs/images/compare_hga.jpg)

![compare Apollo 17 surface configuration](docs/images/compare_a17_surface.jpg)

![compare Apollo 15 configuration](docs/images/compare_a15.jpg)

</details>

## Capabilities

- **Articulation.** 72 links: 71 tree joints (48 revolute, 5 prismatic, 4 spherical, 14 fixed) plus 12 loop-closure joints, 8 for the suspension four-bars and 4 for the steering tie rods. The loop joints are `excludeFromArticulation` in USD and CONNECT equalities in MJCF. URDF has no loops, so it uses mimic joints instead.
- **Driven joints:**
  - 4 wheel velocity drives (14 km/h top speed)
  - 2 steering-sector position drives (front and rear). Each sector moves its two kingpins through ball-jointed tie rods, which gives the double-Ackermann geometry: inner wheel 50°, outer 22° at full lock
  - 3-axis hand controller (T-handle)
  - HGA azimuth, elevation, mast and telescope
  - TV camera pan and tilt (+214/−134°, +85/−45°)
  - LGA
  - fender extensions, chassis and seat folds, dust covers, tools
- **Suspension.** Double-wishbone torsion-bar suspension with dampers, tuned to the handbook ride height. Fenders steer with their wheels.
- **Sensors.** A `UsdGeom.Camera` at the GCTA TV camera, with real specs:
  - 12.5–75 mm zoom (9–54° FOV)
  - f/2.2–f/22
  - 1-inch tube, NTSC 640×480 at 29.97 fps
  - The camera rides on the pan/tilt joints.
- **Materials:**
  - **Textures.** Tileable PBR texture sets (base colour, roughness, metallic, normal), sized in real-world metres.
  - **Cut-outs.** Opacity cut-outs for the woven-wire tyres and the HGA mesh.
  - **Colour sampling.** Colours were sampled from the NASM photos.
  - **Decals.** Console legend, placards and the US flag on the front fender extensions.
- **Formats:**

  | Format | Path | Notes |
  |---|---|---|
  | USD | `assets/lrv/lrv.usda` | Primary asset. Newton, Isaac Sim / PhysX, MuJoCo schemas |
  | URDF | `assets/urdf/lrv.urdf` | Mimic joints for the four-bars; folds locked |
  | MJCF | `assets/mjcf/lrv.xml`, `assets/mjcf/lrv_scene.xml` | Connect equalities, springs, 16 actuators, lunar gravity scene |
  | Blender | `assets/blender/lrv.blend` | Source model with all textures packed; every configuration's parts included (non-default ones hidden, tagged with a `variant` custom property) |

### USD variants

| Variant set | Options (**default**) |
|---|---|
| `Physics` | none, physics (UsdPhysics + Newton), **physx**, mujoco |
| `Fidelity` | **full**, proxy (one box per part, no textures) |
| `Mission` | a15 (LRV-1: flags on all four fender extensions), **a17** (LRV-3: flags on the front extensions only, fender-extension stops), a17_repair (EVA-2/3 map-and-clamp right-rear fender repair), nasm (the Smithsonian display vehicle: maroon left-front extension, flags on all four extensions, grey flaps behind the front wheels, no TV camera; used for the gallery with the LCRU closed) |
| `WheelCollider` | cylinder, **mesh** |
| `BodyColliders` | wheels_only, chassis, **full** (chassis boxes plus convex hulls) |
| `Sensor` | none, **all** |
| `LCRU` | closed, **radiator_open** |
| `SeatBelts` | buckled, **stowed** |

## USD asset structure

The layout follows the Isaac Sim robot asset convention (the same layout as Isaac's Viper URDF-converter output) and the SimReady Robot-Body profile (`assets/lrv/`):

```
lrv/
  lrv.usda                    interface layer: variant sets, SimReady_Metadata, IsaacRobotAPI
  payloads/
    base.usda                 link hierarchy -> robot.usda
    robot.usda                IsaacRobotAPI links/joints, sites (seat H-points, footrests, handholds, T-handle grip)
    geometries.usd            all meshes and collider geometry (crate)
    instances.usda            instanceable prototypes (visual + physics materials)
    materials.usda            UsdPreviewSurface materials
    physics_materials.usda    PhysicsMaterialAPI + NewtonMaterialAPI
    Physics/{physics,physx,mujoco}.usda
    WheelCollider/, BodyColliders/, Fidelity/, Mission/ ...
    Textures/
  .thumbs/256x256/lrv.usda.png
```

## Validation

| Validator | Result |
|---|---|
| `simready-validate` 2026.8, profile Robot-Body 3.0.0 | **PASSED** (only optional features unmet: MDL/OpenPBR, ROS) |
| `nvidia_usd_validate` (usd-validation-nvidia 1.22, all variants) | 0 failures; warnings are mesh-quality only |
| Newton 1.6 (`SolverMuJoCo`) | Loads with all textured materials and settles at handbook ride height; the drive and steering yaw rate matches Ackermann geometry |
| MuJoCo 3.12 | `lrv_scene.xml` settles and turns within 3% of geometry |

```bash
.venv/bin/nvidia_usd_validate --group-by rule_name assets/lrv/lrv.usda
# SimReady specs: sparse checkout of the spec tier (~19 MB) from github.com/NVIDIA/simready-foundation
GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 --filter=blob:none --sparse https://github.com/NVIDIA/simready-foundation.git
git -C simready-foundation sparse-checkout set nv_core/tiers/simready_foundation_tier_core
T=simready-foundation/nv_core/tiers/simready_foundation_tier_core/simready/foundation/tier_core
.venv/bin/simready-validate --rules-path $T/capabilities --features-path $T/features --profiles-path $T/profiles \
    --profile Robot-Body --version 3.0.0 assets/lrv/lrv.usda
```

## Rebuild

You need Blender 5.2 on `PATH` (or set `BLENDER=/path/to/blender`) and a [uv](https://docs.astral.sh/uv/) venv with `bpy`, `usd-core`, `newton[importers,sim]`, `scipy` and `pillow`.

```bash
./pipeline.sh            # textures -> Blender build -> USD package -> URDF -> MJCF -> assets/ -> Newton test -> renders
./pipeline.sh package    # USD asset only (build/asset/lrv)
./pipeline.sh update     # rebuild USD, URDF and MJCF and copy them into assets/ (committed with Git LFS)
./pipeline.sh assets     # copy existing build outputs into assets/ (LRV_ASSETS="usd mjcf" for a subset; formats: usd urdf mjcf blend)
./pipeline.sh hero       # Cycles gallery -> docs/images
./pipeline.sh release    # stamp VERSION, rebuild, refresh assets/ and build the release zips
LRV_DRAFT=1 ./pipeline.sh textures build package   # fast iteration: 512 px textures
```

All dimensions live in `scripts/params.py`, with handbook drawing references.

The Newton example ([`examples/newton/`](examples/newton/), `./pipeline.sh example`) runs on stock Newton. Applying [`newton_viewer_materials.patch`](examples/newton/newton_viewer_materials.patch) to a Newton checkout carries more of the materials through to its GL and RTX viewers: metallic, roughness and normal maps, the see-through tyre wire and HGA knit, and smooth normals.

### Versions and releases

The asset version lives in `VERSION`. The packager stamps it into the USD:

- `assetInfo.version` and `isaac:version` on `/lrv`
- `asset_version` in the root layer's `customLayerData` and `SimReady_Metadata`, which also record the authoring OpenUSD version as `usd_version`

To cut a release:

1. Bump `VERSION` and run `./pipeline.sh release`. It rebuilds every format, refreshes `assets/` with the new version stamped in, and builds the release zips in `build/release/v<VERSION>/` so you can check them.
2. Commit `assets/` and `VERSION`, tag the commit `v<VERSION>`, and push the tag.
3. The [release workflow](.github/workflows/release.yml) runs on the tag. It checks that the tag matches `VERSION` and the stamped USD, zips the committed assets, and publishes the GitHub release with `SHA256SUMS`.

## Sources

The geometry comes from public-domain NASA documents and Boeing's LRV press material:

- LRV-3 Systems Handbook (MSC-07464)
- LRV Operations Handbook
- Apollo 17 Mission Report
- Boeing *Apollo 17 LRV Technical Information* booklet

Reference photos are Smithsonian NASM Open Access images (CC0) and NASA Apollo 15–17 Hasselblad frames (public domain).

## License

Copyright 2026 Alex Maietta (see [`NOTICE`](NOTICE)).

- **Code** (`scripts/`, `pipeline.sh`, `examples/`): [Apache License 2.0](LICENSE).
- **Model assets** (`assets/`, `textures/gen/`) and **renders** (`docs/images/`): [Creative Commons Attribution 4.0 International (CC BY 4.0)](assets/LICENSE). Credit: "Apollo LRV model by Alex Maietta".

Third-party material:

- `examples/newton/newton_viewer_materials.patch` modifies [Newton](https://github.com/newton-physics/newton) and, like the example script based on Newton's examples, is Apache-2.0, © The Newton Developers.
- The reference photographs in [`docs/references/`](docs/references/) and in the comparison panels keep their own licences: Smithsonian National Air and Space Museum Open Access images are CC0, and NASA Apollo frames are public domain.

This project is not affiliated with or endorsed by NASA or the Smithsonian Institution.
