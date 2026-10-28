# Microduck STS3215 conversion: first hip/upper-leg fit study

This starts the requested conversion with the **original Pollen Robotics geometry**. It scales the structural references, cuts larger motor pockets into the original left upper-leg mesh, and positions two **true-size** STS3215 STEP assemblies at its hip-pitch and knee axes.

**Status: mechanical fit study, not a completed robot redesign or a functional/load-bearing leg.** The open pockets remove the XL330 mounting features. They have no completed motor retention or horn coupling. Do not use this part to carry a robot.

## Requirements carried forward

- Work from the original Microduck assets, rather than the earlier community reconstruction.
- Start at the hip/upper leg; retain joint topology and axis directions.
- Scale the robot geometry, not the supplied STS3215 or Jetson hardware.
- Redesign the torso around the true-size Jetson, battery and controller/IMU hardware.
- Deliver modified original geometry; standalone generic mounting plates do not meet the task.
- The full conversion ultimately needs revised masses, inertias, collision checks and servo dynamics before retraining. This study changes no live simulation model or robot runtime.

The newly supplied `Microduck.zip` contains the software repository and original simulation meshes, not the previously discussed community assembly. It has no STEP/Fusion robot assembly. The meshes and MJCF are usable source geometry, but not an editable parametric manufacturing CAD release.

## This iteration

**1.6× is a provisional working scale**, not a confirmed final robot size. The source STL units are metres; outputs convert to millimetres explicitly. The supplied motor housing measures 45.4 × 24.8 × 35.3 mm. Its complete assembly includes connectors, horns and a screw, giving about 42.1 mm total Z span.

The original upper-leg joint offsets are `(22, 35.7771, -4)` mm. At 1.6× they become `(35.2, 57.24336, -6.4)` mm. The two aligned motor housing boxes have a 9.64336 mm Y gap. This is a static layout measurement, not a full joint-range clearance certification.

Each motor pocket is 46.2 × 25.6 mm, providing 0.4 mm nominal clearance per housing side. The holes around the horn axis are enlarged to 21 mm diameter. These are open-through fit pockets, deliberately leaving access for inspection; remaining scaled screw/bearing features must not be treated as compatible hardware interfaces.

The STEP output axis passes through `(12.5, 0)` in XY, and the outer horn face is Z=18.7 mm. This study puts that face at the existing joint plane. That datum placement is an explicit interface assumption. The four top-horn hole centres form a 9.9 × 9.9 mm square in the supplied model. No fastener thread, engagement, load rating or production tolerance is inferred from those circles.

The supplied P3766/P3768/P3767 assembly measures about **103 × 90.877 × 34.766 mm**, including modeled protrusions. Its full 1,393-solid STEP is measured without scaling. The torso has not been redesigned in this iteration. The earlier 75 × 34 × 26.5 mm battery envelope remains provisional, and controller/cable envelopes still need confirmation from the actual hardware.

## Build

Use a separate CAD environment; these packages are not ML training dependencies.

```bash
python -m venv .venv-cad
. .venv-cad/bin/activate
pip install -r cad/sts3215/requirements.txt
python cad/sts3215/build_fit_study.py \
  --servo /path/to/servo-motor_sts3215-v10.step \
  --jetson /path/to/P3766-P3768SKU4-P3767ENVELOPE.stp \
  --output /path/to/Microduck_STS3215_first_pass
python -m unittest discover -s cad/sts3215 -p 'test_*.py'
```

`--scale` and `--clearance` expose the two fit-study choices. The script rejects unexpected servo housing dimensions and invalid/disconnected candidate meshes. The original model files are read only. `fit_report.json` records input hashes, transforms, dimensions and measured housing intersections.

## Open the output

- `hip_fit/upper_leg_fit_assembly_mm.3mf`: assembled candidate and two real-size motors; useful for a CAD/mesh viewer. This is **not a print job**: motor meshes are references.
- `hip_fit/upper_leg_left_open_pocket_FIT_ONLY_mm.stl`: modified original upper-leg mesh, one closed connected solid, in the local hip-pitch frame. A dimensionally calibrated fit trial only.
- `hip_fit/hardware_only_1to1.step`: two actual STEP servo assemblies in that same frame. In Fusion, import this STEP and insert the candidate STL with **millimetres** selected. Do not auto-centre either import.
- `hip_fit/upper_leg_left_scaled_reference_mm.stl`: unchanged scaled upper leg in the same frame, for overlay comparison.
- `hip_fit/upper_leg_fit_assembly.glb`: colored viewer version, correctly converted to glTF metres.
- `scaled_structure_REFERENCE_ONLY_mm.3mf`: original structural geometry at 1.6× in the MJCF zero pose. Electronics, XL330s and commercial bearings are omitted; their old seats remain unresolved.
- `one_leg_study_OTHER_INTERFACES_UNRESOLVED_mm.3mf`: the candidate and two STS3215s placed in that structural reference. Neighbouring joints, original rigidity plate, shells and other interfaces are **not fitted** by this study.
- `scaled_structural_meshes/`: individual reference meshes in their original part-local frames, converted to millimetres and enlarged. They are not a converted production parts set.
- PNGs: rendered views from the actual output meshes.

The remeshed leg is editable through the build script or mesh editing; it is not falsely represented as an analytic, parametric STEP solid. The original supplied hardware STEP files are not redistributed in GitHub.

## Validation and next geometry work

The build checks positive volume, watertightness, single connected component, and mesh intersection against both tessellated servo housings and their enlarged rectangular pockets. These checks apply only to static housing fit. They do not validate screws, plug insertion, cable bends, strength, moving horns or neighbouring links.

Next: redesign the horn connection and opposite support in the hip link; add case retention tied into the upper leg; sweep hip/knee ranges with the adjacent links; then apply the established interface to the other joints. Rebuild torso packaging around actual Jetson, battery and controller envelopes. Re-measure mass properties and validate motor dynamics before changing training assets or running a policy.

## Attribution

Original robot geometry: **Pollen Robotics**, https://github.com/pollen-robotics/microduck_rl.
The scaled and modified robot meshes in this study are adaptations under **CC BY-NC-SA 4.0**: https://creativecommons.org/licenses/by-nc-sa/4.0/ . See the repository's `LICENSE-MODELS.md`. The changes are scaling, frame transformation, and clearance-pocket subtraction. Code and MJCF licensing follows the repository's Apache-2.0 license.
