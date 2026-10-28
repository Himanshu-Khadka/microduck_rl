# First-pass validation

Source baseline: `bae39f5900b318f7e87fc485c30a31213cfa14eb` (user's `develop` branch).

The build completed with the supplied STEP files. The JSON report records their hashes and geometric transforms.

| Check | Result |
|---|---|
| Modified upper-leg STL | Watertight, positive volume, one connected component |
| Hip servo housing intersection before pocket cut | 25.407 mm³ |
| Hip servo housing intersection after pocket cut | 0.000 mm³ |
| Knee servo housing intersection before/after | 0.000 / 0.000 mm³ |
| Expanded pocket intersection after subtraction | Below 1e-4 mm³ numerical tolerance |
| CAD regression tests | 3 passed: datum mapping, connected/clear pocket geometry, local/world frame agreement |
| Ruff | Check and format passed |

The knee housing already cleared the uniformly scaled mesh in this pose. Cutting its pocket adds the specified rectangular clearance/access envelope; it does not demonstrate a necessary collision fix there. The larger through-cuts remove additional roof material beyond the measured initial housing collision.

These are static, tessellated-housing checks. They are not validation of motor retention, moving horn fit, hardware tolerances, neighbouring links, joint range, strength, electronics packaging or locomotion. The existing XL330-based simulation and policies are unchanged.
