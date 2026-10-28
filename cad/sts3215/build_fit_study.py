"""Reproducible first STS3215 fit study using the original Microduck meshes.

Outputs are fit-study geometry, NOT a released, load-bearing robot design.
STL inputs are metres; all exported CAD/mesh coordinates are millimetres.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import cadquery as cq
import numpy as np
import trimesh
from OCP.gp import gp_Trsf
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[2]
MODEL = REPO / "src/mjlab_microduck/robot/microduck"
STRUCTURAL = {
    "ankle_left",
    "ankle_right",
    "banana_pcb_locker",
    "bottom_head_shell",
    "face_part",
    "foot_left",
    "foot_right",
    "hip_l",
    "jaw",
    "jaw_soft",
    "left_shell",
    "leg",
    "m12_lens_holder",
    "motor_support",
    "neck",
    "neck_pitch",
    "noenoeil",
    "power_support",
    "right_shell",
    "soft_mouth_top",
    "sole_left",
    "sole_right",
    "top_head_shell",
    "trunk_base",
    "upper_leg_left",
    "upper_leg_right",
    "upper_leg_rigidity_plate",
    "yaw2roll",
    "yaw_roll_motion",
    "bearing_roll",
}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def transform(pos=(0, 0, 0), quat=(1, 0, 0, 0), units=1.0):
    q = np.asarray(quat, dtype=float)
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    t[:3, 3] = np.asarray(pos, dtype=float) * units
    return t


def element_transform(e, units=1.0):
    return transform(
        e.get("pos", "0 0 0").split(), e.get("quat", "1 0 0 0").split(), units
    )


def transformed(mesh, t):
    return mesh.copy().apply_transform(t)


def bounds(shape):
    b = shape.BoundingBox()
    return [[b.xmin, b.ymin, b.zmin], [b.xmax, b.ymax, b.zmax]]


def cq_mesh(shape):
    vertices, faces = shape.tessellate(0.15, 0.3)
    return trimesh.Trimesh(
        vertices=[v.toTuple() for v in vertices], faces=faces, process=True
    )


def servo_placements(scale):
    # Existing upper-leg frame: hip pitch origin (0,0,0), knee joint
    # (22,35.7771,-4) mm. Keep joint ordering/directions and scale offsets.
    # Servo STEP output axis = (x=12.5,y=0), outer horn face z=18.7.
    # Interface assumption: that outer face coincides with each joint plane.
    matrices = []
    for r, origin in [
        (np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]]), np.zeros(3)),
        (np.diag([1, -1, -1]), np.array([22, 35.7771, -4]) * scale),
    ]:
        t = np.eye(4)
        t[:3, :3] = r
        t[:3, 3] = origin - r @ np.array([12.5, 0, 18.7])
        matrices.append(t)
    return matrices


def original_upper(scale):
    root = ET.parse(MODEL / "robot_allcollisions.xml").getroot()
    body = root.find(".//body[@name='upper_leg_left']")
    geom = body.find("geom[@class='visual'][@mesh='upper_leg_left']")
    mesh = trimesh.load_mesh(MODEL / "assets/upper_leg_left.stl")
    mesh.apply_transform(element_transform(geom))
    mesh.apply_scale(1000 * scale)
    return mesh


def make_candidate(original, placements, clearance):
    cutters = []
    for t in placements:
        # Open-through pockets allow insertion and inspection. They deliberately
        # remove the XL330 mounting interface; retention is NOT yet designed.
        box = trimesh.creation.box([45.4 + 2 * clearance, 24.8 + 2 * clearance, 150])
        local = np.eye(4)
        local[:3, 3] = [0, 0, 25]
        cutters.append(transformed(box, t @ local))
        horn = trimesh.creation.cylinder(
            radius=10 + clearance + 0.1, height=150, sections=96
        )
        local[:3, 3] = [12.5, 0, 0]
        cutters.append(transformed(horn, t @ local))
    result = trimesh.boolean.difference([original, *cutters], engine="manifold")
    return result, cutters


def assembly_records(scale):
    root = ET.parse(MODEL / "robot_allcollisions.xml").getroot()
    records = []
    body_frames = {}

    def visit(body, parent):
        frame = parent @ element_transform(body, 1000 * scale)
        body_frames[body.get("name")] = frame
        for geom in body.findall("geom[@class='visual']"):
            name = geom.get("mesh")
            if name not in STRUCTURAL:
                continue
            mesh = trimesh.load_mesh(MODEL / "assets" / f"{name}.stl")
            mesh.apply_scale(1000 * scale)
            records.append(
                (
                    body.get("name"),
                    name,
                    transformed(mesh, frame @ element_transform(geom, 1000 * scale)),
                )
            )
        for child in body.findall("body"):
            visit(child, frame)

    for body in root.findall("worldbody/body"):
        visit(body, np.eye(4))
    return records, body_frames


def colored(mesh, color):
    m = mesh.copy()
    m.visual.face_colors = color
    return m


def save_scene(scene, path):
    if path.suffix == ".glb":
        scene = scene.copy()
        scene.apply_scale(0.001)  # glTF uses metres.
    path.write_bytes(scene.export(file_type=path.suffix[1:]))


def render_parts(items, path, scale, elev=25, azim=-60):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(10, 8), dpi=160)
    fig.patch.set_facecolor("#f2f5f8")
    ax = fig.add_subplot(projection="3d")
    ax.set_facecolor("#f2f5f8")
    vertices = []
    for mesh, color in items:
        tri = mesh.triangles
        normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-12)
        shade = 0.40 + 0.60 * np.abs(normals @ np.array([0.3, -0.5, 0.8]))
        poly = Poly3DCollection(
            tri,
            facecolors=np.clip(np.array(color) * shade[:, None], 0, 1),
            edgecolors="none",
            rasterized=True,
        )
        ax.add_collection3d(poly)
        vertices.append(mesh.vertices)
    vertices = np.vstack(vertices)
    lo = vertices.min(0)
    hi = vertices.max(0)
    mid = (hi + lo) / 2
    radius = max(hi - lo) * 0.55
    ax.set_xlim(mid[0] - radius, mid[0] + radius)
    ax.set_ylim(mid[1] - radius, mid[1] + radius)
    ax.set_zlim(mid[2] - radius, mid[2] + radius)
    ax.set_box_aspect([1, 1, 1])
    ax.view_init(elev, azim)
    ax.set_xlabel("X / mm")
    ax.set_ylabel("Y / mm")
    ax.set_zlabel("Z / mm")
    ax.grid(False)
    ax.set_title(
        "Original Microduck upper leg • STS3215 fit study\n1.6× structure / 1:1 hardware"
        if scale == 1.6
        else "Microduck STS3215 fit study",
        fontsize=15,
        pad=20,
    )
    fig.text(
        0.08,
        0.04,
        "Teal: modified original mesh   |   Orange: supplied servo STEP\nOpen pockets; retention and horn connections pending. Not load-bearing.",
        fontsize=11,
        color="#344054",
    )
    fig.subplots_adjust(left=0, right=1, bottom=0.12, top=0.88)
    fig.savefig(path)
    plt.close(fig)


def main(args):
    if not 0.1 <= args.clearance <= 2 or not 1 <= args.scale <= 3:
        raise ValueError("Unsupported scale or clearance")
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "scaled_structural_meshes").mkdir(exist_ok=True)
    (out / "hip_fit").mkdir(exist_ok=True)
    servo = cq.importers.importStep(str(args.servo)).val()
    solids = servo.Solids()
    housing = max(solids, key=lambda s: s.Volume())
    # Verify the exact source geometry assumptions before modifying a mesh.
    hb = np.array(bounds(housing))
    if not np.allclose(hb, [[-22.7, -12.4, -18.3], [22.7, 12.4, 17]], atol=0.02):
        raise ValueError(
            "Servo housing does not match this study; remeasure its axis and housing"
        )
    if len(solids) != 9:
        raise ValueError("Unexpected servo assembly")
    placements = servo_placements(args.scale)
    original = original_upper(args.scale)
    candidate, cutters = make_candidate(original, placements, args.clearance)
    if not candidate.is_volume or len(candidate.split()) != 1:
        raise ValueError(
            "Cut part must remain one watertight positive-volume component"
        )
    original.export(out / "hip_fit/upper_leg_left_scaled_reference_mm.stl")
    candidate.export(out / "hip_fit/upper_leg_left_open_pocket_FIT_ONLY_mm.stl")
    hardware = cq.Assembly(name="STS3215_true_size_hardware_only")
    servo_meshes = []
    checks = []
    fullmesh = cq_mesh(servo)
    housingmesh = cq_mesh(housing)
    for i, t in enumerate(placements):
        name = ["hip_pitch", "knee"][i]
        rigid = gp_Trsf()
        rigid.SetValues(*t[:3].ravel().tolist())
        shape = servo.moved(cq.Location(rigid))
        hardware.add(shape, name=name, color=cq.Color(0.87, 0.43, 0.17))
        m = transformed(fullmesh, t)
        servo_meshes.append(m)
        m.export(out / f"hip_fit/{name}_servo_reference_mm.stl")
        hs = transformed(housingmesh, t)
        before = trimesh.boolean.intersection([original, hs], engine="manifold").volume
        after = trimesh.boolean.intersection([candidate, hs], engine="manifold").volume
        pocket_overlap = trimesh.boolean.intersection(
            [candidate, cutters[2 * i]], engine="manifold"
        ).volume
        if abs(after) > 1e-4 or abs(pocket_overlap) > 1e-4:
            raise ValueError("Candidate intersects the servo housing/pocket envelope")
        checks.append(
            {
                "motor": name,
                "housing_overlap_before_mm3": float(before),
                "housing_overlap_after_mm3": float(after),
                "pocket_overlap_after_mm3": float(pocket_overlap),
            }
        )
    hardware.save(str(out / "hip_fit/hardware_only_1to1.step"))
    scene = trimesh.Scene()
    scene.add_geometry(
        colored(candidate, [35, 169, 184, 255]),
        node_name="Modified_original_upper_leg_FIT_ONLY",
    )
    for i, m in enumerate(servo_meshes):
        scene.add_geometry(
            colored(m, [223, 112, 44, 255]),
            node_name=["STS3215_hip_pitch", "STS3215_knee"][i],
        )
    save_scene(scene, out / "hip_fit/upper_leg_fit_assembly_mm.3mf")
    save_scene(scene, out / "hip_fit/upper_leg_fit_assembly.glb")
    records, frames = assembly_records(args.scale)
    reference = trimesh.Scene()
    study = trimesh.Scene()
    for i, (body, name, mesh) in enumerate(records):
        c = colored(mesh, [180, 191, 202, 255])
        reference.add_geometry(c, node_name=f"{i}_{body}_{name}")
        if body == "upper_leg_left" and name == "upper_leg_left":
            c = colored(
                transformed(candidate, frames["upper_leg_left"]), [35, 169, 184, 255]
            )
        # Original rigidity plate is an unresolved interface, not new retention.
        study.add_geometry(c, node_name=f"{i}_{body}_{name}")
    for i, m in enumerate(servo_meshes):
        study.add_geometry(
            colored(transformed(m, frames["upper_leg_left"]), [223, 112, 44, 255]),
            node_name=f"STS3215_{i}",
        )
    save_scene(reference, out / "scaled_structure_REFERENCE_ONLY_mm.3mf")
    save_scene(study, out / "one_leg_study_OTHER_INTERFACES_UNRESOLVED_mm.3mf")
    for name in sorted(STRUCTURAL):
        mesh = trimesh.load_mesh(MODEL / "assets" / f"{name}.stl")
        mesh.apply_scale(1000 * args.scale)
        mesh.export(out / "scaled_structural_meshes" / f"{name}_REFERENCE_mm.stl")
    jetson = cq.importers.importStep(str(args.jetson)).val()
    jetson_bounds = np.array(bounds(jetson))
    sources = {
        str(p.name): {"sha256": sha256(p), "bytes": p.stat().st_size}
        for p in [
            args.servo,
            args.jetson,
            MODEL / "robot_allcollisions.xml",
            MODEL / "assets/upper_leg_left.stl",
        ]
    }
    report = {
        "status": "first mechanical fit study; incomplete robot redesign; no training performed",
        "source_repository": "https://github.com/Himanshu-Khadka/microduck_rl",
        "upstream": "https://github.com/pollen-robotics/microduck_rl",
        "source_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=REPO, text=True
        ).strip(),
        "units": "STEP/STL/3MF in mm; GLB in metres per glTF convention",
        "scale": args.scale,
        "scale_status": "provisional fit-study choice, not a confirmed final robot scale",
        "clearance_per_side_mm": args.clearance,
        "housing_dimensions_mm": np.ptp(hb, axis=0).tolist(),
        "servo_complete_bounds_mm": bounds(servo),
        "servo_axis_STEP": {
            "point_mm": [12.5, 0, 18.7],
            "direction": [0, 0, 1],
            "datum": "outer horn face; joint-plane placement assumption",
        },
        "horn_hole_centres_STEP_mm": [
            [7.55, -4.95, 18.7],
            [7.55, 4.95, 18.7],
            [17.45, -4.95, 18.7],
            [17.45, 4.95, 18.7],
        ],
        "horn_hole_radius_STEP_mm": 1.26365,
        "joint_positions_in_upper_leg_frame_mm": [
            [0, 0, 0],
            (np.array([22, 35.7771, -4]) * args.scale).tolist(),
        ],
        "servo_STEP_to_upper_leg_matrices": [t.tolist() for t in placements],
        "housing_AABB_gap_y_mm": 35.7771 * args.scale - 12.4 - 35.2,
        "cut_pocket_dimensions_mm": [
            45.4 + 2 * args.clearance,
            24.8 + 2 * args.clearance,
        ],
        "original_scaled_volume_mm3": float(original.volume),
        "candidate_volume_mm3": float(candidate.volume),
        "candidate_watertight": bool(candidate.is_watertight),
        "candidate_components": len(candidate.split()),
        "housing_intersections": checks,
        "jetson_complete_bounds_mm": jetson_bounds.tolist(),
        "jetson_dimensions_mm": np.ptp(jetson_bounds, axis=0).tolist(),
        "jetson_solid_count": len(jetson.Solids()),
        "sources": sources,
        "unresolved": [
            "motor case retention",
            "horn attachment and opposite support",
            "hip/yaw/roll neighbouring link clearances",
            "cable and plug insertion/bend space",
            "Jetson/battery/controller torso packaging",
            "minimum wall and strength analysis",
            "joint-range sweep and all-robot collisions",
            "measured masses, inertias, servo dynamics, COM and retraining",
        ],
    }
    (out / "fit_report.json").write_text(json.dumps(report, indent=2) + "\n")
    shutil.copy2(REPO / "LICENSE-MODELS.md", out / "LICENSE-MODELS.md")
    shutil.copy2(REPO / "LICENSE", out / "LICENSE-APACHE-2.0.txt")
    render_parts(
        [(candidate, (0.12, 0.66, 0.73))]
        + [(m, (0.88, 0.44, 0.17)) for m in servo_meshes],
        out / "upper_leg_fit_preview.png",
        args.scale,
    )
    render_parts(
        [(candidate, (0.12, 0.66, 0.73))],
        out / "modified_upper_leg_preview.png",
        args.scale,
    )
    print(
        json.dumps(
            {
                "output": str(out),
                "watertight": report["candidate_watertight"],
                "components": report["candidate_components"],
                "checks": checks,
                "jetson_mm": report["jetson_dimensions_mm"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--servo", type=Path, required=True)
    parser.add_argument("--jetson", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scale", type=float, default=1.6)
    parser.add_argument("--clearance", type=float, default=0.4)
    args = parser.parse_args()
    main(args)
