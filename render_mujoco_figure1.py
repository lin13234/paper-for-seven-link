import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

SRC_DIR = Path(r"E:\hub\mujoco_render_source_ascii")
XML_PATH = SRC_DIR / "zhengti.xml"
MESH_DIR = SRC_DIR / "meshes"
TMP_OUT_DIR = Path(r"E:\hub\_tmp_mujoco_figure1_outputs")
FINAL_OUT_DIR = Path(r"E:\hub\02论文正文\001paper\03_图表与素材\机构图")
TMP_OUT_DIR.mkdir(parents=True, exist_ok=True)
FINAL_OUT_DIR.mkdir(parents=True, exist_ok=True)

PAPER_MAP = {
    "joint1": "J1",
    "joint2": "J2",
    "joint3": "J3",
    "joint4": "J4",
    "joint5": "J5",
    "joint6": "J6",
    "joint7": "J7",
}
ACTIVE_PLANAR = {"J1", "J2"}
ACTIVE_SPATIAL = {"J3", "J7"}


def euler_xyz_to_matrix(e):
    x, y, z = e
    cx, sx = math.cos(x), math.sin(x)
    cy, sy = math.cos(y), math.sin(y)
    cz, sz = math.cos(z), math.sin(z)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rx @ ry @ rz


def make_transform(pos, euler):
    T = np.eye(4)
    T[:3, :3] = euler_xyz_to_matrix(euler)
    T[:3, 3] = pos
    return T


def parse_vec(text, default):
    return np.array(list(map(float, text.split()))) if text else np.array(default, dtype=float)


def body_pos(body):
    return parse_vec(body.attrib.get("pos", "0 0 0"), [0.0, 0.0, 0.0])


def body_euler(body):
    return parse_vec(body.attrib.get("euler", "0 0 0"), [0.0, 0.0, 0.0])


def parse_assets(root):
    assets = {}
    for mesh in root.findall("./asset/mesh"):
        assets[mesh.attrib["name"]] = mesh.attrib["file"]
    return assets


def add_mesh(ax, mesh, facecolor="#d4d8dd", edgecolor="none", alpha=1.0):
    tris = mesh.vertices[mesh.faces]
    coll = Poly3DCollection(
        tris,
        facecolors=facecolor,
        edgecolors=edgecolor,
        linewidths=0.0,
        alpha=alpha,
        antialiaseds=False,
    )
    ax.add_collection3d(coll)


def make_disc_mesh(origin, axis, radius=0.012, thickness=0.004):
    disc = trimesh.creation.cylinder(radius=radius, height=thickness, sections=48)
    axis = axis / np.linalg.norm(axis)
    rot = trimesh.geometry.align_vectors(np.array([0.0, 0.0, 1.0]), axis)
    disc.apply_transform(rot)
    disc.apply_translation(origin)
    return disc


def draw_axis_line(ax, origin, axis, length=0.085, color="#2c60d3", lw=1.4):
    axis = axis / np.linalg.norm(axis)
    p1 = origin - axis * length / 2
    p2 = origin + axis * length / 2
    ax.plot(
        [p1[0], p2[0]],
        [p1[1], p2[1]],
        [p1[2], p2[2]],
        linestyle=(0, (3.2, 2.0)),
        color=color,
        linewidth=lw,
        alpha=0.95,
    )


def draw_frame(ax, origin, rot, scale=0.03, labels=("x", "y", "z"), linewidth=1.0, alpha=0.95):
    colors = ["#d62728", "#2ca02c", "#1f77b4"]
    for i, (lab, col) in enumerate(zip(labels, colors)):
        vec = rot[:, i] * scale
        ax.quiver(
            origin[0], origin[1], origin[2],
            vec[0], vec[1], vec[2],
            color=col,
            arrow_length_ratio=0.18,
            linewidth=linewidth,
            alpha=alpha,
        )
        tip = origin + vec * 1.16
        ax.text(tip[0], tip[1], tip[2], lab, fontsize=8, color=col)


def draw_symmetry_plane(ax, point, axis_dir, inplane_dir, width=0.23, height=0.125):
    axis_dir = axis_dir / np.linalg.norm(axis_dir)
    inplane_dir = inplane_dir / np.linalg.norm(inplane_dir)
    p1 = point - inplane_dir * width / 2 - axis_dir * height / 2
    p2 = point + inplane_dir * width / 2 - axis_dir * height / 2
    p3 = point + inplane_dir * width / 2 + axis_dir * height / 2
    p4 = point - inplane_dir * width / 2 + axis_dir * height / 2
    plane = Poly3DCollection(
        [[p1, p2, p3, p4]],
        facecolors="#cfe0f4",
        edgecolors="#7fa7cf",
        linewidths=0.9,
        alpha=0.18,
    )
    ax.add_collection3d(plane)
    txt = point + inplane_dir * 0.04 + axis_dir * 0.07
    ax.text(txt[0], txt[1], txt[2], "Symmetry plane", fontsize=9, color="#537da9")


def set_equal_axes(ax, pts):
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    center = (mins + maxs) / 2
    radius = (maxs - mins).max() / 2 * 1.18
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius * 0.75, center[2] + radius * 0.75)
    ax.set_box_aspect([1.0, 1.0, 0.56])


def collect_body_recursive(body, parent_tf, assets, meshes, joints, frames):
    local_tf = make_transform(body_pos(body), body_euler(body))
    world_tf = parent_tf @ local_tf
    body_name = body.attrib["name"]
    frames[body_name] = world_tf

    for geom in body.findall("geom"):
        if geom.attrib.get("type") == "mesh":
            mesh_name = geom.attrib["mesh"]
            mesh_file = assets[mesh_name]
            mesh = trimesh.load_mesh(MESH_DIR / mesh_file, force="mesh")
            mesh = mesh.copy()
            mesh.apply_transform(world_tf)
            meshes.append((body_name, mesh))

    for joint in body.findall("joint"):
        origin_local = parse_vec(joint.attrib.get("pos", "0 0 0"), [0.0, 0.0, 0.0])
        axis_local = parse_vec(joint.attrib.get("axis", "0 0 1"), [0.0, 0.0, 1.0])
        origin_world = (world_tf @ np.r_[origin_local, 1.0])[:3]
        axis_world = world_tf[:3, :3] @ axis_local
        axis_world = axis_world / np.linalg.norm(axis_world)
        joints.append(
            {
                "xml_name": joint.attrib["name"],
                "paper_name": PAPER_MAP.get(joint.attrib["name"], joint.attrib["name"]),
                "origin": origin_world,
                "axis": axis_world,
                "rot": world_tf[:3, :3],
                "body": body_name,
            }
        )

    for child in body.findall("body"):
        collect_body_recursive(child, world_tf, assets, meshes, joints, frames)


def get_joint(joints, name):
    for joint in joints:
        if joint["xml_name"] == name:
            return joint
    raise KeyError(name)


def derive_symmetry_plane_direction(joints):
    j5 = get_joint(joints, "joint5")
    j3 = get_joint(joints, "joint3")
    j7 = get_joint(joints, "joint7")
    axis = j5["axis"]
    target = 0.5 * (j3["origin"] + j7["origin"])
    vec = target - j5["origin"]
    vec = vec - axis * np.dot(vec, axis)
    if np.linalg.norm(vec) < 1e-9:
        vec = np.array([1.0, 0.0, 0.0])
        vec = vec - axis * np.dot(vec, axis)
    return vec / np.linalg.norm(vec)


def render_figure(meshes, joints, out_path, with_local_frames=True, with_pair_text=True):
    fig = plt.figure(figsize=(11.2, 8.0), dpi=220)
    ax = fig.add_subplot(111, projection="3d", proj_type="ortho")
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    for _, mesh in meshes:
        add_mesh(ax, mesh, facecolor="#d5d9de", edgecolor="none", alpha=1.0)

    passive_disc = "#6f7f95"
    planar_disc = "#2e5ac9"
    spatial_disc = "#b53a49"

    for joint in joints:
        paper_name = joint["paper_name"]
        if paper_name in ACTIVE_PLANAR:
            disc_color = planar_disc
        elif paper_name in ACTIVE_SPATIAL:
            disc_color = spatial_disc
        else:
            disc_color = passive_disc
        disc = make_disc_mesh(joint["origin"], joint["axis"])
        add_mesh(ax, disc, facecolor=disc_color, edgecolor="none", alpha=0.97)
        draw_axis_line(ax, joint["origin"], joint["axis"])

    j5 = get_joint(joints, "joint5")
    plane_dir = derive_symmetry_plane_direction(joints)
    draw_symmetry_plane(ax, j5["origin"], j5["axis"], plane_dir)

    all_pts = np.vstack([mesh.vertices for _, mesh in meshes])
    mins = all_pts.min(axis=0)
    maxs = all_pts.max(axis=0)
    global_origin = np.array([mins[0] - 0.04, mins[1] - 0.03, mins[2] - 0.005])
    draw_frame(ax, global_origin, np.eye(3), scale=0.038, labels=("X", "Y", "Z"), linewidth=1.1)
    ax.text(global_origin[0] - 0.006, global_origin[1] - 0.006, global_origin[2] - 0.004, "O", fontsize=9, color="black")

    if with_local_frames:
        for xml_name, offset in [
            ("joint1", np.array([0.012, 0.000, 0.000])),
            ("joint3", np.array([0.014, -0.004, 0.002])),
            ("joint5", np.array([0.010, 0.000, 0.004])),
        ]:
            joint = get_joint(joints, xml_name)
            draw_frame(ax, joint["origin"] + offset, joint["rot"], scale=0.021, labels=("x", "y", "z"), linewidth=0.9)

    label_offsets = {
        "joint1": np.array([0.012, 0.010, 0.018]),
        "joint2": np.array([0.010, -0.010, 0.018]),
        "joint3": np.array([0.008, -0.014, 0.018]),
        "joint4": np.array([-0.022, -0.006, 0.016]),
        "joint5": np.array([-0.018, 0.008, 0.016]),
        "joint6": np.array([0.004, 0.016, 0.016]),
        "joint7": np.array([0.010, 0.012, 0.018]),
    }
    for joint in joints:
        pt = joint["origin"] + label_offsets[joint["xml_name"]]
        ax.text(pt[0], pt[1], pt[2], joint["paper_name"], fontsize=11, color="black", fontweight="bold")

    if with_pair_text:
        ax.text(-0.02, -0.135, 0.050, "Planar mode active pair: J1, J2", fontsize=9, color="#2446a4")
        ax.text(0.060, 0.055, 0.046, "Spatial mode active pair: J3, J7", fontsize=9, color="#8f2332")

    set_equal_axes(ax, all_pts)
    ax.view_init(elev=18, azim=-57)
    ax.set_axis_off()
    plt.tight_layout(pad=0.1)
    fig.savefig(out_path, dpi=320, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def main():
    root = ET.parse(XML_PATH).getroot()
    assets = parse_assets(root)
    worldbody = root.find("worldbody")
    base = worldbody.find("./body[@name='link0']")

    meshes = []
    joints = []
    frames = {}
    collect_body_recursive(base, np.eye(4), assets, meshes, joints, frames)

    annotated_tmp = TMP_OUT_DIR / "figure1_mujoco_academic_annotated.png"
    clean_tmp = TMP_OUT_DIR / "figure1_mujoco_academic_clean.png"
    render_figure(meshes, joints, annotated_tmp, with_local_frames=True, with_pair_text=True)
    render_figure(meshes, joints, clean_tmp, with_local_frames=False, with_pair_text=False)

    final_paths = []
    for tmp_path in [annotated_tmp, clean_tmp]:
        final_path = FINAL_OUT_DIR / tmp_path.name
        shutil.copy2(tmp_path, final_path)
        final_paths.append(final_path)

    for p in final_paths:
        print(str(p))


if __name__ == "__main__":
    main()
