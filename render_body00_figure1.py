import math
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import trimesh
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


SRC_DIR = Path(r"E:\hub\body00_render_source_ascii")
URDF_PATH = SRC_DIR / "body00.SLDASM.urdf"
MESH_DIR = SRC_DIR / "meshes"
OUT_DIR = Path(r"E:\hub\02论文正文\001paper\03_图表与素材\机构图")
OUT_DIR.mkdir(parents=True, exist_ok=True)


def rpy_to_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def make_transform(xyz, rpy):
    tf = np.eye(4)
    tf[:3, :3] = rpy_to_matrix(*rpy)
    tf[:3, 3] = xyz
    return tf


def parse_urdf(urdf_path):
    root = ET.parse(urdf_path).getroot()
    links = {}
    joints = []

    for link in root.findall("link"):
        name = link.attrib["name"]
        mesh_tag = link.find("./visual/geometry/mesh")
        links[name] = {
            "mesh": mesh_tag.attrib["filename"].split("/")[-1] if mesh_tag is not None else None,
        }

    for joint in root.findall("joint"):
        xyz = np.array(list(map(float, joint.find("origin").attrib["xyz"].split())))
        rpy = np.array(list(map(float, joint.find("origin").attrib["rpy"].split())))
        axis = np.array(list(map(float, joint.find("axis").attrib["xyz"].split())))
        joints.append(
            {
                "urdf_name": joint.attrib["name"],
                "parent": joint.find("parent").attrib["link"],
                "child": joint.find("child").attrib["link"],
                "xyz": xyz,
                "rpy": rpy,
                "axis": axis,
            }
        )
    return links, joints


def build_chain(joints):
    by_parent = {}
    for joint in joints:
        by_parent.setdefault(joint["parent"], []).append(joint)

    transforms = {"base_link": np.eye(4)}
    chain = []
    current = "base_link"
    while current in by_parent and by_parent[current]:
        joint = by_parent[current][0]
        transforms[joint["child"]] = transforms[current] @ make_transform(joint["xyz"], joint["rpy"])
        chain.append(joint)
        current = joint["child"]
    return transforms, chain


def load_world_mesh(mesh_path, transform):
    mesh = trimesh.load_mesh(mesh_path, force="mesh")
    mesh = mesh.copy()
    mesh.apply_transform(transform)
    return mesh


def add_mesh(ax, mesh, facecolor, edgecolor="#666666", alpha=1.0, linewidth=0.5):
    tri = mesh.vertices[mesh.faces]
    coll = Poly3DCollection(
        tri,
        facecolors=facecolor,
        edgecolors=edgecolor,
        linewidths=linewidth,
        alpha=alpha,
    )
    ax.add_collection3d(coll)


def add_joint_disc(ax, origin, axis, radius=0.013, thickness=0.002, color="#2f4fbb", alpha=0.95):
    cyl = trimesh.creation.cylinder(radius=radius, height=thickness, sections=40)
    z_axis = np.array([0.0, 0.0, 1.0])
    axis = axis / np.linalg.norm(axis)
    rot = trimesh.geometry.align_vectors(z_axis, axis)
    cyl.apply_transform(rot)
    cyl.apply_translation(origin)
    add_mesh(ax, cyl, facecolor=color, edgecolor=color, alpha=alpha, linewidth=0.2)


def draw_axis_line(ax, origin, axis, length=0.08, color="#1553ff", lw=1.6):
    axis = axis / np.linalg.norm(axis)
    p1 = origin - axis * length / 2
    p2 = origin + axis * length / 2
    ax.plot(
        [p1[0], p2[0]],
        [p1[1], p2[1]],
        [p1[2], p2[2]],
        linestyle=(0, (3, 2)),
        color=color,
        linewidth=lw,
    )


def draw_frame(ax, origin, rot, scale=0.03, labels=("x", "y", "z"), alpha=0.95):
    colors = ["#d62728", "#2ca02c", "#1f77b4"]
    for i, (lab, col) in enumerate(zip(labels, colors)):
        vec = rot[:, i] * scale
        ax.quiver(
            origin[0],
            origin[1],
            origin[2],
            vec[0],
            vec[1],
            vec[2],
            color=col,
            arrow_length_ratio=0.18,
            linewidth=1.1,
            alpha=alpha,
        )
        txt = origin + vec * 1.15
        ax.text(txt[0], txt[1], txt[2], lab, color=col, fontsize=8)


def draw_symmetry_plane(ax, point, axis_dir, inplane_dir, width=0.18, height=0.11):
    axis_dir = axis_dir / np.linalg.norm(axis_dir)
    inplane_dir = inplane_dir / np.linalg.norm(inplane_dir)
    p1 = point - inplane_dir * width / 2 - axis_dir * height / 2
    p2 = point + inplane_dir * width / 2 - axis_dir * height / 2
    p3 = point + inplane_dir * width / 2 + axis_dir * height / 2
    p4 = point - inplane_dir * width / 2 + axis_dir * height / 2
    plane = Poly3DCollection([[p1, p2, p3, p4]], facecolors="#b7d5f0", alpha=0.18, edgecolors="#5b8db8", linewidths=0.8)
    ax.add_collection3d(plane)
    label_pos = point + inplane_dir * 0.02 + axis_dir * 0.055
    ax.text(label_pos[0], label_pos[1], label_pos[2], "Symmetry plane", color="#3e6d93", fontsize=9)


def set_equal_axes(ax, pts):
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    center = (mins + maxs) / 2
    radius = (maxs - mins).max() / 2 * 1.18
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_box_aspect([1, 1, 0.62])


def main():
    links, joints = parse_urdf(URDF_PATH)
    transforms, chain = build_chain(joints)

    paper_joint_names = [f"J{i}" for i in range(1, 8)]
    active_planar = {"J1", "J2"}
    active_spatial = {"J3", "J7"}

    world_meshes = []
    all_points = []
    joint_data = []

    base_mesh = load_world_mesh(MESH_DIR / links["base_link"]["mesh"], np.eye(4))
    world_meshes.append(("base_link", base_mesh))
    all_points.append(base_mesh.vertices)

    for idx, joint in enumerate(chain, start=1):
        child = joint["child"]
        mesh_name = links[child]["mesh"]
        mesh = load_world_mesh(MESH_DIR / mesh_name, transforms[child])
        world_meshes.append((child, mesh))
        all_points.append(mesh.vertices)

        parent_tf = transforms[joint["parent"]]
        joint_tf = parent_tf @ make_transform(joint["xyz"], joint["rpy"])
        origin = (parent_tf @ np.r_[joint["xyz"], 1.0])[:3]
        rot = joint_tf[:3, :3]
        axis_world = rot @ joint["axis"]
        joint_data.append(
            {
                "paper_name": paper_joint_names[idx - 1],
                "origin": origin,
                "axis": axis_world / np.linalg.norm(axis_world),
                "rot": rot,
            }
        )

    pts = np.vstack(all_points)

    fig = plt.figure(figsize=(11.5, 8.2), dpi=220)
    ax = fig.add_subplot(111, projection="3d", proj_type="ortho")
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    for _, mesh in world_meshes:
        add_mesh(ax, mesh, facecolor="#d5d8dc", edgecolor="#666666", alpha=1.0, linewidth=0.35)

    for info in joint_data:
        if info["paper_name"] in active_planar:
            disc_color = "#3257c6"
        elif info["paper_name"] in active_spatial:
            disc_color = "#b23345"
        else:
            disc_color = "#8f9db2"
        add_joint_disc(ax, info["origin"], info["axis"], color=disc_color)
        draw_axis_line(ax, info["origin"], info["axis"])

    # Symmetry plane passes through J5 axis. Use a diagonal in-plane direction to add spatial sense.
    j5 = next(item for item in joint_data if item["paper_name"] == "J5")
    draw_symmetry_plane(
        ax,
        point=j5["origin"],
        axis_dir=j5["axis"],
        inplane_dir=np.array([1.0, -1.0, 0.0]),
    )

    # Global frame near lower-left open area.
    o = np.array([-0.125, -0.03, -0.045])
    global_rot = np.eye(3)
    draw_frame(ax, o, global_rot, scale=0.035, labels=("X", "Y", "Z"), alpha=0.95)
    ax.text(o[0] - 0.005, o[1] - 0.005, o[2] - 0.005, "O", fontsize=9, color="black")

    # Representative DH frames at J1, J3, J5 only.
    for key, offset in [("J1", np.array([0.01, 0.0, 0.0])), ("J3", np.array([0.012, 0.005, 0.0])), ("J5", np.array([0.012, -0.006, 0.0]))]:
        info = next(item for item in joint_data if item["paper_name"] == key)
        draw_frame(ax, info["origin"] + offset, info["rot"], scale=0.022, labels=("x", "y", "z"), alpha=0.88)

    # Joint labels.
    label_offsets = {
        "J1": np.array([0.008, -0.012, 0.03]),
        "J2": np.array([0.012, -0.018, 0.018]),
        "J3": np.array([0.014, 0.012, 0.02]),
        "J4": np.array([-0.014, 0.012, 0.02]),
        "J5": np.array([-0.016, 0.008, 0.02]),
        "J6": np.array([-0.015, -0.02, 0.02]),
        "J7": np.array([0.0, -0.018, 0.02]),
    }
    for info in joint_data:
        p = info["origin"] + label_offsets[info["paper_name"]]
        ax.text(p[0], p[1], p[2], info["paper_name"], fontsize=11, color="black", fontweight="bold")

    # Active pair annotations.
    ax.text(0.045, -0.028, 0.045, "Planar mode active pair", fontsize=9, color="#2446a4")
    ax.text(0.085, 0.115, 0.04, "Spatial mode active pair", fontsize=9, color="#8c2232")

    set_equal_axes(ax, pts)
    ax.view_init(elev=20, azim=-52)
    ax.set_axis_off()
    plt.tight_layout(pad=0.2)

    out = OUT_DIR / "figure1_body00_academic.png"
    fig.savefig(out, dpi=320, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(out)


if __name__ == "__main__":
    main()
