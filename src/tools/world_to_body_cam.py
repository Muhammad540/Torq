#!/usr/bin/env python3
"""
Transform a world-frame camera pose into a MuJoCo body-local frame.

Given a MuJoCo XML model, a desired camera pose in world coordinates
(position + xyaxes), and a target body name, this tool computes forward
kinematics at a specified joint configuration (default: home / all-zeros),
then inverse-transforms the camera pose into the body's local frame.

Output is a ready-to-paste MuJoCo <camera> element.

Examples
--------
  # Minimal: model, body, world-frame camera spec
  python world_to_body_cam.py \\
      --model workspace/models/SO101/so101_new_calib.xml \\
      --body gripper \\
      --cam-pos 0.1865 -0.0002 0.3400 \\
      --cam-xyaxes -0.0328 -0.9995 0.0 0.2130 -0.0070 0.9770

  # With custom joint angles (radians)
  python world_to_body_cam.py \\
      --model workspace/models/SO101/so101_new_calib.xml \\
      --body gripper \\
      --cam-pos 0.1865 -0.0002 0.3400 \\
      --cam-xyaxes -0.0328 -0.9995 0.0 0.2130 -0.0070 0.9770 \\
      --joints shoulder_pan=0.5 elbow_flex=-0.3

  # Override camera name and fovy
  python world_to_body_cam.py \\
      --model workspace/models/SO101/so101_new_calib.xml \\
      --body gripper \\
      --cam-pos 0.1865 -0.0002 0.3400 \\
      --cam-xyaxes -0.0328 -0.9995 0.0 0.2130 -0.0070 0.9770 \\
      --cam-name wrist_cam --fovy 80
"""
from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np


# ── Rotation helpers ────────────────────────────────────────────────────────

def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """MuJoCo quaternion [w, x, y, z] → 3×3 rotation matrix."""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def axis_angle_to_rot(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues: rotation of `angle` radians about unit `axis`."""
    axis = axis / np.linalg.norm(axis)
    K = np.array([
        [       0, -axis[2],  axis[1]],
        [ axis[2],        0, -axis[0]],
        [-axis[1],  axis[0],        0],
    ])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


def make_tf(pos: np.ndarray, rot: np.ndarray) -> np.ndarray:
    """Build a 4×4 homogeneous transform from position + 3×3 rotation."""
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = pos
    return T


# ── XML kinematic-chain parser ──────────────────────────────────────────────

@dataclass
class BodyNode:
    name: str
    pos: np.ndarray
    quat: np.ndarray
    joint_name: str | None = None
    joint_axis: np.ndarray | None = None
    children: list["BodyNode"] = field(default_factory=list)


def _parse_floats(s: str) -> np.ndarray:
    return np.array([float(x) for x in s.split()])


def _parse_body(elem: ET.Element) -> BodyNode:
    """Recursively parse a <body> element into a BodyNode tree."""
    name = elem.get("name", "unnamed")
    pos = _parse_floats(elem.get("pos", "0 0 0"))
    quat = _parse_floats(elem.get("quat", "1 0 0 0"))

    joint_name = None
    joint_axis = None
    joint_elem = elem.find("joint")
    if joint_elem is not None and joint_elem.get("type", "hinge") == "hinge":
        joint_name = joint_elem.get("name")
        joint_axis = _parse_floats(joint_elem.get("axis", "0 0 1"))

    node = BodyNode(name=name, pos=pos, quat=quat,
                    joint_name=joint_name, joint_axis=joint_axis)

    for child in elem.findall("body"):
        node.children.append(_parse_body(child))

    return node


def parse_model(xml_path: str) -> list[BodyNode]:
    """Parse a MuJoCo XML and return the top-level body nodes under worldbody."""
    tree = ET.parse(xml_path)
    worldbody = tree.getroot().find("worldbody")
    if worldbody is None:
        raise ValueError(f"No <worldbody> in {xml_path}")
    return [_parse_body(b) for b in worldbody.findall("body")]


def find_path(roots: list[BodyNode], target: str) -> list[BodyNode] | None:
    """DFS to find the chain from a root body to the named target body."""
    for root in roots:
        path = _dfs(root, target)
        if path is not None:
            return path
    return None


def _dfs(node: BodyNode, target: str) -> list[BodyNode] | None:
    if node.name == target:
        return [node]
    for child in node.children:
        path = _dfs(child, target)
        if path is not None:
            return [node] + path
    return None


# ── Forward kinematics ──────────────────────────────────────────────────────

def forward_kinematics(
    chain: list[BodyNode],
    joint_angles: dict[str, float],
) -> np.ndarray:
    """
    Compose transforms along `chain` with the given joint angles.

    Returns the 4×4 world-frame transform of the last body in the chain.
    """
    T = np.eye(4)
    for node in chain:
        R_body = quat_to_rot(node.quat)
        T_body = make_tf(node.pos, R_body)
        T = T @ T_body

        if node.joint_name is not None and node.joint_axis is not None:
            q = joint_angles.get(node.joint_name, 0.0)
            if q != 0.0:
                R_joint = axis_angle_to_rot(node.joint_axis, q)
                T = T @ make_tf(np.zeros(3), R_joint)
    return T


# ── Inverse transform ──────────────────────────────────────────────────────

def world_to_local(
    T_body_world: np.ndarray,
    cam_pos_world: np.ndarray,
    cam_x_world: np.ndarray,
    cam_y_world: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Transform camera pos + xyaxes from world frame into body-local frame."""
    R = T_body_world[:3, :3]
    p = T_body_world[:3, 3]
    pos_local = R.T @ (cam_pos_world - p)
    x_local = R.T @ cam_x_world
    y_local = R.T @ cam_y_world
    return pos_local, x_local, y_local


# ── CLI ─────────────────────────────────────────────────────────────────────

def parse_joint_arg(s: str) -> tuple[str, float]:
    """Parse 'joint_name=value' into (name, radians)."""
    if "=" not in s:
        raise argparse.ArgumentTypeError(
            f"Joint spec must be name=value, got '{s}'")
    name, val = s.split("=", 1)
    return name.strip(), float(val.strip())


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert a world-frame camera pose to a MuJoCo body-local "
                    "<camera> element.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--model", required=True,
                        help="Path to MuJoCo XML model file")
    parser.add_argument("--body", required=True,
                        help="Target body name (camera parent)")
    parser.add_argument("--cam-pos", nargs=3, type=float, required=True,
                        metavar=("X", "Y", "Z"),
                        help="Camera position in world frame")
    parser.add_argument("--cam-xyaxes", nargs=6, type=float, required=True,
                        metavar=("X1", "X2", "X3", "Y1", "Y2", "Y3"),
                        help="Camera xyaxes in world frame (x-axis 3 + y-axis 3)")
    parser.add_argument("--joints", nargs="*", type=parse_joint_arg,
                        default=[],
                        help="Joint angles as name=radians pairs "
                             "(default: all zeros / home pose)")
    parser.add_argument("--cam-name", default=None,
                        help="Camera name for the output element")
    parser.add_argument("--fovy", type=float, default=None,
                        help="Field of view (degrees)")

    args = parser.parse_args()

    roots = parse_model(args.model)
    chain = find_path(roots, args.body)
    if chain is None:
        print(f"Error: body '{args.body}' not found in {args.model}",
              file=sys.stderr)
        sys.exit(1)

    joint_angles = dict(args.joints)

    print(f"Kinematic chain: {' → '.join(n.name for n in chain)}")
    print(f"Joint config:    {joint_angles if joint_angles else '(home / all zeros)'}")

    T = forward_kinematics(chain, joint_angles)
    print(f"\n{args.body} world position:  "
          f"[{T[0,3]:.6f}, {T[1,3]:.6f}, {T[2,3]:.6f}]")
    print(f"{args.body} world rotation:\n{T[:3,:3]}")

    cam_pos = np.array(args.cam_pos)
    cam_x = np.array(args.cam_xyaxes[:3])
    cam_y = np.array(args.cam_xyaxes[3:])

    pos_local, x_local, y_local = world_to_local(T, cam_pos, cam_x, cam_y)

    name_attr = f' name="{args.cam_name}"' if args.cam_name else ""
    fovy_attr = f' fovy="{args.fovy:.0f}"' if args.fovy is not None else ""

    xml = (
        f'<camera{name_attr}'
        f' pos="{pos_local[0]:.4f} {pos_local[1]:.4f} {pos_local[2]:.4f}"'
        f' xyaxes="{x_local[0]:.4f} {x_local[1]:.4f} {x_local[2]:.4f}'
        f' {y_local[0]:.4f} {y_local[1]:.4f} {y_local[2]:.4f}"'
        f'{fovy_attr}/>'
    )

    print(f"\n{'─' * 60}")
    print(f"Body-local camera spec:")
    print(f"  pos    = [{pos_local[0]:.6f}, {pos_local[1]:.6f}, {pos_local[2]:.6f}]")
    print(f"  x-axis = [{x_local[0]:.6f}, {x_local[1]:.6f}, {x_local[2]:.6f}]")
    print(f"  y-axis = [{y_local[0]:.6f}, {y_local[1]:.6f}, {y_local[2]:.6f}]")
    print(f"\nReady-to-paste XML:")
    print(f"  {xml}")


if __name__ == "__main__":
    main()
