"""ROS-independent helpers for the Experiment localization projection."""

from __future__ import print_function

import math
import re

ABSOLUTE_ROS_NAMESPACE_RE = re.compile(r"^(/[A-Za-z][A-Za-z0-9_]*)+$")
MOCAP_RIGID_BODY_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
EXPLICIT_TOPIC_PARAMS = (
    "pose_in",
    "twist_in",
    "accel_in",
    "pose_out",
    "twist_out",
    "accel_out",
    "vision_out",
)
TRUE_BOOL_TEXT = ("true", "yes", "on", "1")
FALSE_BOOL_TEXT = ("false", "no", "off", "0")


def finite_values(*values):
    return all(
        isinstance(value, (int, float))
        and not math.isnan(value)
        and not math.isinf(value)
        for value in values
    )


def translated_position(x, y, z, offset_x, offset_y, offset_z):
    values = (x, y, z, offset_x, offset_y, offset_z)
    if not finite_values(*values):
        raise ValueError("position and localization offset must be finite")
    return x + offset_x, y + offset_y, z + offset_z


def valid_pose(x, y, z, qx, qy, qz, qw):
    if not finite_values(x, y, z, qx, qy, qz, qw):
        return False
    return qx * qx + qy * qy + qz * qz + qw * qw >= 1.0e-12


def parse_bool(value, name):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if value == 1:
            return True
        if value == 0:
            return False
    text = str(value).strip().lower()
    if text in TRUE_BOOL_TEXT:
        return True
    if text in FALSE_BOOL_TEXT:
        return False
    raise ValueError("{} must be a bool".format(name))


def absolute_ros_namespace(value, name):
    text = str(value).strip()
    if not ABSOLUTE_ROS_NAMESPACE_RE.match(text):
        raise ValueError(
            "{} must be one absolute ROS namespace such as /vrpn_client_node or /uav1".format(
                name
            )
        )
    return text


def legal_mocap_rigid_body(value):
    text = str(value).strip()
    if not MOCAP_RIGID_BODY_RE.match(text):
        raise ValueError(
            "mocap_rigid_body must be one legal tracker name matching ^[A-Za-z][A-Za-z0-9_]*$"
        )
    return text


def derive_experiment_topics(
    source_root, mocap_rigid_body, robot_namespace, publish_vision
):
    """Map the four domain parameters to the only allowed input/output topics."""
    root = absolute_ros_namespace(source_root, "source_root")
    body = legal_mocap_rigid_body(mocap_rigid_body)
    namespace = absolute_ros_namespace(robot_namespace, "robot_namespace")
    vision = parse_bool(publish_vision, "publish_vision")
    topics = {
        "pose_in": "{}/{}/pose".format(root, body),
        "twist_in": "{}/{}/twist".format(root, body),
        "accel_in": "{}/{}/accel".format(root, body),
        "pose_out": "{}/pose".format(namespace),
        "twist_out": "{}/twist".format(namespace),
        "accel_out": "{}/accel".format(namespace),
        "vision_out": "{}/mavros/vision_pose/pose".format(namespace) if vision else "",
    }
    outputs = [topics["pose_out"], topics["twist_out"], topics["accel_out"]]
    if topics["vision_out"]:
        outputs.append(topics["vision_out"])
    if len(set(outputs)) != len(outputs):
        raise ValueError("Experiment localization outputs must be unique")
    return topics
