#!/usr/bin/env python3
from __future__ import print_function

import os
import sys
import unittest
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

from xgc2_vrpn_relay.projection import (
    EXPLICIT_TOPIC_PARAMS,
    derive_experiment_topics,
    finite_values,
    parse_bool,
    translated_position,
    valid_pose,
)


class ProjectionTests(unittest.TestCase):
    def test_translation_changes_only_position_values(self):
        self.assertEqual(translated_position(1.0, -2.0, 3.0, 0.5, 1.0, -1.5), (1.5, -1.0, 1.5))

    def test_nonfinite_translation_is_rejected(self):
        with self.assertRaises(ValueError):
            translated_position(1.0, 2.0, 3.0, float("nan"), 0.0, 0.0)

    def test_pose_validation_does_not_normalize_orientation(self):
        self.assertTrue(valid_pose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 2.0))
        self.assertFalse(valid_pose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0))
        self.assertFalse(valid_pose(float("inf"), 2.0, 3.0, 0.0, 0.0, 0.0, 1.0))

    def test_finite_values_accepts_ints_and_floats(self):
        self.assertTrue(finite_values(0, 1.5, -2.0))


class ExperimentTopicContractTests(unittest.TestCase):
    def test_px4_slot_derives_vision_under_namespace(self):
        topics = derive_experiment_topics(
            "/vrpn_client_node", "FS150_01", "/uav1", True
        )
        self.assertEqual(topics["pose_in"], "/vrpn_client_node/FS150_01/pose")
        self.assertEqual(topics["twist_in"], "/vrpn_client_node/FS150_01/twist")
        self.assertEqual(topics["accel_in"], "/vrpn_client_node/FS150_01/accel")
        self.assertEqual(topics["pose_out"], "/uav1/pose")
        self.assertEqual(topics["twist_out"], "/uav1/twist")
        self.assertEqual(topics["accel_out"], "/uav1/accel")
        self.assertEqual(topics["vision_out"], "/uav1/mavros/vision_pose/pose")

    def test_ugv_slot_does_not_publish_vision(self):
        topics = derive_experiment_topics(
            "/vrpn_client_node", "Scout1", "/ugv1", False
        )
        self.assertEqual(topics["pose_in"], "/vrpn_client_node/Scout1/pose")
        self.assertEqual(topics["twist_out"], "/ugv1/twist")
        self.assertEqual(topics["accel_out"], "/ugv1/accel")
        self.assertEqual(topics["vision_out"], "")

    def test_simulation_source_root_is_the_client_namespace(self):
        topics = derive_experiment_topics(
            "/vrpn_client_node_simulation", "Wheeltec1", "/ugv2", False
        )
        self.assertEqual(
            topics["pose_in"], "/vrpn_client_node_simulation/Wheeltec1/pose"
        )
        self.assertEqual(topics["accel_in"], "/vrpn_client_node_simulation/Wheeltec1/accel")

    def test_publish_vision_accepts_common_bool_spellings(self):
        on_topics = derive_experiment_topics("/vrpn_client_node", "FS150_01", "/uav1", "true")
        off_topics = derive_experiment_topics("/vrpn_client_node", "FS150_01", "/uav1", "0")
        self.assertTrue(on_topics["vision_out"].endswith("/mavros/vision_pose/pose"))
        self.assertEqual(off_topics["vision_out"], "")
        self.assertTrue(parse_bool(True, "publish_vision"))
        self.assertFalse(parse_bool("false", "publish_vision"))

    def test_hyphenated_tracker_is_rejected(self):
        with self.assertRaises(ValueError):
            derive_experiment_topics("/vrpn_client_node", "rigid-body", "/uav1", False)

    def test_relative_or_trailing_slash_namespaces_are_rejected(self):
        with self.assertRaises(ValueError):
            derive_experiment_topics("vrpn_client_node", "FS150_01", "/uav1", False)
        with self.assertRaises(ValueError):
            derive_experiment_topics("/vrpn_client_node/", "FS150_01", "/uav1", False)
        with self.assertRaises(ValueError):
            derive_experiment_topics("/vrpn_client_node", "FS150_01", "/uav1/", True)
        with self.assertRaises(ValueError):
            derive_experiment_topics("/vrpn//client", "FS150_01", "/uav1", False)

    def test_root_namespace_and_empty_body_are_rejected(self):
        with self.assertRaises(ValueError):
            derive_experiment_topics("/", "FS150_01", "/uav1", False)
        with self.assertRaises(ValueError):
            derive_experiment_topics("/vrpn_client_node", "", "/uav1", False)
        with self.assertRaises(ValueError):
            derive_experiment_topics("/vrpn_client_node", "1body", "/uav1", False)

    def test_invalid_publish_vision_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_bool("", "publish_vision")
        with self.assertRaises(ValueError):
            parse_bool("maybe", "publish_vision")

    def test_launch_exposes_only_domain_args(self):
        path = os.path.join(HERE, "..", "launch", "experiment_projection.launch")
        tree = ET.parse(path)
        args = {
            element.attrib["name"]: element
            for element in tree.getroot().findall("arg")
        }
        self.assertEqual(
            set(args),
            {
                "source_root",
                "mocap_rigid_body",
                "robot_namespace",
                "publish_vision",
                "vision_target_hz",
                "offset_x",
                "offset_y",
                "offset_z",
            },
        )
        for name in EXPLICIT_TOPIC_PARAMS:
            self.assertNotIn(name, args)
        self.assertNotIn("default", args["source_root"].attrib)
        self.assertNotIn("default", args["mocap_rigid_body"].attrib)
        self.assertNotIn("default", args["robot_namespace"].attrib)
        self.assertEqual(args["publish_vision"].attrib.get("default"), "false")
        self.assertEqual(args["vision_target_hz"].attrib.get("default"), "30.0")
        params = {
            element.attrib["name"]: element
            for element in tree.find("node").findall("param")
        }
        self.assertEqual(params["publish_vision"].attrib.get("type"), "bool")
        for name in EXPLICIT_TOPIC_PARAMS:
            self.assertNotIn(name, params)

    def test_node_does_not_read_explicit_topic_params(self):
        path = os.path.join(HERE, "..", "scripts", "experiment_projection")
        with open(path) as handle:
            text = handle.read()
        for name in EXPLICIT_TOPIC_PARAMS:
            self.assertNotIn('get_param("~{}")'.format(name), text)
            self.assertNotIn('get_param("~{}",'.format(name), text)
        self.assertIn("source_root", text)
        self.assertIn("mocap_rigid_body", text)
        self.assertIn("robot_namespace", text)
        self.assertIn("publish_vision", text)
        self.assertIn("derive_experiment_topics", text)


if __name__ == "__main__":
    unittest.main()
