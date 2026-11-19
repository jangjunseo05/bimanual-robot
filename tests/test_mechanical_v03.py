from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
SPEC_PATH = REPO_ROOT / "design/mechanical/hold_flow_mechanical_v0_3.yaml"
CAD_MANIFEST = REPO_ROOT / "design/cad/exports/manifest.json"
CUP_MANIFEST = REPO_ROOT / "design/gripper/exports/cup_overcaps/manifest.json"
IMPORT_SCRIPT = REPO_ROOT / "src/description/hold_flow_description/scripts/import_isaac_sim.py"
URDF_PATH = REPO_ROOT / "src/description/hold_flow_description/urdf/hold_flow.urdf"
DESCRIPTION_VALIDATOR = REPO_ROOT / "src/description/hold_flow_description/scripts/validate_description.py"
QUALITY_AUDITOR = REPO_ROOT / "src/description/hold_flow_description/scripts/audit_urdf_quality.py"
CALIBRATION_PATH = REPO_ROOT / "src/description/hold_flow_description/config/mechanical_calibration.yaml"
SO101_LICENSE = REPO_ROOT / "src/description/hold_flow_description/third_party/so_arm_101/LICENSE"


class MechanicalV03Test(unittest.TestCase):
    def test_so101_generator_uses_embedded_source_and_external_output(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "so101.urdf.xacro"
            subprocess.run(
                [sys.executable, str(DESCRIPTION_VALIDATOR.with_name("generate_so101_arm_xacro.py")),
                 "--output", str(output)],
                cwd=REPO_ROOT, check=True, capture_output=True, text=True,
            )
            expected = URDF_PATH.parents[1] / "third_party/so_arm_101/so101_arm.urdf.xacro"
            self.assertEqual(output.read_bytes(), expected.read_bytes())

    def test_xacro_expands_without_ros_package_index(self) -> None:
        script = """
import sys
import xacro

# ROS가 설치된 개발 PC에서도 CI의 누락된 의존성을 재현한다.
sys.modules['ament_index_python'] = None
sys.modules['ament_index_python.packages'] = None
print(xacro.process_file(sys.argv[1]).toxml())
"""
        with tempfile.TemporaryDirectory() as directory:
            env = os.environ.copy()
            ros_python = Path("/opt/ros/jazzy/lib/python3.12/site-packages")
            if ros_python.is_dir():
                existing = env.get("PYTHONPATH", "")
                env["PYTHONPATH"] = str(ros_python) + (":" + existing if existing else "")
            result = subprocess.run(
                [sys.executable, "-c", script, str(URDF_PATH.with_suffix(".urdf.xacro"))],
                cwd=directory, check=True, capture_output=True, text=True, env=env,
            )
        self.assertEqual(
            ET.canonicalize(result.stdout, strip_text=True),
            ET.canonicalize(URDF_PATH.read_text(encoding="utf-8"), strip_text=True),
        )

    def test_design_spec_has_requested_geometry(self) -> None:
        spec = yaml.safe_load(SPEC_PATH.read_text(encoding="utf-8"))
        self.assertEqual(spec["chassis"]["tabletop_footprint"], [340.0, 450.0])
        self.assertEqual(len(spec["chassis"]["plates"]["tabletop"]["quadrant_names"]), 4)
        self.assertEqual(spec["chassis"]["profile_rings"]["y_rail_center_x"], 160.0)
        self.assertEqual(spec["chassis"]["plates"]["tabletop"]["z_top"], 720.0)
        self.assertEqual(spec["arm_mounts"]["left"]["xyz"], [20.0, 170.0, 726.0])
        self.assertEqual(spec["arm_mounts"]["right"]["xyz"], [20.0, -170.0, 726.0])
        self.assertEqual(
            spec["arm_mounts"]["left"]["gripper"],
            "stock_so101_rotating_jaw_with_tpu_moving_jaw",
        )
        self.assertIsNone(spec["left_cup_gripper"]["selected_attachment"])
        self.assertFalse(
            spec["provenance"]["left_finray_fingers_historical"]["current_build_use"]
        )
        self.assertEqual(spec["camera"]["height_above_tabletop"], 540.0)
        self.assertEqual(spec["camera"]["optical_center_xyz"], [-110.0, 0.0, 1260.0])
        self.assertEqual(spec["camera"]["mount"]["mast_visual_proxy_length"], 530.0)
        self.assertEqual(
            spec["camera"]["mount"]["mast_length_status"],
            "legacy_cad_candidate_not_current_physical_mast",
        )
        self.assertEqual(
            spec["camera"]["optical_center_xyz"][2]
            - spec["chassis"]["plates"]["tabletop"]["z_top"],
            spec["camera"]["height_above_tabletop"],
        )
        self.assertEqual(spec["camera"]["mount"]["mast_z_bottom"], 728.0)
        self.assertEqual(
            spec["camera"]["mount"]["mast_z_bottom"],
            spec["chassis"]["plates"]["tabletop"]["z_top"]
            + spec["camera"]["mount"]["backing_thickness"],
        )
        self.assertNotIn("ball_casters", spec["navigation"])
        self.assertEqual(spec["navigation"]["ball_caster_count"], 0)
        self.assertEqual(spec["navigation"]["drive_type"], "skid_steer_four_wheel_drive")
        self.assertEqual(
            spec["navigation"]["wheel"]["center_xyz_left"], [105.0, 255.0, 32.9]
        )
        self.assertEqual(
            spec["navigation"]["wheel"]["center_xyz_right"], [105.0, -255.0, 32.9]
        )
        self.assertEqual(
            spec["navigation"]["wheel"]["center_xyz_rear_left"], [-105.0, 255.0, 32.9]
        )
        self.assertEqual(
            spec["navigation"]["wheel"]["center_xyz_rear_right"], [-105.0, -255.0, 32.9]
        )
        self.assertEqual(spec["navigation"]["wheel"]["wheelbase_design"], 210.0)
        self.assertEqual(spec["navigation"]["wheel"]["visual_width"], 30.0)
        self.assertEqual(
            spec["navigation"]["joint_names_by_corner"],
            {
                "front_left": "left_wheel_joint",
                "front_right": "right_wheel_joint",
                "rear_left": "rear_left_wheel_joint",
                "rear_right": "rear_right_wheel_joint",
            },
        )
        self.assertEqual(
            spec["navigation"]["servo_ids_by_corner"],
            {"front_left": 2, "front_right": 1, "rear_left": 3, "rear_right": 4},
        )
        self.assertEqual(
            spec["navigation"]["wheel"]["initial_host_parameters"]["wheel_separation_m"],
            0.510,
        )
        self.assertEqual(
            spec["navigation"]["support_polygon_xy"],
            [[105.0, 255.0], [-105.0, 255.0], [-105.0, -255.0], [105.0, -255.0]],
        )
        self.assertEqual(spec["compute_plan"]["later_raspberry_pi_5"]["priority"], "deferred")
        self.assertEqual(
            spec["provenance"]["so101"]["checked_commit"],
            "eecbe3e0a9ebb23e25ad7b2759b03884c6660903",
        )

    def test_calculation_and_isaac_static_contract_pass(self) -> None:
        commands = (
            ["python3", "design/mechanical/calculate_full_size_tabletop_design.py"],
            ["python3", str(DESCRIPTION_VALIDATOR.relative_to(REPO_ROOT))],
            ["python3", str(QUALITY_AUDITOR.relative_to(REPO_ROOT)), "--strict"],
            ["python3", "src/description/hold_flow_description/scripts/validate_isaac_contract.py"],
            ["python3", "src/description/hold_flow_description/scripts/import_isaac_sim.py", "--check-only"],
        )
        for command in commands:
            with self.subTest(command=command):
                subprocess.run(command, cwd=REPO_ROOT, check=True, capture_output=True, text=True)

    def test_generated_cad_and_cup_holder_manifests(self) -> None:
        cad = json.loads(CAD_MANIFEST.read_text(encoding="utf-8"))
        cup = json.loads(CUP_MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(cad["source_spec"], "design/mechanical/hold_flow_mechanical_v0_3.yaml")
        self.assertTrue(cad["all_breps_valid"])
        parts = {part["name"]: part for part in cad["parts"]}
        panel_names = {
            "tabletop_front_left", "tabletop_front_right",
            "tabletop_rear_left", "tabletop_rear_right",
        }
        self.assertTrue(panel_names <= parts.keys())
        self.assertTrue(all(parts[name]["fdm_part"] for name in panel_names))
        self.assertTrue(all(parts[name]["k1_max_safe_fit"] for name in panel_names))
        self.assertEqual(parts["frame_column"]["quantity"], 4)
        self.assertTrue(parts["battery_mount_plate"]["fdm_part"])
        self.assertTrue(parts["lidar_mount_plate"]["fdm_part"])
        placements = cad["assembly_placements_mm"]
        self.assertEqual(placements["camera_backing"], [-120.0, 0.0, 720.0])
        self.assertEqual(placements["camera_mast_segment"], [-120.0, 0.0, 728.0])
        self.assertEqual(
            placements["arm_adapters"],
            [[20.0, 170.0, 720.0], [20.0, -170.0, 720.0]],
        )
        self.assertEqual(len(placements["frame_columns"]), 4)
        self.assertEqual(len(placements["frame_rail_x"]), 5)
        self.assertEqual(len(placements["frame_rail_y"]), 5)
        self.assertTrue(cup["valid_brep"])
        self.assertEqual(cup["quantity"], 2)
        self.assertTrue(cup["support_expected"])
        self.assertTrue((REPO_ROOT / cup["stl"]).is_file())

    def test_urdf_mass_matches_provisional_mass_model(self) -> None:
        calculation = subprocess.run(
            ["python3", "design/mechanical/calculate_full_size_tabletop_design.py"],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        mass_model = json.loads(calculation.stdout)["provisional_mass_model"]
        expected_total = mass_model["exact_total_kg"]
        root = ET.parse(URDF_PATH).getroot()
        actual_total = sum(
            float(mass.attrib["value"])
            for mass in root.findall("link/inertial/mass")
        )
        self.assertAlmostEqual(actual_total, expected_total, places=6)
        self.assertAlmostEqual(sum(mass_model["groups_kg"].values()), mass_model["total_kg"], places=3)
        self.assertEqual(mass_model["task_mass_cases_kg"]["empty_robot"], 7.479)

    def test_isaac_contract_uses_four_velocity_joints_without_contact_proxies(self) -> None:
        config = yaml.safe_load(
            (REPO_ROOT / "src/description/hold_flow_description/config/isaac_sim_6.yaml").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(
            config["articulation_contract"]["wheel_velocity_joints"],
            [
                "left_wheel_joint",
                "right_wheel_joint",
                "rear_left_wheel_joint",
                "rear_right_wheel_joint",
            ],
        )
        self.assertEqual(config["articulation_contract"]["fixed_contact_proxies"], [])

    def test_wheel_limit_contract_rejects_positive_config_or_urdf_drift(self) -> None:
        module_spec = importlib.util.spec_from_file_location(
            "hold_flow_description_validator", DESCRIPTION_VALIDATOR
        )
        validator = importlib.util.module_from_spec(module_spec)
        assert module_spec.loader is not None
        module_spec.loader.exec_module(validator)
        mechanical = yaml.safe_load(SPEC_PATH.read_text(encoding="utf-8"))
        calibration = yaml.safe_load(CALIBRATION_PATH.read_text(encoding="utf-8"))
        root = ET.parse(URDF_PATH).getroot()
        joint_names = list(mechanical["navigation"]["joint_names_by_corner"].values())
        validator.assert_drive_limit_contract(
            root, mechanical["navigation"]["wheel"], calibration["drive"], joint_names
        )

        drifted = copy.deepcopy(calibration["drive"])
        drifted["joint_velocity_radps"] += 0.01
        with self.assertRaises(AssertionError):
            validator.assert_drive_limit_contract(
                root, mechanical["navigation"]["wheel"], drifted, joint_names
            )

        drifted_root = ET.fromstring(ET.tostring(root))
        rear_limit = drifted_root.find("joint[@name='rear_right_wheel_joint']/limit")
        assert rear_limit is not None
        rear_limit.attrib["effort"] = str(float(rear_limit.attrib["effort"]) + 0.001)
        with self.assertRaises(AssertionError):
            validator.assert_drive_limit_contract(
                drifted_root,
                mechanical["navigation"]["wheel"],
                calibration["drive"],
                joint_names,
            )

    def test_isaac_yaml_is_the_importer_source(self) -> None:
        spec = importlib.util.spec_from_file_location("hold_flow_isaac_import", IMPORT_SCRIPT)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        settings, robot_type, source_urdf, output_usd = module.load_import_settings(
            REPO_ROOT / "src/description/hold_flow_description/config/isaac_sim_6.yaml"
        )
        self.assertEqual(robot_type, "Mobile Manipulators")
        self.assertEqual(source_urdf, URDF_PATH)
        self.assertEqual(output_usd, REPO_ROOT / "build/isaac/hold_flow/hold_flow.usd")
        self.assertEqual(
            settings,
            {
                "collision_from_visuals": False,
                "merge_mesh": False,
                "merge_fixed_joints": False,
                "allow_self_collision": False,
                "fix_base": False,
            },
        )

        class FakeImporterConfig:
            def __init__(self, **kwargs) -> None:
                self.kwargs = kwargs

        configured = module.create_import_config(
            FakeImporterConfig,
            source_urdf,
            output_usd.parent,
            settings,
            robot_type,
        )
        self.assertEqual(
            configured.kwargs,
            {
                "urdf_path": str(source_urdf),
                "usd_path": str(output_usd.parent),
                **settings,
                "robot_type": "Mobile Manipulators",
            },
        )

    def test_isaac_materialized_urdf_has_no_package_uri(self) -> None:
        spec = importlib.util.spec_from_file_location("hold_flow_isaac_import", IMPORT_SCRIPT)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory(prefix="hold_flow_isaac_test_") as directory:
            output = module.materialize_isaac_urdf(URDF_PATH, Path(directory))
            text = output.read_text(encoding="utf-8")
        self.assertNotIn("package://hold_flow_description/", text)
        self.assertIn(str(REPO_ROOT / "src/description/hold_flow_description/meshes"), text)

    def test_so101_upstream_license_copy_is_pinned(self) -> None:
        self.assertEqual(
            hashlib.sha256(SO101_LICENSE.read_bytes()).hexdigest(),
            "c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4",
        )


if __name__ == "__main__":
    unittest.main()
