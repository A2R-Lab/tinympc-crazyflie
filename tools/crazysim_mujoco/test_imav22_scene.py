#!/usr/bin/env python3
import hashlib
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parent
SCENE = ROOT / "scenes" / "vision_imav22.xml"
MANIFEST = ROOT / "scenes" / "imav22_environment.json"
TEXTURES = ROOT / "scenes" / "textures" / "imav22"
RUNNER = ROOT / "run.sh"
SIMULATOR_PATCH = ROOT / "patches" / "crazysim-imav22-complexity.patch"
CAMERA_PATCH = ROOT / "patches" / "crazysim-drone-models-hm01b0-camera.patch"


class Imav22SceneTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = ET.parse(SCENE).getroot()
        cls.world = cls.root.find("worldbody")
        cls.assets = cls.root.find("asset")
        cls.manifest = json.loads(MANIFEST.read_text())

    def geom(self, name):
        return self.world.find(f".//geom[@name='{name}']")

    def body(self, name):
        return self.world.find(f"body[@name='{name}']")

    def test_isaac_fidelity_v3_arena_and_open_venue(self):
        self.assertEqual(self.manifest["arena_abi"], "imav22_nanocopter_visual_fidelity_v3")
        self.assertEqual(self.geom("imav_scoring_floor").get("size"), "4 4 0.035")
        self.assertEqual(self.geom("imav_floor_north").get("size"), "5 0.5 0.035")
        self.assertEqual(self.geom("imav_floor_east").get("size"), "0.5 4 0.035")
        self.assertEqual(self.manifest["arena_size_m"], [10.0, 10.0])
        self.assertEqual(self.manifest["scored_area_size_m"], [8.0, 8.0])
        self.assertIsNone(self.geom("imav_room_ceiling"))
        self.assertFalse(self.manifest["venue"]["ceiling"])
        self.assertEqual(self.assets.find("texture[@name='imav_neutral_dome']").get("type"),
                         "skybox")
        for name in ("imav_traffic_rug", "imav_venue_north_net", "imav_venue_east_net",
                     "imav_venue_post_0", "imav_venue_cabinet"):
            self.assertIsNotNone(self.geom(name), name)

    def test_exact_gate_collision_and_official_visual_mesh(self):
        gate = self.manifest["gate_specification"]
        self.assertEqual(gate["clear_opening_m"], [0.4, 0.4])
        self.assertEqual(gate["border_width_m"], 0.094)
        self.assertEqual(gate["collision_depth_m"], 0.08)
        self.assertIsNotNone(self.assets.find("mesh[@name='imav_official_gate_mesh']"))
        for index in (1, 2):
            body = self.body(f"imav_gate_{index}")
            self.assertEqual(float(body.get("pos").split()[2]), 1.0)
            left = body.find(f"geom[@name='imav_gate_{index}_left']")
            self.assertEqual(left.get("pos"), "0 -0.247 0")
            self.assertEqual(left.get("size"), "0.04 0.047 0.20")
            visual = body.find(f"geom[@name='imav_gate_{index}_visual']")
            self.assertEqual(visual.get("mesh"), "imav_official_gate_mesh")
            self.assertEqual(visual.get("contype"), "0")

    def test_exact_seed22_layout(self):
        expected = {
            "imav_dynamic_panel_1": (3.0974954076, -2.4311083311, 1.1),
            "imav_dynamic_panel_2": (-2.5647976527, 1.0236075862, 1.1),
            "imav_dynamic_flag_1": (0.7039503286, 3.0073297262, 0.0),
            "imav_dynamic_flag_2": (2.9499902138, 1.4484802663, 0.0),
            "imav_gate_1": (-1.3577944002, -2.2223674651, 1.0),
            "imav_gate_2": (1.9647847572, 3.3076693734, 1.0),
            "imav_dynamic_pole_1": (1.4799543987, 0.8379416674, 1.1),
            "imav_dynamic_pole_2": (3.4071090042, -1.3989561011, 1.1),
            "imav_dynamic_pole_3": (-2.6690135103, 2.4261856523, 1.1),
            "imav_dynamic_pole_4": (-2.3496883601, -3.3635266966, 1.1),
        }
        for name, pose in expected.items():
            actual = tuple(map(float, self.body(name).get("pos").split()))
            self.assertEqual(actual, pose, name)
        self.assertEqual(self.manifest["layout"]["seed"], 22)

    def test_obstacle_dimensions_and_inventory(self):
        self.assertEqual(self.geom("imav_orange_pole_1").get("size"), "0.15 1.1")
        self.assertEqual(self.geom("imav_black_panel_1").get("size"), "0.015 0.50 0.90")
        self.assertEqual(self.geom("imav_flag_1_collision").get("size"), "0.0225 0.40 0.775")
        self.assertEqual(self.geom("imav_flag_1_mast").get("size"), "0.018 0.95")
        self.assertEqual(sum(self.manifest["object_inventory"].values()), 10)
        for index in range(1, 5):
            self.assertIsNotNone(self.geom(f"imav_orange_pole_{index}"))
        for index in range(1, 3):
            self.assertIsNotNone(self.geom(f"imav_black_panel_{index}"))
            self.assertIsNotNone(self.geom(f"imav_flag_{index}_collision"))

    def test_official_complexity_modes_and_one_at_a_time_relocation(self):
        modes = self.manifest["complexity_modes"]
        self.assertEqual({name: spec["environment_factor"] for name, spec in modes.items()},
                         {"gates": 1, "static": 5, "dynamic": 10})
        dynamic = self.world.findall("body")
        dynamic = [body for body in dynamic if body.get("name", "").startswith("imav_dynamic_")]
        self.assertEqual(len(dynamic), 8)
        for body in dynamic:
            self.assertEqual(body.get("mocap"), "true")
            self.assertIsNotNone(self.world.find(f"site[@name='{body.get('name')}_target']"))
        runner = RUNNER.read_text()
        patch = SIMULATOR_PATCH.read_text()
        self.assertIn('IMAV22_RELOCATION_CLEARANCE_M="1.5"', runner)
        self.assertIn("item_index = (requested_cycle - 1) % len(self._imav22_dynamic)", patch)
        self.assertIn("self._imav22_relocation_clearance + item['radius']", patch)
        self.assertIn("self.data.mocap_quat[item['mocap_id']]", patch)
        self.assertIn("segment_distance", patch)

    def test_pinned_isaac_texture_contract(self):
        expected = {
            "GateTex.png": "5643f7bea93f18bef334407efba8cc5cecdb0a26bd355e495cc062fdcefc4b64",
            "grass_green.png": "14afb8bc3961dcdee01a4f833fd68024177e6d30d4842cdc1c7fc4ff779e5a78",
            "grass_black.png": "3c5059fea187f6b3ce05a53d9cbc67fe18f4247fb36560cd7719366370befc7d",
            "grass_blue.png": "6782d555b7ff0333dd71173644f599bfd435643b95594a2ee723f27e0eefe52e",
            "orange_pole.png": "27574859a4969b59c24f856d19bda908771d3387f004ca1216ce4c3b3fc21eba",
            "metal_panel1.png": "aba3c4fa41ed11e7ef7e3c75f53264b164e87865b3542905423ee259431358ab",
            "metal_panel3.png": "2801e1ff6dca49624bff5c7e6c55005dc2635e0b89a17d4c71576fb36de5a0e2",
            "flag_white_blue_proxy_v1.png": "8fdf17147fb5055e2514c906590a7d2a69be3f4408415e05c618fc177d88150e",
            "feather_flag_proxy.obj": "0f7a0c868b42b13f8fe28eabca7968c9bea15575b23345ca0a79cef7d934f0ad",
            "imav2022-gate.dae": "afa65a0b5130a268a1a73f5a81cd039ddad78b05eb1afd236320cc7328f36be1",
            "imav2022-gate.obj": "881e0c58b918303e36db5cbeb09c8ef13c6f34541e8c84c0c43b8e51152f6806",
        }
        for name, digest in expected.items():
            self.assertEqual(hashlib.sha256((TEXTURES / name).read_bytes()).hexdigest(), digest, name)
        provenance = (TEXTURES / "PROVENANCE.md").read_text()
        self.assertIn("da3636651e43ba7663eb3ed4f73c59f641058cef", provenance)
        self.assertIn("Copyright (c) 2022 Bitcraze", provenance)

    def test_imav_camera_mount_and_projection_match_isaac(self):
        runner = RUNNER.read_text()
        camera_patch = CAMERA_PATCH.read_text()
        self.assertIn('if [[ -n "$imav22_mode" ]]', runner)
        self.assertIn("camera_width=160; camera_height=160", runner)
        self.assertIn("camera_fovy=83.6091095", runner)
        self.assertIn('"ai_deck_optical_center_body_m": [0.0, 0.0, 0.030]', runner)
        self.assertIn('pos="0 0 0.03"', camera_patch)
        self.assertEqual(self.manifest["camera_contract"]["body_optical_center_m"],
                         [0.0, 0.0, 0.03])

    def test_extended_objects_remain_explicitly_non_official(self):
        extra_geoms = [geom for geom in self.world.iter("geom")
                       if geom.get("name", "").startswith("imav_extra_")]
        self.assertEqual(len(extra_geoms), 11)
        self.assertIn("not an official scoring configuration",
                      self.manifest["optional_object_sets"]["extended"]["purpose"])


if __name__ == "__main__":
    unittest.main()
