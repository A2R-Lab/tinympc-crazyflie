import json, tempfile, unittest
from pathlib import Path
import numpy as np
import torch
from PIL import Image
from .generate_dataset import generate, split_for_scene
from .make_runtime_track_retention import canonical_json, project_gate_corners, write_retention
from .make_runtime_obstacle_retention import write_retention as write_obstacle_retention
from .model import JointTemporalPolicy
from .train import RUNTIME_OBSTACLE_SOURCE_HASHES, advantage_weighted_ce, load_newbee_train_supervision, load_runtime_obstacle_retention, load_runtime_track_retention, masked_corner_loss, newbee_batch, runtime_obstacle_retention_loss, runtime_track_retention_loss, selection_score, sha256, validate_joint_shards

def runtime_projection(run_count):
    calibration={"model":"Himax HM01B0","resolution":[160,160],"fx_px":89.1558392549,
                 "fy_px":89.4608171623,"cx_px":81.103810523,"cy_px":73.3473030288,
                 "distortion_model":"opencv_plumb_bob",
                 "simulation_distortion_coefficients":[-.0176448766,.0994132451,.0054432154,-.006040012,-.055]}
    return {"schema":"runtime_gate_projection_v1",
            "source":"interpolated state.csv body pose + run_config HM01B0 calibration + course gate geometry",
            "corner_order":["TL","TR","BR","BL"],"rail_center_span_m":.555,
            "course_gate":{"center":[4.,.3,1.5],"normal":[1.,0.],"opening":[.45,.45]},
            "ai_deck_optical_center_body_m":[0.,0.,.01],
            "camera_axes":{"u":"-body_y","v":"-body_z","depth":"+body_x"},
            "projection":"cv2.projectPoints with zero extrinsics and simulation distortion",
            "normalization_divisor_px":[160,160],"runs":[dict(calibration) for _ in range(run_count)]}

class JointPipelineTest(unittest.TestCase):
    def test_scene_split_is_deterministic_and_isolated(self):
        cfg={"seed":7,"episodes":30,"samples_per_episode":3,"validation_fraction":.3,"test_fraction":.2,"image_size":32,"gate_reward":20.,"contact_penalty":100.,"safe_clearance_m":.2}
        with tempfile.TemporaryDirectory() as tmp:
            first=generate(cfg,Path(tmp)/"a"); second=generate(cfg,Path(tmp)/"b")
            sets=[set(first["splits"][x]["scenes"]) for x in ("train","validation","test")]
            self.assertFalse(any(sets[i]&sets[j] for i in range(3) for j in range(i)))
            self.assertEqual(first["splits"],second["splits"])
            with np.load(Path(tmp)/"a"/"train.npz") as z: self.assertEqual(z["frames"].shape[1:],(2,32,32))
    def test_split_function_has_no_sample_dependency(self):
        self.assertEqual(split_for_scene(8,99,.2),split_for_scene(8,99,.2))
    def test_reward_changes_advantage_weighted_actor_loss(self):
        logits=torch.tensor([[1.,0.,0.],[1.,0.,0.]])
        action=torch.tensor([0,1]); value=torch.zeros(2)
        low,w_low=advantage_weighted_ce(logits,action,torch.tensor([0.,0.]),value)
        high,w_high=advantage_weighted_ce(logits,action,torch.tensor([0.,8.]),value)
        self.assertGreater(float(w_high[1]),float(w_low[1])); self.assertGreater(float(high),float(low))
    def test_invisible_targets_do_not_change_corner_loss(self):
        pred=torch.zeros(2,4,2); visible=torch.tensor([1.,0.]); target=torch.zeros(2,4,2)
        first=masked_corner_loss(pred,target,visible); target[1].fill_(1.)
        self.assertEqual(float(first),float(masked_corner_loss(pred,target,visible)))
    def test_normalized_onnx_contract_model_output(self):
        model=JointTemporalPolicy(); _,corners,_=model(torch.zeros(1,2,160,160))
        self.assertEqual(tuple(corners.shape),(1,4,2)); self.assertTrue(bool(torch.all((corners>=0)&(corners<=1))))
    def test_joint_selection_rewards_gate_quality(self):
        base={"macro_recall":.8,"gate_visibility":{"f1":.5},"visible_gate_corner_mae_px":20.}
        better={**base,"gate_visibility":{"f1":.8},"visible_gate_corner_mae_px":8.}
        self.assertGreater(selection_score(better),selection_score(base))
    def test_external_manifest_hash_and_provenance_leakage(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); paths=[]
            for index,name in enumerate(("train","validation","test")):
                path=root/f"{name}.npz"; paths.append(path)
                np.savez(path,frames=np.zeros((1,2,160,160),np.uint8),action=np.zeros(1,np.int64),gate_corners=np.zeros((1,4,2),np.float32),gate_visible=np.zeros(1,np.float32),obstacle_risk=np.zeros(1,np.float32),reward=np.zeros(1,np.float32),episode_id=np.array([index]),scene_id=np.array([index]),layout_seed=np.array([index]))
            manifest=root/"manifest.json"; manifest.write_text(json.dumps({"splits":{name:{"sha256":sha256(path)} for name,path in zip(("train","validation","test"),paths)}}))
            shards=[np.load(path) for path in paths]
            info=validate_joint_shards(shards,paths,manifest); self.assertEqual(info["sha256"],sha256(manifest))
            leaked=[dict(x) for x in shards]; leaked[1]["scene_id"]=np.array([0]); leaked[1]["layout_seed"]=np.array([0])
            with self.assertRaisesRegex(RuntimeError,"leakage"): validate_joint_shards(leaked,paths,manifest)
    def test_newbee_auxiliary_uses_train_only_and_duplicates_single_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); images=root/"images"; targets=root/"targets"; images.mkdir();targets.mkdir()
            split={"seed":2026,"grouping":"capture shard (one deterministic generator seed per shard)","warning":"fixture","train":["shard_000000000"],"validation":["shard_000001000"],"test":["shard_000002000"]}
            (targets/"split_shards.json").write_text(json.dumps(split))
            directory=images/"shard_000000000";directory.mkdir()
            Image.fromarray(np.full((160,160),17,np.uint8)).save(directory/"hm01b0_mono_0000.png")
            np.savez(targets/"shard_000000000.npz",corners_xy160_f16=np.full((1,4,2),80,np.float16),corner_valid_u8=np.array([1],np.uint8),global_indices_i32=np.array([0],np.int32))
            records,provenance=load_newbee_train_supervision(images,targets)
            self.assertEqual(provenance["records"],1); self.assertEqual(provenance["train_shards"],["shard_000000000"])
            frames,corners,visible=newbee_batch(records,1,torch.Generator().manual_seed(1),torch.device("cpu"))
            self.assertEqual(tuple(frames.shape),(1,2,160,160)); self.assertTrue(torch.equal(frames[:,0],frames[:,1])); self.assertTrue(torch.allclose(corners,torch.full((1,4,2),.5))); self.assertEqual(float(visible[0]),1.)
            split["train"].append("shard_000001000"); (targets/"split_shards.json").write_text(json.dumps(split))
            with self.assertRaisesRegex(RuntimeError,"leakage"): load_newbee_train_supervision(images,targets)
    def test_runtime_track_retention_contract_and_deployed_head_losses(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"track.npz"
            runs=[{"path":f"/runs/{index}","run_config_sha256":str(index+1)*64,"state_sha256":str(index+2)*64,"video_sha256":str(index+3)*64} for index in range(2)]
            source_runs_json=json.dumps(runs,sort_keys=True,separators=(",",":"))
            gate_corners=np.zeros((4,4,2),np.float32);gate_corners[[0,2]]=.5
            np.savez(path,frames=np.zeros((4,2,160,160),np.uint8),expert_action=np.zeros(4,np.int64),source_run_index=np.array([0,0,1,1],np.int64),source_frame_index=np.array([7,8,4,5],np.int64),source_time_s=np.array([.2,.3,.1,.2],np.float64),gate_corners=gate_corners,gate_visible=np.array([1,0,1,0],np.float32),source_runs_json=np.asarray(source_runs_json),gate_projection_json=np.asarray(canonical_json(runtime_projection(2))),source_course_sha256=np.asarray("4"*64),selection_rule=np.asarray("unambiguous pre-gate TRACK"))
            data,provenance=load_runtime_track_retention(path)
            self.assertEqual(provenance["samples"],4);self.assertEqual(provenance["runs"],2);self.assertEqual(provenance["sha256"],sha256(path));self.assertEqual(provenance["source_runs"],runs);self.assertEqual(provenance["gate_visible"]["visible_samples"],2)
            model=JointTemporalPolicy(4);loss=runtime_track_retention_loss(model,torch.as_tensor(data["frames"],dtype=torch.float32),torch.as_tensor(data["expert_action"]),torch.as_tensor(data["gate_corners"]),torch.as_tensor(data["gate_visible"]),.5,.35,.2);loss.backward()
            self.assertIsNotNone(model.actor.weight.grad);self.assertIsNotNone(model.gate_head.weight.grad);self.assertIsNone(model.risk_head.weight.grad);self.assertIsNone(model.value_head.weight.grad)
            data["expert_action"][1]=1
            np.savez(path,**data)
            with self.assertRaisesRegex(RuntimeError,"all TRACK=0"): load_runtime_track_retention(path)
            data["expert_action"][:]=0;data["source_runs_json"]=np.asarray(json.dumps(runs))
            np.savez(path,**data)
            with self.assertRaisesRegex(RuntimeError,"canonical JSON"): load_runtime_track_retention(path)
            data["source_runs_json"]=np.asarray(source_runs_json);data["gate_corners"]=data["gate_corners"].astype(np.float64)
            np.savez(path,**data)
            with self.assertRaisesRegex(RuntimeError,"normalized float32"): load_runtime_track_retention(path)
            data["gate_corners"]=data["gate_corners"].astype(np.float32);projection=runtime_projection(2);projection["rail_center_span_m"]=.45;data["gate_projection_json"]=np.asarray(canonical_json(projection))
            np.savez(path,**data)
            with self.assertRaisesRegex(RuntimeError,"geometry/convention"): load_runtime_track_retention(path)
    def test_runtime_gate_projection_uses_body_camera_axes_and_rail_span(self):
        calibration=runtime_projection(1)["runs"][0].copy()
        calibration["simulation_distortion_coefficients"]=[0.,0.,0.,0.,0.]
        gate={"center":[4.,0.,1.5],"normal":[1.,0.],"opening":[.45,.45]}
        corners,visible=project_gate_corners(np.array([0.,0.,1.49]),np.array([1.,0.,0.,0.]),gate,calibration)
        pixels=corners*160.
        self.assertEqual(float(visible),1.);self.assertLess(pixels[0,0],pixels[1,0]);self.assertLess(pixels[0,1],pixels[3,1])
        self.assertAlmostEqual(float(pixels[1,0]-pixels[0,0]),89.1558392549*.555/4.,places=4)
        self.assertAlmostEqual(float(pixels[3,1]-pixels[0,1]),89.4608171623*.555/4.,places=4)
    def test_runtime_obstacle_retention_is_pinned_and_actor_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"obstacle.npz";sequence=np.arange(2,666,dtype=np.int64);frame_index=sequence-1;times=np.round(sequence/30.,3)
            actions=np.zeros(664,np.int64);actions[-46:]=2
            source={"path":"/home/cchen/tinympc-crazyflie/apps/controller_tinympc_eigen/sim_runs/crazysim/gate_obstacle_joint/joint_model_5912_matrix_seed3201/baseline_obstacle_only_seed3201","course_completion_time_s":22.173000000005384,**RUNTIME_OBSTACLE_SOURCE_HASHES}
            write_obstacle_retention(path,np.zeros((664,2,160,160),np.uint8),actions,sequence,frame_index,times,source)
            data,provenance=load_runtime_obstacle_retention(path)
            self.assertEqual(provenance["samples"],664);self.assertEqual(provenance["action_counts_track_left_right"],[618,0,46]);self.assertEqual(provenance["source_run"],source)
            model=JointTemporalPolicy(4);loss=runtime_obstacle_retention_loss(model,torch.as_tensor(data["frames"][:4],dtype=torch.float32),torch.as_tensor(data["expert_action"][:4]));loss.backward()
            self.assertIsNotNone(model.actor.weight.grad);self.assertIsNone(model.gate_head.weight.grad);self.assertIsNone(model.risk_head.weight.grad);self.assertIsNone(model.value_head.weight.grad)
            data["action_counts"][0]-=1;data["action_counts"][1]+=1;np.savez(path,**data)
            with self.assertRaisesRegex(RuntimeError,"exactly match"):load_runtime_obstacle_retention(path)
    def test_runtime_track_writer_concatenates_independent_run_clocks(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);course=root/"course.json";course.write_text('{"gates":[{"center":[4,0,1.5],"normal":[1,0],"opening":[0.45,0.45]}]}')
            collected=[]
            for run_index in range(2):
                collected.append({"frames":np.full((2,2,160,160),run_index,np.uint8),"frame_indices":np.array([1,2],np.int64),"frame_times":np.array([1/30,2/30],np.float64),"gate_corners":np.full((2,4,2),.5,np.float32),"gate_visible":np.ones(2,np.float32),"maximum_source_x_m":float(run_index),"gate_x_m":4.,"calibration":runtime_projection(1)["runs"][0],"provenance":{"path":f"/runs/{run_index}","video_sha256":str(run_index+1)*64,"state_sha256":str(run_index+2)*64,"run_config_sha256":str(run_index+3)*64}})
            output=root/"retention.npz";summary=write_retention(output,course,collected,.4)
            data,provenance=load_runtime_track_retention(output)
            self.assertEqual(summary["samples"],4);self.assertEqual(provenance["runs"],2)
            np.testing.assert_array_equal(data["source_run_index"],[0,0,1,1]);np.testing.assert_array_equal(data["source_frame_index"],[1,2,1,2])
