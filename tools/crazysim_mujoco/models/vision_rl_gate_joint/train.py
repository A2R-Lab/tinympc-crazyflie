#!/usr/bin/env python3
"""Train/export the compact joint student using only NPZ offline data."""
from __future__ import annotations
import argparse, hashlib, json, random
from pathlib import Path
import numpy as np
import torch
import torch.nn.functional as F
from .model import JointTemporalPolicy

RUNTIME_OBSTACLE_TEACHER_SHA256="291d1de3a7152f09cc2e96f4a6973d322c95bade4e5c8249c7bc14f7b656187e"
RUNTIME_OBSTACLE_COURSE_SHA256="1fcc8d41f70a13528de9ea8be92ed507156b39d932f748b1c2a13fca1302e14c"
RUNTIME_OBSTACLE_SOURCE_HASHES={"run_config_sha256":"e0bf5eedca94f374f123bb7a86c633e3663080d9a0add1a381c19d593567aa76","summary_sha256":"4b30c1e6567c141fc2687876ae3beaf97c5cbe1ba6ace76690a54c8a64dfb946","state_sha256":"b8baf5f9abb27a62a03fb91e9b95dd445edfab99ff606bd169addb756ff61a72","vision_sha256":"f7010ea8951de4a4f857e7685b37d8fc9767cc476b66f76793c375658a1b36d6","video_sha256":"91da516dbc92c22e7393683c3c61d5dbf57702e97a78944c159f443e0c665d4b"}
RUNTIME_OBSTACLE_SELECTION_RULE="post-HM01B0 H.264 sequence N uses adjacent decoded frames [N-2,N-1] and vision.csv rl_action at N; select N>=2 and time_s<=summary course_completion_time_s"

def sha256(path):
    h=hashlib.sha256()
    with open(path,"rb") as f:
        for b in iter(lambda:f.read(1<<20),b""): h.update(b)
    return h.hexdigest()
def load(path):
    with np.load(path,allow_pickle=False) as z: return {k:np.asarray(z[k]) for k in z.files}
def array_sha256(value):
    """Hash one contiguous array including dtype/shape provenance."""
    array=np.ascontiguousarray(value)
    h=hashlib.sha256();h.update(array.dtype.str.encode());h.update(str(array.shape).encode());h.update(array.tobytes())
    return h.hexdigest()
def load_runtime_track_retention(path: Path):
    """Validate runtime clear-flight TRACK and projected-gate supervision."""
    if not path.is_file(): raise FileNotFoundError(f"runtime TRACK retention NPZ missing: {path}")
    data=load(path)
    required={"frames","expert_action","source_run_index","source_frame_index","source_time_s",
              "gate_corners","gate_visible","source_runs_json","gate_projection_json",
              "source_course_sha256","selection_rule"}
    if set(data) != required: raise RuntimeError(f"runtime TRACK retention schema must be exactly {sorted(required)}")
    frames=data["frames"];action=data["expert_action"];run_index=data["source_run_index"];frame_index=data["source_frame_index"];source_time=data["source_time_s"]
    corners=data["gate_corners"];visible=data["gate_visible"]
    count=len(frames)
    if count < 1 or frames.dtype != np.uint8 or frames.shape != (count,2,160,160): raise RuntimeError("runtime TRACK frames must be nonempty uint8[N,2,160,160]")
    if action.dtype != np.int64 or action.shape != (count,) or np.any(action != 0): raise RuntimeError("runtime TRACK expert_action must be int64[N] and all TRACK=0")
    if run_index.dtype != np.int64 or run_index.shape != (count,) or np.any(run_index < 0) or np.any(np.diff(run_index) < 0): raise RuntimeError("runtime TRACK source_run_index must be nondecreasing nonnegative int64[N]")
    unique_runs=np.unique(run_index)
    if not np.array_equal(unique_runs,np.arange(len(unique_runs),dtype=np.int64)): raise RuntimeError("runtime TRACK source_run_index must be contiguous from zero")
    if frame_index.dtype != np.int64 or frame_index.shape != (count,) or np.any(frame_index < 0): raise RuntimeError("runtime TRACK source_frame_index must be nonnegative int64[N]")
    if source_time.dtype not in (np.float32,np.float64) or source_time.shape != (count,) or np.any(~np.isfinite(source_time)) or np.any(source_time < 0): raise RuntimeError("runtime TRACK source_time_s must be finite nonnegative float[N]")
    if corners.dtype != np.float32 or corners.shape != (count,4,2) or np.any(~np.isfinite(corners)) or np.any((corners < 0.) | (corners > 1.)): raise RuntimeError("runtime TRACK gate_corners must be finite normalized float32[N,4,2]")
    if visible.dtype != np.float32 or visible.shape != (count,) or np.any((visible != 0.) & (visible != 1.)): raise RuntimeError("runtime TRACK gate_visible must be binary float32[N]")
    if np.any(corners[visible == 0.] != 0.): raise RuntimeError("runtime TRACK invisible gate_corners must be exactly zero")
    for index in unique_runs:
        selected=run_index==index
        if np.any(np.diff(frame_index[selected]) <= 0) or np.any(np.diff(source_time[selected]) < 0): raise RuntimeError(f"runtime TRACK run {index} frame/time provenance is not ordered")
    source_runs_text=str(data["source_runs_json"].item()) if data["source_runs_json"].shape == () else ""
    try: source_runs=json.loads(source_runs_text)
    except json.JSONDecodeError as error: raise RuntimeError("runtime TRACK source_runs_json is invalid") from error
    if not isinstance(source_runs,list) or len(source_runs) != len(unique_runs) or json.dumps(source_runs,sort_keys=True,separators=(",",":")) != source_runs_text: raise RuntimeError("runtime TRACK source_runs_json must be one canonical JSON entry per run")
    for entry in source_runs:
        if not isinstance(entry,dict) or set(entry) != {"path","video_sha256","state_sha256","run_config_sha256"} or not Path(entry["path"]).is_absolute(): raise RuntimeError("runtime TRACK source run provenance contract mismatch")
        for key in ("video_sha256","state_sha256","run_config_sha256"):
            text=entry[key]
            if not isinstance(text,str) or len(text) != 64 or any(character not in "0123456789abcdef" for character in text): raise RuntimeError(f"runtime TRACK source run {key} must be lowercase SHA-256")
    projection_text=str(data["gate_projection_json"].item()) if data["gate_projection_json"].shape == () else ""
    try: projection=json.loads(projection_text)
    except json.JSONDecodeError as error: raise RuntimeError("runtime TRACK gate_projection_json is invalid") from error
    projection_keys={"schema","source","corner_order","rail_center_span_m","course_gate",
                     "ai_deck_optical_center_body_m","camera_axes","projection",
                     "normalization_divisor_px","runs"}
    if (not isinstance(projection,dict) or set(projection) != projection_keys or
            json.dumps(projection,sort_keys=True,separators=(",",":")) != projection_text):
        raise RuntimeError("runtime TRACK gate_projection_json must be the exact canonical projection schema")
    if (projection["schema"] != "runtime_gate_projection_v1" or
            projection["source"] != "interpolated state.csv body pose + run_config HM01B0 calibration + course gate geometry" or
            projection["corner_order"] != ["TL","TR","BR","BL"] or
            projection["rail_center_span_m"] != .555 or
            projection["ai_deck_optical_center_body_m"] != [0.,0.,.01] or
            projection["camera_axes"] != {"u":"-body_y","v":"-body_z","depth":"+body_x"} or
            projection["projection"] != "cv2.projectPoints with zero extrinsics and simulation distortion" or
            projection["normalization_divisor_px"] != [160,160]):
        raise RuntimeError("runtime TRACK gate projection geometry/convention mismatch")
    gate=projection["course_gate"]
    if not isinstance(gate,dict) or set(gate) != {"center","normal","opening"}:
        raise RuntimeError("runtime TRACK course gate projection provenance mismatch")
    for key,size in (("center",3),("normal",2),("opening",2)):
        value=gate[key]
        if not isinstance(value,list) or len(value) != size or not np.all(np.isfinite(value)):
            raise RuntimeError(f"runtime TRACK course gate {key} projection provenance mismatch")
    if np.any(np.asarray(gate["opening"],dtype=np.float64) <= 0.) or np.linalg.norm(gate["normal"]) <= 0.:
        raise RuntimeError("runtime TRACK course gate opening/normal projection provenance mismatch")
    calibrations=projection["runs"]
    calibration_keys={"model","resolution","fx_px","fy_px","cx_px","cy_px",
                      "distortion_model","simulation_distortion_coefficients"}
    if not isinstance(calibrations,list) or len(calibrations) != len(unique_runs):
        raise RuntimeError("runtime TRACK projection must contain one calibration per run")
    for calibration in calibrations:
        if (not isinstance(calibration,dict) or set(calibration) != calibration_keys or
                calibration["model"] != "Himax HM01B0" or calibration["resolution"] != [160,160] or
                calibration["distortion_model"] != "opencv_plumb_bob"):
            raise RuntimeError("runtime TRACK HM01B0 projection calibration contract mismatch")
        numeric=[calibration[key] for key in ("fx_px","fy_px","cx_px","cy_px")]
        distortion=calibration["simulation_distortion_coefficients"]
        if len(distortion) != 5 or not np.all(np.isfinite(numeric+distortion)) or numeric[0] <= 0. or numeric[1] <= 0.:
            raise RuntimeError("runtime TRACK HM01B0 projection calibration is invalid")
    course_value=data["source_course_sha256"]
    course_hash=str(course_value.item()) if course_value.shape == () else ""
    if len(course_hash) != 64 or any(character not in "0123456789abcdef" for character in course_hash): raise RuntimeError("runtime TRACK source_course_sha256 must be one lowercase SHA-256 scalar")
    selection_rule=str(data["selection_rule"].item()) if data["selection_rule"].shape == () else ""
    if not selection_rule.strip(): raise RuntimeError("runtime TRACK selection_rule must be one nonempty scalar")
    provenance={"path":str(path.resolve()),"sha256":sha256(path),"samples":count,
                "runs":len(unique_runs),"source_run_index_sha256":array_sha256(run_index),
                "source_frame_index":{"minimum":int(frame_index.min()),"maximum":int(frame_index.max()),"sha256":array_sha256(frame_index)},
                "source_time_s":{"minimum":float(source_time.min()),"maximum":float(source_time.max()),"sha256":array_sha256(source_time)},
                "source_runs":source_runs,"source_course_sha256":course_hash,"selection_rule":selection_rule,
                "gate_projection":projection,"gate_corners_sha256":array_sha256(corners),
                "gate_visible":{"visible_samples":int(visible.sum()),"sha256":array_sha256(visible)},
                "supervision":"actor TRACK cross-entropy plus masked gate corners and gate confidence only; no risk, reward, or value targets"}
    return data,provenance
def runtime_track_retention_loss(model, frames, action, corners, visible,
                                 action_weight, corner_weight, confidence_weight):
    """Supervise only deployed actor/gate outputs from privileged runtime labels."""
    logits,gpred,gconf,_risk,_value=model.supervised(frames)
    actor=F.cross_entropy(logits,action)
    corner=masked_corner_loss(gpred,corners,visible)
    confidence=F.binary_cross_entropy_with_logits(gconf,visible)
    return action_weight*actor+corner_weight*corner+confidence_weight*confidence
def load_runtime_obstacle_retention(path: Path):
    """Validate the provenance-locked frozen-teacher obstacle correction."""
    if not path.is_file(): raise FileNotFoundError(f"runtime obstacle retention NPZ missing: {path}")
    data=load(path)
    required={"frames","expert_action","source_sequence","source_frame_index","source_time_s","action_counts","array_hashes_json","source_run_json","teacher_policy_sha256","source_course_sha256","selection_rule"}
    if set(data) != required: raise RuntimeError(f"runtime obstacle retention schema must be exactly {sorted(required)}")
    frames=data["frames"];action=data["expert_action"];sequence=data["source_sequence"];frame_index=data["source_frame_index"];source_time=data["source_time_s"];counts=data["action_counts"]
    count=len(frames)
    if count != 664 or frames.dtype != np.uint8 or frames.shape != (count,2,160,160): raise RuntimeError("runtime obstacle frames must be exactly uint8[664,2,160,160]")
    if action.dtype != np.int64 or action.shape != (count,) or np.any((action < 0) | (action > 2)): raise RuntimeError("runtime obstacle expert_action must be int64[N] in TRACK/LEFT/RIGHT")
    if counts.dtype != np.int64 or counts.shape != (3,) or not np.array_equal(counts,np.asarray([618,0,46],np.int64)) or not np.array_equal(counts,np.bincount(action,minlength=3)): raise RuntimeError("runtime obstacle action_counts must exactly match [618,0,46]")
    if sequence.dtype != np.int64 or not np.array_equal(sequence,np.arange(2,666,dtype=np.int64)): raise RuntimeError("runtime obstacle source_sequence must be exactly 2..665")
    if frame_index.dtype != np.int64 or not np.array_equal(frame_index,sequence-1): raise RuntimeError("runtime obstacle source_frame_index must be zero-based current frame N-1")
    expected_time=np.round(sequence/30.,3)
    if source_time.dtype != np.float64 or source_time.shape != (count,) or np.any(~np.isfinite(source_time)) or np.any(np.abs(source_time-expected_time) > 5.01e-4): raise RuntimeError("runtime obstacle source_time_s must exactly align sequence at 30 Hz")
    hashes_text=str(data["array_hashes_json"].item()) if data["array_hashes_json"].shape == () else ""
    try: hashes=json.loads(hashes_text)
    except json.JSONDecodeError as error: raise RuntimeError("runtime obstacle array_hashes_json is invalid") from error
    expected_hashes={key:array_sha256(data[key]) for key in ("frames","expert_action","source_sequence","source_frame_index","source_time_s")}
    if hashes != expected_hashes or json.dumps(hashes,sort_keys=True,separators=(",",":")) != hashes_text: raise RuntimeError("runtime obstacle array hashes must exactly match frames/actions/frame/time provenance")
    source_text=str(data["source_run_json"].item()) if data["source_run_json"].shape == () else ""
    try: source=json.loads(source_text)
    except json.JSONDecodeError as error: raise RuntimeError("runtime obstacle source_run_json is invalid") from error
    source_keys={"path","course_completion_time_s",*RUNTIME_OBSTACLE_SOURCE_HASHES}
    if (not isinstance(source,dict) or set(source) != source_keys or
            json.dumps(source,sort_keys=True,separators=(",",":")) != source_text or
            not isinstance(source["path"],str) or not Path(source["path"]).is_absolute() or
            {key:source[key] for key in RUNTIME_OBSTACLE_SOURCE_HASHES} != RUNTIME_OBSTACLE_SOURCE_HASHES):
        raise RuntimeError("runtime obstacle source provenance does not match the pinned run")
    completion=source["course_completion_time_s"]
    if not isinstance(completion,(int,float)) or not np.isfinite(completion) or source_time[-1] > completion or completion >= round((int(sequence[-1])+1)/30.,3): raise RuntimeError("runtime obstacle completion-time selection provenance mismatch")
    for field,expected,label in (("teacher_policy_sha256",RUNTIME_OBSTACLE_TEACHER_SHA256,"teacher"),("source_course_sha256",RUNTIME_OBSTACLE_COURSE_SHA256,"course"),("selection_rule",RUNTIME_OBSTACLE_SELECTION_RULE,"selection")):
        value=data[field]
        if value.shape != () or str(value.item()) != expected: raise RuntimeError(f"runtime obstacle {label} provenance mismatch")
    provenance={"path":str(path.resolve()),"sha256":sha256(path),"samples":count,
                "action_counts_track_left_right":counts.tolist(),"source_run":source,
                "teacher_policy_sha256":RUNTIME_OBSTACLE_TEACHER_SHA256,
                "source_course_sha256":RUNTIME_OBSTACLE_COURSE_SHA256,
                "source_sequence_sha256":array_sha256(sequence),"source_frame_index_sha256":array_sha256(frame_index),
                "frames_sha256":hashes["frames"],"expert_action_sha256":hashes["expert_action"],
                "source_time_s":{"minimum":float(source_time[0]),"maximum":float(source_time[-1]),"sha256":array_sha256(source_time)},
                "selection_rule":RUNTIME_OBSTACLE_SELECTION_RULE,
                "supervision":"actor action cross-entropy only; no gate, risk, reward, or value targets"}
    return data,provenance
def runtime_obstacle_retention_loss(model, frames, action):
    """Keep the frozen-teacher obstacle path actor-only."""
    return F.cross_entropy(model(frames)[0],action)
def load_newbee_train_supervision(images_root: Path, targets_root: Path):
    """Load only split-listed NewBee *train* images for gate-only losses."""
    split_path=targets_root/"split_shards.json"
    if not images_root.is_dir() or not targets_root.is_dir() or not split_path.is_file():
        raise FileNotFoundError("NewBee images, targets, and split_shards.json are required")
    split=json.loads(split_path.read_text())
    if set(split) != {"seed","grouping","warning","train","validation","test"} or not isinstance(split["train"],list):
        raise RuntimeError("unexpected NewBee split_shards contract")
    forbidden=set(split["validation"])|set(split["test"])
    if forbidden & set(split["train"]): raise RuntimeError("NewBee split leakage")
    records=[]; shard_hashes={}; clipped_valid_coordinates=0; image_inventory=hashlib.sha256()
    from PIL import Image
    for shard in split["train"]:
        if not isinstance(shard,str) or shard in forbidden: raise RuntimeError("non-train NewBee shard requested")
        target_path=targets_root/f"{shard}.npz"; image_dir=images_root/shard
        if not target_path.is_file() or not image_dir.is_dir(): raise FileNotFoundError(f"missing NewBee shard: {shard}")
        with np.load(target_path,allow_pickle=False) as archive:
            if set(("corners_xy160_f16","corner_valid_u8","global_indices_i32")) - set(archive.files): raise RuntimeError(f"NewBee target schema missing in {shard}")
            corners=np.asarray(archive["corners_xy160_f16"],np.float32); valid=np.asarray(archive["corner_valid_u8"],np.uint8); indices=np.asarray(archive["global_indices_i32"],np.int64)
        if corners.ndim != 3 or corners.shape[1:] != (4,2) or valid.shape != (len(corners),) or indices.shape != (len(corners),): raise RuntimeError(f"NewBee target shape mismatch: {shard}")
        # Invalid targets are intentionally NaN in the source corpus; only
        # valid rail-centre rows must be finite/in-range, then invalid rows are
        # zeroed before the masked loss to avoid NaN * 0 propagation.
        valid_rows=valid.astype(bool)
        if (np.any((valid != 0) & (valid != 1)) or np.any(~np.isfinite(corners[valid_rows])) or
                np.any(corners[valid_rows] < -4) or np.any(corners[valid_rows] > 164)):
            raise RuntimeError(f"NewBee target range mismatch: {shard}")
        clipped_valid_coordinates+=int(np.count_nonzero((corners[valid_rows] < 0) | (corners[valid_rows] > 160)))
        corners[valid_rows]=np.clip(corners[valid_rows],0.,160.)
        corners[~valid_rows]=0.0
        if len(set(indices.tolist())) != len(indices): raise RuntimeError(f"duplicate NewBee image indices: {shard}")
        for index, corner, is_valid in zip(indices,corners,valid):
            image_path=image_dir/f"hm01b0_mono_{int(index)%1000:04d}.png"
            if not image_path.is_file(): raise FileNotFoundError(f"missing NewBee image: {image_path}")
            with Image.open(image_path) as image:
                if image.mode != "L" or image.size != (160,160): raise RuntimeError(f"NewBee image must be grayscale 160x160: {image_path}")
            image_inventory.update(f"{shard}/{image_path.name}:{sha256(image_path)}\n".encode())
            records.append((image_path,corner/160.,float(is_valid)))
        shard_hashes[shard]=sha256(target_path)
    if not records: raise RuntimeError("NewBee train split has no gate-supervision records")
    return records,{"images_root":str(images_root.resolve()),"targets_root":str(targets_root.resolve()),"split_shards_sha256":sha256(split_path),"train_shards":list(split["train"]),"target_shard_sha256":shard_hashes,"image_inventory_sha256":image_inventory.hexdigest(),"records":len(records),"clipped_valid_coordinates":clipped_valid_coordinates,"coordinate_range_validation":"finite valid coordinates must be [-4,164] px; bounded one-sided image-edge projection is clipped to [0,160] before normalization","excluded_validation_shards":list(split["validation"]),"excluded_test_shards":list(split["test"])}
def newbee_batch(records, batch_size, generator, device):
    """Duplicate each independent still into [previous,current]; no policy labels."""
    from PIL import Image
    indices=torch.randint(len(records),(min(len(records),batch_size),),generator=generator).tolist()
    images=[]; corners=[]; visible=[]
    for index in indices:
        path, corner, valid=records[index]
        with Image.open(path) as image: pixels=np.asarray(image,dtype=np.uint8).copy()
        images.append(np.stack((pixels,pixels))); corners.append(corner); visible.append(valid)
    return (torch.as_tensor(np.asarray(images),device=device,dtype=torch.float32)/255.,
            torch.as_tensor(np.asarray(corners),device=device,dtype=torch.float32),
            torch.as_tensor(visible,device=device,dtype=torch.float32))
def discounted_returns(reward, episode_id, gamma=.98):
    """Episodic returns; no return crosses an exporter episode boundary."""
    result=np.zeros(len(reward),np.float32); running=0.
    for i in range(len(reward)-1,-1,-1):
        if i == len(reward)-1 or episode_id[i] != episode_id[i+1]: running=0.
        running=float(reward[i])+gamma*running; result[i]=running
    return result
def advantage_weighted_ce(logits, action, returns, value, temperature=4., clip=2.):
    """Clipped AWR: ordinary CE remains alongside this bounded reward signal."""
    advantage=(returns-value.detach()).clamp(-temperature*clip, temperature*clip)
    weights=torch.exp(advantage/temperature).clamp(1./np.exp(clip),np.exp(clip))
    return (weights*F.cross_entropy(logits,action,reduction="none")).mean(), weights
def masked_corner_loss(prediction, target, visible):
    visible=visible.reshape(-1).float()
    per_sample=F.smooth_l1_loss(prediction,target,reduction="none").mean((1,2))
    return (per_sample*visible).sum()/visible.sum().clamp_min(1.)
def validate_joint_shards(shards, paths, manifest_path=None):
    required={"frames","action","gate_corners","gate_visible","obstacle_risk","reward","episode_id","scene_id","layout_seed"}
    if any(not required <= set(x) for x in shards): raise RuntimeError("joint shard missing required image, reward, privileged, scene, or layout provenance fields")
    groups=[set(zip(x["scene_id"].tolist(),x["layout_seed"].tolist())) for x in shards]
    if any(groups[i]&groups[j] for i in range(3) for j in range(i)): raise RuntimeError("scene/layout leakage between joint splits")
    for shard in shards:
        if shard["frames"].ndim != 4 or shard["frames"].shape[1:] != (2,160,160): raise RuntimeError("joint frames must be [N,2,160,160]")
        if np.any((shard["gate_corners"] < 0) | (shard["gate_corners"] > 1)): raise RuntimeError("gate corners must be normalized to [0,1]")
    if manifest_path is None: return None
    manifest=json.loads(manifest_path.read_text()); expected=manifest.get("splits",{})
    names=("train","validation","test")
    for name,path in zip(names,paths):
        entry=expected.get(name,{})
        if entry.get("sha256") != sha256(path): raise RuntimeError(f"external manifest hash mismatch for {name}")
    return {"path":str(manifest_path.resolve()),"sha256":sha256(manifest_path)}
def metrics(model, data, device):
    model.eval()
    with torch.no_grad():
        x=torch.as_tensor(data["frames"],device=device,dtype=torch.float32)/255.
        logits,corners,confidence_logit=model(x)
        pred=logits.argmax(1).cpu().numpy()
        corners=corners.cpu().numpy()
        confidence=(torch.sigmoid(confidence_logit)>=.5).cpu().numpy()
    y=data["action"]; recalls=[]
    for cls in range(3):
        rows=y==cls; recalls.append(float((pred[rows]==cls).mean()) if rows.any() else 0.)
    visible=data["gate_visible"].astype(bool); true_positive=int(np.sum(confidence & visible)); false_positive=int(np.sum(confidence & ~visible)); false_negative=int(np.sum(~confidence & visible))
    precision=true_positive/max(true_positive+false_positive,1); gate_recall=true_positive/max(true_positive+false_negative,1)
    corner_mae_px=(float(np.mean(np.abs(corners[visible]-data["gate_corners"][visible]))*160.)
                   if np.any(visible) else None)
    return {"accuracy":float((pred==y).mean()),"macro_recall":float(np.mean(recalls)),"recall":recalls,"samples":int(len(y)),"gate_visibility":{"precision":precision,"recall":gate_recall,"f1":2.*precision*gate_recall/max(precision+gate_recall,1.e-12)},"visible_gate_corner_mae_px":corner_mae_px}
def selection_score(report):
    """Balance navigation, gate presence, and usable corner geometry."""
    corner_mae=report["visible_gate_corner_mae_px"]
    corner_quality=0. if corner_mae is None else max(0.,1.-corner_mae/40.)
    return .5*report["macro_recall"]+.25*report["gate_visibility"]["f1"]+.25*corner_quality
def main():
    p=argparse.ArgumentParser(); p.add_argument("--config",type=Path,required=True);p.add_argument("--dataset",type=Path,required=True);p.add_argument("--external-train",type=Path);p.add_argument("--external-validation",type=Path);p.add_argument("--external-test",type=Path);p.add_argument("--external-manifest",type=Path);p.add_argument("--retention-dataset",type=Path);p.add_argument("--runtime-track-retention",type=Path);p.add_argument("--runtime-obstacle-retention",type=Path);p.add_argument("--newbee-images",type=Path);p.add_argument("--newbee-targets",type=Path);p.add_argument("--output",type=Path,required=True);p.add_argument("--device",default="cpu"); a=p.parse_args()
    cfg=json.loads(a.config.read_text()); seed=int(cfg["seed"]);random.seed(seed);np.random.seed(seed);torch.manual_seed(seed);torch.use_deterministic_algorithms(True,warn_only=True)
    external=(a.external_train,a.external_validation,a.external_test)
    if any(external) and (not all(external) or not a.external_manifest): raise RuntimeError("external training requires train/validation/test shards and manifest")
    train, val, test=(load(x) for x in external) if all(external) else (load(a.dataset/"train.npz"),load(a.dataset/"validation.npz"),load(a.dataset/"test.npz"))
    source=validate_joint_shards((train,val,test), external if all(external) else (a.dataset/"train.npz",a.dataset/"validation.npz",a.dataset/"test.npz"), a.external_manifest if all(external) else None)
    train_returns=discounted_returns(train["reward"],train["episode_id"])
    retention=None
    if a.retention_dataset:
        retention=load(a.retention_dataset)
        if not {"frames","expert_action"} <= set(retention): raise RuntimeError("retention NPZ must be existing frozen obstacle expert schema")
    runtime_track,runtime_track_provenance=(load_runtime_track_retention(a.runtime_track_retention.resolve()) if a.runtime_track_retention else (None,None))
    runtime_obstacle,runtime_obstacle_provenance=(load_runtime_obstacle_retention(a.runtime_obstacle_retention.resolve()) if a.runtime_obstacle_retention else (None,None))
    if bool(a.newbee_images) != bool(a.newbee_targets): raise RuntimeError("NewBee gate supervision requires both --newbee-images and --newbee-targets")
    newbee_records,newbee_provenance=(load_newbee_train_supervision(a.newbee_images.resolve(),a.newbee_targets.resolve()) if a.newbee_images else (None,None))
    device=torch.device(a.device); model=JointTemporalPolicy(cfg["model_width"]).to(device); opt=torch.optim.AdamW(model.parameters(),lr=cfg["learning_rate"],weight_decay=cfg["weight_decay"])
    generator=torch.Generator().manual_seed(seed); best=(-1.,None); n=len(train["action"])
    for epoch in range(int(cfg["epochs"])):
        model.train()
        for _ in range(max(1,n//int(cfg["batch_size"]))):
            ix=torch.randint(n,(min(n,int(cfg["batch_size"])),),generator=generator)
            x=torch.as_tensor(train["frames"][ix],device=device,dtype=torch.float32)/255.; y=torch.as_tensor(train["action"][ix],device=device)
            corners=torch.as_tensor(train["gate_corners"][ix],device=device,dtype=torch.float32); visible=torch.as_tensor(train["gate_visible"][ix],device=device,dtype=torch.float32); risk=torch.as_tensor(train["obstacle_risk"][ix],device=device,dtype=torch.float32)
            returns=torch.as_tensor(train_returns[ix],device=device)
            logits,gpred,gconf,rpred,value=model.supervised(x)
            # Action is task policy; gate/risk privileged losses shape the shared encoder.
            awr,weights=advantage_weighted_ce(logits,y,returns,value[:,0])
            loss=F.cross_entropy(logits,y)+cfg["awr_loss_weight"]*awr+cfg["rollout_gate_corner_loss_weight"]*masked_corner_loss(gpred,corners,visible)+cfg["rollout_gate_confidence_loss_weight"]*F.binary_cross_entropy_with_logits(gconf,visible)+cfg["risk_loss_weight"]*F.mse_loss(torch.sigmoid(rpred[:,0]),risk)+cfg["value_loss_weight"]*F.mse_loss(value[:,0],returns)
            if newbee_records is not None:
                nx,ncorners,nvisible=newbee_batch(newbee_records,int(cfg["batch_size"]),generator,device)
                _nlogits,ngpred,ngconf,_nrisk,_nvalue=model.supervised(nx)
                # These stills deliberately have no action, reward, risk, or return loss.
                loss=loss+cfg["newbee_gate_corner_loss_weight"]*masked_corner_loss(ngpred,ncorners,nvisible)+cfg["newbee_gate_confidence_loss_weight"]*F.binary_cross_entropy_with_logits(ngconf,nvisible)
            if retention is not None:
                ri=torch.randint(len(retention["expert_action"]),(min(len(retention["expert_action"]),int(cfg["batch_size"])),),generator=generator)
                rx=torch.as_tensor(retention["frames"][ri],device=device,dtype=torch.float32)/255.; ry=torch.as_tensor(retention["expert_action"][ri],device=device)
                loss=loss+cfg["retention_action_loss_weight"]*F.cross_entropy(model(rx)[0],ry)
            if runtime_track is not None:
                ti=torch.randint(len(runtime_track["expert_action"]),(min(len(runtime_track["expert_action"]),int(cfg["batch_size"])),),generator=generator)
                tx=torch.as_tensor(runtime_track["frames"][ti],device=device,dtype=torch.float32)/255.;ty=torch.as_tensor(runtime_track["expert_action"][ti],device=device);tcorners=torch.as_tensor(runtime_track["gate_corners"][ti],device=device,dtype=torch.float32);tvisible=torch.as_tensor(runtime_track["gate_visible"][ti],device=device,dtype=torch.float32)
                loss=loss+runtime_track_retention_loss(model,tx,ty,tcorners,tvisible,cfg["runtime_track_retention_action_loss_weight"],cfg["runtime_track_retention_gate_corner_loss_weight"],cfg["runtime_track_retention_gate_confidence_loss_weight"])
            if runtime_obstacle is not None:
                oi=torch.randint(len(runtime_obstacle["expert_action"]),(min(len(runtime_obstacle["expert_action"]),int(cfg["batch_size"])),),generator=generator)
                ox=torch.as_tensor(runtime_obstacle["frames"][oi],device=device,dtype=torch.float32)/255.;oy=torch.as_tensor(runtime_obstacle["expert_action"][oi],device=device)
                loss=loss+cfg["runtime_obstacle_retention_action_loss_weight"]*runtime_obstacle_retention_loss(model,ox,oy)
            opt.zero_grad();loss.backward();opt.step()
        score=selection_score(metrics(model,val,device))
        if score>best[0]: best=(score,{k:v.detach().cpu() for k,v in model.state_dict().items()})
    model.load_state_dict(best[1]); a.output.mkdir(parents=True,exist_ok=True)
    dataset_manifest=a.external_manifest if all(external) else a.dataset/"manifest.json"
    checkpoint=a.output/"checkpoint.pt";torch.save({"model":model.state_dict(),"width":cfg["model_width"],"config":cfg,"dataset_manifest_sha256":sha256(dataset_manifest)},checkpoint)
    onnx=a.output/"policy.onnx"; model.eval();torch.onnx.export(model,torch.zeros(1,2,160,160,device=device),onnx,input_names=["frames"],output_names=["action_logits","gate_corners","gate_confidence_logit"],opset_version=17,dynamo=False)
    contract={"input":{"name":"frames","shape":[1,2,160,160],"dtype":"float32","range":[0,1],"temporal_order":["previous","current"]},"outputs":{"action_logits":[1,3],"gate_corners":{"shape":[1,4,2],"range":[0,1],"order":["TL","TR","BR","BL"]},"gate_confidence_logit":[1]},"actions":["TRACK","LEFT","RIGHT"],"fixed_intrinsics":{"fx_normalized":89.1558392549/160,"fy_normalized":89.4608171623/160,"cx_normalized":81.103810523/160,"cy_normalized":73.3473030288/160}}
    report={"train":metrics(model,train,device),"validation":metrics(model,val,device),"sealed_test":metrics(model,test,device),"best_validation_joint_score":best[0],"validation_selection":"0.50 action macro recall + 0.25 gate visibility F1 + 0.25 max(0, 1-corner_mae_px/40)","dataset_manifest":{"path":str(dataset_manifest.resolve()),"sha256":sha256(dataset_manifest),"external":bool(external[0])},"external_source":source,"reward_actor_learning":{"method":"clipped advantage-weighted cross entropy plus ordinary CE","gamma":.98,"temperature":4.,"clip":2.},"loss_weights":{key:value for key,value in cfg.items() if key.endswith("_loss_weight")},"newbee_gate_supervision":newbee_provenance,"retention_dataset_sha256":sha256(a.retention_dataset) if a.retention_dataset else None,"runtime_track_retention":runtime_track_provenance,"runtime_obstacle_retention":runtime_obstacle_provenance,"checkpoint_sha256":sha256(checkpoint),"policy_onnx_sha256":sha256(onnx),"scene_leakage":False,"deployment_contract":contract}
    (a.output/"training_metrics.json").write_text(json.dumps(report,indent=2,sort_keys=True)+"\n")
    (a.output/"bundle.json").write_text(json.dumps({"format":"tinympc-joint-gate-obstacle-student-v1","runtime_adapter":"joint_gate_rl","representativeness":"offline only; closed-loop MuJoCo evaluation required before runtime integration","deployment":contract,"artifacts":{"checkpoint":{"path":"checkpoint.pt","sha256":sha256(checkpoint)},"policy_onnx":{"path":"policy.onnx","sha256":sha256(onnx)},"newbee_gate_supervision":newbee_provenance,"runtime_track_retention":runtime_track_provenance,"runtime_obstacle_retention":runtime_obstacle_provenance},"frozen_obstacle_baseline":{"path":"../vision_rl_mpc_balanced_v2/policy.onnx","sha256":"291d1de3a7152f09cc2e96f4a6973d322c95bade4e5c8249c7bc14f7b656187e"}},indent=2,sort_keys=True)+"\n")
if __name__=="__main__":main()
