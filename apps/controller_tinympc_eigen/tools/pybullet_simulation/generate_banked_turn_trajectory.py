#!/usr/bin/env python3
"""Generate the first smooth 15-degree left-bank simulation trajectory."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np


DT = 0.02
G = 9.81
MASS = 0.045
INERTIA = np.diag([2.3951e-5, 2.3951e-5, 3.2347e-5])
ARM = 0.03535
YAW_RATIO = 7.73e-11 / 3.72e-8


def smooth(u: float) -> float:
    u = min(max(u, 0.0), 1.0)
    return 10 * u**3 - 15 * u**4 + 6 * u**5


def rotation_to_quaternion(R: np.ndarray) -> np.ndarray:
    trace = float(np.trace(R))
    if trace > 0:
        s = math.sqrt(trace + 1) * 2; q = np.asarray([.25*s, (R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s])
    else:
        index = int(np.argmax(np.diag(R)))
        if index == 0:
            s=math.sqrt(1+R[0,0]-R[1,1]-R[2,2])*2; q=np.asarray([(R[2,1]-R[1,2])/s,.25*s,(R[0,1]+R[1,0])/s,(R[0,2]+R[2,0])/s])
        elif index == 1:
            s=math.sqrt(1+R[1,1]-R[0,0]-R[2,2])*2; q=np.asarray([(R[0,2]-R[2,0])/s,(R[0,1]+R[1,0])/s,.25*s,(R[1,2]+R[2,1])/s])
        else:
            s=math.sqrt(1+R[2,2]-R[0,0]-R[1,1])*2; q=np.asarray([(R[1,0]-R[0,1])/s,(R[0,2]+R[2,0])/s,(R[1,2]+R[2,1])/s,.25*s])
    return q / np.linalg.norm(q)


def desired_rotation(acceleration: np.ndarray, yaw: float) -> np.ndarray:
    zb = acceleration + np.asarray([0, 0, G]); zb /= np.linalg.norm(zb)
    heading = np.asarray([math.cos(yaw), math.sin(yaw), 0.0])
    yb = np.cross(zb, heading); yb /= np.linalg.norm(yb)
    xb = np.cross(yb, zb)
    return np.column_stack((xb, yb, zb))


def allocation() -> np.ndarray:
    return np.asarray([[1,1,1,1],[-ARM,-ARM,ARM,ARM],[-ARM,ARM,ARM,-ARM],[-YAW_RATIO,YAW_RATIO,-YAW_RATIO,YAW_RATIO]],float)


def main() -> int:
    ap=argparse.ArgumentParser(); ap.add_argument("--out",type=Path,default=Path("sim/trajectories/banked_left_15.csv")); ap.add_argument("--yaw-deg",type=float,default=90.0); ap.add_argument("--bank-deg",type=float,default=15.0); ap.add_argument("--speed-mps",type=float,default=1.0); ap.add_argument("--speed-ramp-s",type=float,default=1.0); args=ap.parse_args()
    bank_angle = math.radians(float(args.bank_deg))
    cruise_speed = float(args.speed_mps)
    speed_ramp_duration = float(args.speed_ramp_s)
    if speed_ramp_duration <= 0.0:
        raise ValueError("--speed-ramp-s must be positive")
    target_yaw = math.radians(float(args.yaw_deg))
    nominal_ramp_duration = 0.6
    unit_phase = np.linspace(0.0, 1.0, 1001)
    unit_integral = float(np.trapezoid(G * np.tan(bank_angle * np.asarray([smooth(v) for v in unit_phase])) / cruise_speed, unit_phase))
    ramp_duration = min(nominal_ramp_duration, target_yaw / max(1e-9, 2.0 * unit_integral))
    ramp_time = np.arange(0.0, ramp_duration + DT / 2.0, DT)
    ramp_bank = bank_angle * np.asarray([smooth(t / ramp_duration) for t in ramp_time])
    ramp_yaw = 2.0 * float(np.trapezoid(G * np.tan(ramp_bank) / cruise_speed, ramp_time))
    hold_duration = max(0.0, (target_yaw - ramp_yaw) / (G * math.tan(bank_angle) / cruise_speed))
    entry_end=speed_ramp_duration+ramp_duration; hold_end=entry_end+hold_duration; exit_end=hold_end+ramp_duration; end_time=exit_end+speed_ramp_duration
    times=np.arange(0,end_time+DT/2,DT); speed=np.zeros_like(times); bank=np.zeros_like(times)
    for i,t in enumerate(times):
        if t < speed_ramp_duration: speed[i]=cruise_speed*smooth(t/speed_ramp_duration)
        elif t < exit_end: speed[i]=cruise_speed
        else: speed[i]=cruise_speed*(1.0-smooth((t-exit_end)/speed_ramp_duration))
        if speed_ramp_duration <= t < entry_end: bank[i]=-bank_angle*smooth((t-speed_ramp_duration)/ramp_duration)
        elif entry_end <= t < hold_end: bank[i]=-bank_angle
        elif hold_end <= t < exit_end: bank[i]=-bank_angle*(1-smooth((t-hold_end)/ramp_duration))
    yaw_rate=np.where(speed>.1, -G*np.tan(bank)/np.maximum(speed,.1), 0.0)
    yaw=np.zeros_like(times)
    for i in range(1,len(times)): yaw[i]=yaw[i-1]+.5*DT*(yaw_rate[i-1]+yaw_rate[i])
    velocity=np.column_stack((speed*np.cos(yaw),speed*np.sin(yaw),np.zeros_like(speed)))
    position=np.zeros((len(times),3)); position[:,2]=1.1
    for i in range(1,len(times)): position[i]=position[i-1]+.5*DT*(velocity[i-1]+velocity[i]); position[i,2]=1.1
    acceleration=np.gradient(velocity,DT,axis=0,edge_order=2)
    rotations=np.asarray([desired_rotation(a,float(psi)) for a,psi in zip(acceleration,yaw)])
    quats=np.asarray([rotation_to_quaternion(R) for R in rotations])
    for i in range(1,len(quats)):
        if np.dot(quats[i-1],quats[i])<0: quats[i]*=-1
    omega=np.zeros((len(times),3))
    for i,R in enumerate(rotations):
        Rdot=(rotations[min(i+1,len(times)-1)]-rotations[max(i-1,0)])/(DT if i in (0,len(times)-1) else 2*DT)
        skew=R.T@Rdot; omega[i]=[.5*(skew[2,1]-skew[1,2]),.5*(skew[0,2]-skew[2,0]),.5*(skew[1,0]-skew[0,1])]
    omegadot=np.gradient(omega,DT,axis=0,edge_order=2); motors=np.zeros((len(times),4)); mix=allocation()
    for i in range(len(times)):
        total=MASS*np.linalg.norm(acceleration[i]+np.asarray([0,0,G])); torque=INERTIA@omegadot[i]+np.cross(omega[i],INERTIA@omega[i]); motors[i]=np.linalg.solve(mix,np.r_[total,torque])
    if motors.min() < -1e-6 or motors.max() > 3.72e-8*2900**2+1e-6: raise RuntimeError(f"infeasible motor thrust range {motors.min()}..{motors.max()}")
    args.out.parent.mkdir(parents=True,exist_ok=True)
    fields=["t","x","y","z","vx","vy","vz","qw","qx","qy","qz","wx","wy","wz",*[f"motor_{i}_thrust_n" for i in range(4)]]
    with args.out.open("w",newline="") as f:
        w=csv.writer(f); w.writerow(fields)
        for i,t in enumerate(times): w.writerow([f"{t:.9f}",*position[i],*velocity[i],*quats[i],*omega[i],*motors[i]])
    print(f"wrote {args.out}: bank={args.bank_deg:g} deg, yaw={math.degrees(yaw[-1]):.1f} deg, motor={motors.min():.3f}..{motors.max():.3f} N")
    return 0


if __name__=="__main__": raise SystemExit(main())
