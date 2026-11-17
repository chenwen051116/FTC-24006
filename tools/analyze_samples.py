#!/usr/bin/env python3
"""analyze_samples.py -- spread, conditioning, and residual structure of a
LLCalibration CSV. Usage: python analyze_samples.py <csv>"""
import sys
import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else "llcal_samples.csv"
d = np.genfromtxt(path, delimiter=',', names=True)
ox, oy, oyaw = d['odo_x'], d['odo_y'], d['odo_yaw']
lx, ly = d['ll_x'], d['ll_y']
n = len(ox)
M = 39.3701

print(f"n = {n}")
print(f"odo_x: {ox.min() * M:6.1f} .. {ox.max() * M:6.1f} in   std {ox.std() * M:5.1f}")
print(f"odo_y: {oy.min() * M:6.1f} .. {oy.max() * M:6.1f} in   std {oy.std() * M:5.1f}")
print(f"yaw  : {np.degrees(oyaw).min():6.1f} .. {np.degrees(oyaw).max():6.1f} deg")
print(f"corr(odo_x, odo_y) = {np.corrcoef(ox, oy)[0, 1]:+.2f}   (near +-1 kills kx/ky separation)")

A = np.zeros((2 * n, 4)); b = np.zeros(2 * n)
for j in range(n):
    A[2 * j] = [ox[j], 0, 1, 0]; b[2 * j] = lx[j]
    A[2 * j + 1] = [0, oy[j], 0, 1]; b[2 * j + 1] = ly[j]
print(f"scale design cond = {np.linalg.cond(A):.1f}   (<~50 means kx/ky trustworthy)")
p = np.linalg.lstsq(A, b, rcond=None)[0]
res = np.array([np.hypot(p[0] * ox[j] + p[2] - lx[j],
                          p[1] * oy[j] + p[3] - ly[j]) for j in range(n)]) * M
print(f"\nresidual (in) vs heading per sample:")
for j in range(n):
    print(f"  s{j + 1:2d}: yaw {np.degrees(oyaw[j]):7.1f} deg   resid {res[j]:5.2f} in")

c1 = np.corrcoef(res, np.abs(np.cos(oyaw)))[0, 1]
c2 = np.corrcoef(res, np.abs(np.sin(oyaw)))[0, 1]
print(f"\ncorr(resid, |cos yaw|) = {c1:+.2f}   corr(resid, |sin yaw|) = {c2:+.2f}")
print("(strong = robot-frame/pod effect; weak = world-frame or position effect)")
