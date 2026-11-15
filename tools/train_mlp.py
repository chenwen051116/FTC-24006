#!/usr/bin/env python3
"""
train_mlp.py -- Limelight-reading -> odometry-xy regression: does the MLP beat
the traditional rigid SE(3) transform?

Usage:
    python train_mlp.py <csv> [--save out.json]
    e.g. python train_mlp.py tools/llcal_samples.csv --save tools/mlp_red.json

The CSV comes from the LLCalibration opmode (dpad_up writes
/sdcard/FIRST/llcal_samples.csv on the Robot Controller; pull it with adb).

Direction (v2): the Limelight reading is the INPUT, the Pinpoint odometry xy
is the TARGET. Rationale: the encoder/gyro yaw is always trusted -- MegaTag2
even locks its yaw to the heading we feed it -- so only xy ever needs
correcting, and the full 6D LL pose (x, y, z, roll, pitch, yaw) carries the
viewing geometry that drives the LL's xy bias. The net learns: given
everything the camera reports, which xy is odo-consistent?

Models compared on held-out samples (leave-one-out, xy error in inches):
    SE(3):  Odo = T' * LL      (traditional rigid transform, same direction)
    MLP:    LL 6D -> odo (x, y)

Inputs are standardized (mean/std recorded in the artifact; the generated
Java RedMlp.predict applies the same standardization). With MegaTag2 the
z/roll/pitch readings are structurally 0 -- the inputs are kept anyway so
the same net is ready for MT1 / full-3D data.

Only dependency: numpy.
"""

import datetime
import json
import sys

import numpy as np

H = 6           # hidden units per layer
HIDDEN_LAYERS = 3  # tanh hidden layers between input and linear output
EPOCHS = 4000   # full-batch epochs (tiny data, converges fast)
LR = 2e-3
WD = 1e-4       # weight decay -- the main overfitting defense with tiny N
M_TO_IN = 39.3700787402


# ------------------------------------------------------------------ rotations
def r_x(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def r_y(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def r_z(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_to_R(roll, pitch, yaw):
    """Same convention as the Java solve: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    return r_z(yaw) @ r_y(pitch) @ r_x(roll)


def rot_to_quat(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-6:
        q = np.array([w,
                      (R[2, 1] - R[1, 2]) / (4 * w),
                      (R[0, 2] - R[2, 0]) / (4 * w),
                      (R[1, 0] - R[0, 1]) / (4 * w)])
    else:  # 180-degree rotation
        q = np.array([0.0,
                      np.sqrt(max(0.0, (1 + R[0, 0]) / 2)),
                      np.sqrt(max(0.0, (1 + R[1, 1]) / 2)) * np.sign(R[0, 1] or 1),
                      np.sqrt(max(0.0, (1 + R[2, 2]) / 2)) * np.sign(R[0, 2] or 1)])
    return q / np.linalg.norm(q)


def quat_to_rot(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rot_angle_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ------------------------------------------------------------------ SE(3) fit
def fit_se3(srcT, srcR, dstT, dstR):
    """Mean of per-sample T_i = Dst_i * Src_i^-1 (a rigid src->dst transform)."""
    tTs, quats = [], []
    for i in range(len(srcT)):
        invR = srcR[i].T
        invT = invR @ (-srcT[i])
        tR = dstR[i] @ invR
        tT = dstR[i] @ invT + dstT[i]
        q = rot_to_quat(tR)
        if quats and np.dot(q, quats[0]) < 0:
            q = -q
        tTs.append(tT)
        quats.append(q)
    meanT = np.mean(tTs, axis=0)
    qm = np.mean(quats, axis=0)
    return meanT, quat_to_rot(qm / np.linalg.norm(qm))


def se3_errors(meanT, meanR, srcT, srcR, dstT, dstR):
    """Per-sample (translation meters, rotation degrees) error of T*Src vs Dst."""
    eT, eR = [], []
    for i in range(len(srcT)):
        pR = meanR @ srcR[i]
        pT = meanR @ srcT[i] + meanT
        dR = dstR[i] @ pR.T
        dT = dstR[i] @ (pR.T @ (-pT)) + dstT[i]
        eT.append(np.linalg.norm(dT))
        eR.append(rot_angle_deg(dR))
    return np.array(eT), np.array(eR)


# ------------------------------------------------------------------ MLP
def init_net(rng, n_in, n_out):
    sizes = [n_in] + [H] * HIDDEN_LAYERS + [n_out]
    Ws = [rng.normal(0, np.sqrt(2 / (a + b)), (a, b))
          for a, b in zip(sizes[:-1], sizes[1:])]
    bs = [np.zeros(b) for b in sizes[1:]]
    return Ws, bs


def forward(Ws, bs, X):
    acts = [X]
    for j in range(len(Ws) - 1):
        acts.append(np.tanh(acts[-1] @ Ws[j] + bs[j]))
    out = acts[-1] @ Ws[-1] + bs[-1]
    return acts, out


def train_mlp(X, Y, seed):
    rng = np.random.default_rng(seed)
    Ws, bs = init_net(rng, X.shape[1], Y.shape[1])
    mW = [np.zeros_like(W) for W in Ws]; vW = [np.zeros_like(W) for W in Ws]
    mb = [np.zeros_like(b) for b in bs]; vb = [np.zeros_like(b) for b in bs]
    b1, b2, eps = 0.9, 0.999, 1e-8
    n = len(X)
    for t in range(1, EPOCHS + 1):
        acts, out = forward(Ws, bs, X)
        delta = (2.0 / n) * (out - Y)                    # dL/d(pre-output)
        gW = [None] * len(Ws); gB = [None] * len(bs)
        gW[-1] = acts[-1].T @ delta + WD * Ws[-1]
        gB[-1] = delta.sum(0)
        for j in range(len(Ws) - 2, -1, -1):
            delta = (delta @ Ws[j + 1].T) * (1 - acts[j + 1] ** 2)
            gW[j] = acts[j].T @ delta + WD * Ws[j]
            gB[j] = delta.sum(0)
        for j in range(len(Ws)):                          # Adam step
            mW[j] = b1 * mW[j] + (1 - b1) * gW[j]
            vW[j] = b2 * vW[j] + (1 - b2) * gW[j] ** 2
            mb[j] = b1 * mb[j] + (1 - b1) * gB[j]
            vb[j] = b2 * vb[j] + (1 - b2) * gB[j] ** 2
            Ws[j] -= LR * (mW[j] / (1 - b1 ** t)) / (np.sqrt(vW[j] / (1 - b2 ** t)) + eps)
            bs[j] -= LR * (mb[j] / (1 - b1 ** t)) / (np.sqrt(vb[j] / (1 - b2 ** t)) + eps)
    return Ws, bs


# ------------------------------------------------------------------ main
def main(path, out_path=None):
    data = np.genfromtxt(path, delimiter=',', names=True)
    n = len(data)
    if n < 4:
        print(f"only {n} samples -- need at least 4 (8+ recommended)")
        return

    ox, oy, oyaw = data['odo_x'], data['odo_y'], data['odo_yaw']
    lx, ly = data['ll_x'], data['ll_y']
    lz, lr, lp = data['ll_z'], data['ll_roll'], data['ll_pitch']
    # wrapped to +-PI: matches what the runtime feeds the net
    lyaw = wrap_pi(data['ll_yaw'])

    odoT = [np.array([ox[i], oy[i], 0.0]) for i in range(n)]
    odoR = [r_z(oyaw[i]) for i in range(n)]
    llT = [np.array([lx[i], ly[i], lz[i]]) for i in range(n)]
    llR = [rpy_to_R(lr[i], lp[i], lyaw[i]) for i in range(n)]

    # ---------- traditional SE(3), LL -> odo (the runtime baseline) ----------
    tT, tR = fit_se3(llT, llR, odoT, odoR)
    eT, eR = se3_errors(tT, tR, llT, llR, odoT, odoR)

    # reference solve in the other direction (stored in the artifact)
    rT, rR = fit_se3(odoT, odoR, llT, llR)
    eT2, _ = se3_errors(rT, rR, odoT, odoR, llT, llR)

    print(f"=== {n} samples ===")
    print(f"SE(3) LL->odo   train RMS: {np.sqrt(np.mean(eT**2)) * M_TO_IN:6.2f} in"
          f"   rot RMS {np.sqrt(np.mean(eR**2)):5.2f} deg")
    print(f"SE(3) odo->LL   train RMS: {np.sqrt(np.mean(eT2**2)) * M_TO_IN:6.2f} in   (reference)")
    print("SE(3) LL->odo T' (odo <- ll), row-major meters:")
    for row in np.hstack([tR, tT[:, None]]):
        print("   " + "  ".join(f"{v:8.4f}" for v in row))
    print("   [0 0 0 1]")

    # ---------- MLP: LL 6D -> odo xy ----------
    Xin = np.stack([lx, ly, lz, lr, lp, lyaw / np.pi], axis=1)
    mu = Xin.mean(axis=0)
    sd = Xin.std(axis=0)
    sd[sd < 1e-9] = 1.0          # constant columns (MT2 z/roll/pitch) -> no-op
    Xs = (Xin - mu) / sd
    Y = np.stack([ox, oy], axis=1)

    # ---------- leave-one-out: the honest comparison ----------
    loo_se3, loo_mlp = [], []
    for i in range(n):
        keep = [j for j in range(n) if j != i]
        # SE(3) LL->odo, fit on the others
        T, R = fit_se3([llT[j] for j in keep], [llR[j] for j in keep],
                       [odoT[j] for j in keep], [odoR[j] for j in keep])
        et, _ = se3_errors(T, R, [llT[i]], [llR[i]], [odoT[i]], [odoR[i]])
        loo_se3.append(et[0])
        # MLP (fresh init every fold so it can't carry information over)
        Ws, bs = train_mlp(Xs[keep], Y[keep], seed=i)
        _, out = forward(Ws, bs, Xs[i:i + 1])
        loo_mlp.append(np.hypot(out[0, 0] - ox[i], out[0, 1] - oy[i]))

    loo_se3 = np.array(loo_se3) * M_TO_IN
    loo_mlp = np.array(loo_mlp) * M_TO_IN

    print(f"\n=== held-out (leave-one-out) xy error, inches ===")
    print(f"            median   mean    max")
    print(f"SE(3):      {np.median(loo_se3):6.2f}  {np.mean(loo_se3):6.2f}  {np.max(loo_se3):6.2f}")
    print(f"MLP:        {np.median(loo_mlp):6.2f}  {np.mean(loo_mlp):6.2f}  {np.max(loo_mlp):6.2f}")

    m_se3, m_ml = np.median(loo_se3), np.median(loo_mlp)
    if m_ml < 0.7 * m_se3:
        print("\n>> MLP clearly ahead of the rigid transform -- the LL bias has")
        print(">> non-rigid structure (viewing geometry) the net is picking up.")
    else:
        print("\n>> Rigid transform holds; the residual is mostly noise/drift.")

    # ---------------- optional: record the model ----------------
    if out_path:
        seed = 1234
        Ws, bs = train_mlp(Xs, Y, seed)
        _, out_all = forward(Ws, bs, Xs)
        mlp_rms = np.sqrt(np.mean(np.sum((out_all - Y) ** 2, axis=1))) * M_TO_IN
        t_yaw = float(np.arctan2(rR[1, 0], rR[0, 0]))
        artifact = {
            'note': ('RED-side specific: trained against the RED Hive field map '
                     '(tags 30-37). Not valid for blue.'),
            'caveat': (f'Trained on {n} samples. LL inputs span '
                       f'{(lx.max() - lx.min()) * M_TO_IN:.0f} x {(ly.max() - ly.min()) * M_TO_IN:.0f} in '
                       f'of the field, odo targets span '
                       f'{(ox.max() - ox.min()) * M_TO_IN:.0f} x {(oy.max() - oy.min()) * M_TO_IN:.0f} in, '
                       f'encoder yaw {np.degrees(oyaw.min()):.0f}-{np.degrees(oyaw.max()):.0f} deg. '
                       'Local model -- do not trust it far outside that region; '
                       'retrain with field-wide samples before real use. '
                       'MT2 zeroes ll_z/ll_roll/ll_pitch, so those inputs carry no '
                       'signal until MT1/full-3D data is used.'),
            'created': datetime.date.today().isoformat(),
            'model': (f'MLP {"-".join(["6"] + [str(H)] * HIDDEN_LAYERS + ["2"])} tanh, '
                      f'Adam lr={LR} wd={WD}, {EPOCHS} epochs, seed {seed}'),
            'inputs': ('ll_x, ll_y, ll_z (meters), ll_roll, ll_pitch (radians), '
                       'll_yaw (radians, wrapped to +-PI). Standardized inside '
                       'predict() with input_mean/input_std.'),
            'outputs': ('odo_x, odo_y (meters, calibration-session odo frame). '
                        'Yaw is NEVER predicted -- always take it from the encoders.'),
            'input_mean': [float(v) for v in mu],
            'input_std': [float(v) for v in sd],
            'train_rms_in': {'mlp': round(float(mlp_rms), 2),
                             'se3_ll_to_odo': round(float(np.sqrt(np.mean(eT ** 2)) * M_TO_IN), 2)},
            'layers': [{'W': W.tolist(), 'b': b.tolist()} for W, b in zip(Ws, bs)],
            'se3_reference': {'direction': 'odo->field (field = T * odo)',
                              'T_rotation_rowmajor': rR.tolist(),
                              'T_translation_m': rT.tolist(),
                              't_yaw_rad': t_yaw},
            'training_samples': [[float(v) for v in
                                  (ox[i], oy[i], oyaw[i], lx[i], ly[i], lz[i],
                                   lr[i], lp[i], lyaw[i])] for i in range(n)],
        }
        with open(out_path, 'w') as fjson:
            json.dump(artifact, fjson, indent=1)
        print(f"\nrecorded model -> {out_path}   (train RMS {mlp_rms:.2f} in)")


if __name__ == '__main__':
    if len(sys.argv) < 2 or sys.argv[1].startswith('-'):
        print(__doc__)
        sys.exit(1)
    out = None
    if len(sys.argv) > 2 and sys.argv[2] == '--save':
        out = sys.argv[3] if len(sys.argv) > 3 else 'mlp_model.json'
    main(sys.argv[1], out)
