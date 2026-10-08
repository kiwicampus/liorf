#!/usr/bin/env python3
"""End-to-end check for liorf_densify_map on synthetic data with known ground truth.

Builds a street-like scene (ground, two facades, pillars), a trajectory with a stop, weaving
turns, bumps and an in-place spin, then writes:
  <dir>/sim.bag    /livox/filtered PointCloud2 in the raw (pitched) livox_link frame, Livox
                   layout (x y z intensity tag line timestamp[float64 ns]); /livox/imu at 200 Hz
                   in the same raw frame, gyro with bias + noise
  <dir>/graph/     liorf dump: factor_graph.yaml + NNNNNN/{cloud.pcd,data}, keyframes every
                   0.5 m / 0.2 rad like saveFrame(), clouds = deskewed, every 3rd point, 0.25 m voxel
Then runs the tool in a few modes and reports point-to-surface error against the true scene and
per-scan pose error (scan_poses.txt) against the true trajectory, overall and per phase.

  python3 densify_map_synthetic.py [--dir /tmp/densify_sim] [--kf-noise 0.0]
"""
import argparse
import os
import struct
import subprocess
import sys

import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import Imu, PointCloud2, PointField

R_LEVEL_PITCH = 0.35
T0 = 1.7e9
SCAN_HZ = 10.0
PTS_PER_SCAN = 8000

# Scene: planes (normal, offset) with n.p = d, and axis-aligned boxes (min, max)
PLANES = [(np.array([0, 0, 1.0]), 0.0), (np.array([0, 1.0, 0]), 7.0), (np.array([0, 1.0, 0]), -7.0)]
BOXES = []
for x in np.arange(-10, 120, 7.0):
    for y in (-4.0, 4.0):
        BOXES.append((np.array([x - 0.3, y - 0.3, 0.0]), np.array([x + 0.3, y + 0.3, 3.0])))
    BOXES.append((np.array([x + 3.0, -6.5, 0.0]), np.array([x + 4.2, -5.5, 1.2])))  # benches/kiosks


def rot(axis, a):
    c, s = np.cos(a), np.sin(a)
    if axis == 'x':
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == 'y':
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_to_R(r, p, y):
    return rot('z', y) @ rot('y', p) @ rot('x', r)


def make_trajectory(duration=42.0, dt=1e-3):
    """Dense table of (t, x, y, z, roll, pitch, yaw) for the leveled lidar frame."""
    t = np.arange(0, duration + dt, dt)
    v = np.zeros_like(t)
    yaw_rate = np.zeros_like(t)
    for i, ti in enumerate(t):
        if ti < 1.0:
            v[i] = 1.2 * ti
        elif ti < 9.0:
            v[i] = 1.2
        elif ti < 10.0:
            v[i] = 1.2 * (10.0 - ti)
        elif ti < 16.0:
            v[i] = 0.0                      # stop: keyframe interpolation smears this
        elif ti < 17.0:
            v[i] = 1.2 * (ti - 16.0)
        elif ti < 30.0:
            v[i] = 1.4
            yaw_rate[i] = 0.35 * np.cos(2 * np.pi * (ti - 17.0) / 4.0) * 2 * np.pi / 4.0  # weave
        elif ti < 31.0:
            v[i] = 1.4 * (31.0 - ti)
        elif ti < 37.0:
            yaw_rate[i] = 1.0 * np.sin(np.pi * (ti - 31.0) / 3.0)   # spin in place, ~0.6 turn...
        else:
            v[i] = 1.0 * min(1.0, ti - 37.0)
    yaw = np.cumsum(yaw_rate) * dt
    x = np.cumsum(v * np.cos(yaw)) * dt
    y = np.cumsum(v * np.sin(yaw)) * dt
    bump = np.clip(v / 0.5, 0, 1)  # sidewalk bumps only while driving
    roll = 0.02 * np.sin(2 * np.pi * t / 1.7) * bump
    pitch = 0.025 * np.sin(2 * np.pi * t / 2.3) * bump
    z = 0.55 + 0.02 * np.sin(2 * np.pi * t / 1.1) * bump
    return t, np.stack([x, y, z, roll, pitch, yaw], axis=1)


class GT:
    def __init__(self):
        self.t, self.s = make_trajectory()

    def pose(self, t):
        t = np.atleast_1d(t)
        s = np.stack([np.interp(t, self.t, self.s[:, j]) for j in range(6)], axis=1)
        Rs = np.stack([rpy_to_R(*row[3:]) for row in s])
        return Rs, s[:, :3]


def so3_log(R):
    a = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if a < 1e-9:
        return np.zeros(3)
    return a / (2 * np.sin(a)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def raycast(o, d, max_range=60.0):
    """o: (N,3) origins, d: (N,3) unit dirs -> ranges (inf if no hit)."""
    best = np.full(len(o), np.inf)
    for n, dist in PLANES:
        den = d @ n
        with np.errstate(divide='ignore', invalid='ignore'):
            tt = (dist - o @ n) / den
        tt[(tt <= 0) | ~np.isfinite(tt)] = np.inf
        best = np.minimum(best, tt)
    for lo, hi in BOXES:
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (lo - o) / d
            t2 = (hi - o) / d
        tmin = np.nanmax(np.minimum(t1, t2), axis=1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=1)
        hit = (tmax >= tmin) & (tmin > 0)
        best = np.where(hit, np.minimum(best, tmin), best)
    best[best > max_range] = np.inf
    return best


def surface_distance(p):
    """Unsigned distance from points to the nearest scene surface."""
    best = np.full(len(p), np.inf)
    for n, dist in PLANES:
        best = np.minimum(best, np.abs(p @ n - dist))
    for lo, hi in BOXES:
        q = np.maximum(lo - p, p - hi)
        outside = np.linalg.norm(np.maximum(q, 0), axis=1)
        inside = np.minimum(np.max(q, axis=1), 0)
        best = np.minimum(best, np.abs(outside + inside))
    return best


def write_pcd(path, xyzi):
    xyzi = np.asarray(xyzi, dtype=np.float32)
    n = len(xyzi)
    with open(path, 'wb') as f:
        f.write((f"# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\n"
                 f"SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {n}\nHEIGHT 1\n"
                 f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n").encode())
        f.write(xyzi.tobytes())


def read_pcd_xyz(path):
    with open(path, 'rb') as f:
        n = 0
        while True:
            line = f.readline().decode()
            if line.startswith('POINTS'):
                n = int(line.split()[1])
            if line.startswith('DATA'):
                break
        return np.frombuffer(f.read(n * 16), dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)


def quat_from_R(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def simulate(out_dir, kf_noise, rng, ouster_order=False):
    gt = GT()
    R_lev = rot('y', R_LEVEL_PITCH)
    os.makedirs(os.path.join(out_dir, 'graph'), exist_ok=True)
    bag = rosbag.Bag(os.path.join(out_dir, 'sim.bag'), 'w')
    fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1), PointField('intensity', 12, PointField.FLOAT32, 1),
              PointField('tag', 16, PointField.UINT8, 1), PointField('line', 17, PointField.UINT8, 1),
              PointField('timestamp', 18, PointField.FLOAT64, 1)]
    rec = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'), ('tag', 'u1'),
                    ('line', 'u1'), ('timestamp', '<f8')])
    kfs = []
    last_kf = None
    n_scans = int((gt.t[-1] - 0.2) * SCAN_HZ)
    for s in range(n_scans):
        ts = s / SCAN_HZ
        tp = ts + np.sort(rng.uniform(0, 1.0 / SCAN_HZ, PTS_PER_SCAN))
        tp[0] = ts
        az = rng.uniform(0, 2 * np.pi, PTS_PER_SCAN)
        el = np.radians(rng.uniform(-7, 52, PTS_PER_SCAN))
        d_raw = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=1)
        d_lev = d_raw @ R_lev.T
        Rs, ps = gt.pose(tp)
        d_w = np.einsum('nij,nj->ni', Rs, d_lev)
        r = raycast(ps, d_w)
        ok = np.isfinite(r)
        r = r + rng.normal(0, 0.01, len(r))
        p_raw = d_raw * r[:, None]
        p_lev = d_lev * r[:, None]
        arr = np.zeros(ok.sum(), dtype=rec)
        arr['x'], arr['y'], arr['z'] = p_raw[ok, 0], p_raw[ok, 1], p_raw[ok, 2]
        arr['intensity'] = 50.0
        arr['line'] = rng.integers(0, 4, ok.sum())
        arr['timestamp'] = (T0 + tp[ok]) * 1e9
        stamp_t = T0 + tp[ok][0]  # header stamp = earliest point, also after reordering
        if ouster_order:
            # organized-Ouster-like layout: the array starts at a fixed azimuth, so point 0 is ~97% of
            # the way through the sweep and the earliest point sits near the end of the array
            k = int(0.97 * len(arr))
            arr = np.concatenate([arr[k:], arr[:k]])
        msg = PointCloud2()
        msg.header.stamp = rospy.Time.from_sec(stamp_t)
        msg.header.frame_id = 'livox_link'
        msg.height, msg.width = 1, int(ok.sum())
        msg.fields, msg.is_bigendian, msg.point_step = fields, False, rec.itemsize
        msg.row_step, msg.is_dense, msg.data = rec.itemsize * msg.width, True, arr.tobytes()
        bag.write('/livox/filtered', msg, msg.header.stamp)

        # liorf-style keyframe selection on the pose at the scan stamp (= first point's time)
        first = int(np.argmax(ok))
        R0, p0 = Rs[first], ps[first]
        if last_kf is not None:
            dR = last_kf[0].T @ R0
            ang = np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))
            if np.linalg.norm(p0 - last_kf[1]) < 0.5 and ang < 0.2:
                continue
        last_kf = (R0, p0)
        # Keyframe cloud: deskewed to the scan-start frame with GT, every 3rd point, 0.25 voxel
        pw = np.einsum('nij,nj->ni', Rs[ok], p_lev[ok]) + ps[ok]
        pl = (pw - p0) @ R0
        pl = pl[::3]
        _, keep = np.unique(np.floor(pl / 0.25).astype(np.int64), axis=0, return_index=True)
        pl = pl[np.sort(keep)]
        Rn, pn = R0, p0
        if kf_noise > 0:
            Rn = R0 @ rpy_to_R(*rng.normal(0, np.radians(0.1) * kf_noise / 0.01, 3))
            pn = p0 + rng.normal(0, kf_noise, 3)
        kfs.append((float(msg.header.stamp.to_sec()), Rn, pn))
        d = os.path.join(out_dir, 'graph', '%06d' % (len(kfs) - 1))
        os.makedirs(d, exist_ok=True)
        write_pcd(os.path.join(d, 'cloud.pcd'), np.hstack([pl, np.full((len(pl), 1), 50.0)]))

    # IMU: body rates of the leveled frame, expressed in the raw sensor frame (w_lev = R_lev w_raw)
    bias = np.array([0.004, -0.003, 0.005])
    for ti in np.arange(0.0, gt.t[-1] - 0.01, 1 / 200.0):
        (Ra, Rb), _ = gt.pose(np.array([ti, ti + 1e-3]))
        w_lev = so3_log(Ra.T @ Rb) / 1e-3
        w_raw = R_lev.T @ w_lev + bias + rng.normal(0, 0.002, 3)
        m = Imu()
        m.header.stamp = rospy.Time.from_sec(T0 + ti)
        m.header.frame_id = 'livox_link'
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = w_raw
        m.linear_acceleration.z = 1.0
        bag.write('/livox/imu', m, m.header.stamp)
    bag.close()

    with open(os.path.join(out_dir, 'graph', 'factor_graph.yaml'), 'w') as f:
        f.write(f"metadata:\n  total_keyframes: {len(kfs)}\n  total_factors: 1\nvertices:\n")
        for i, (t, R, p) in enumerate(kfs):
            q = quat_from_R(R)
            f.write(f"  - id: {i}\n    translation: {{x: {p[0]:.17g}, y: {p[1]:.17g}, z: {p[2]:.17g}}}\n"
                    f"    rotation: {{x: {q[0]:.17g}, y: {q[1]:.17g}, z: {q[2]:.17g}, w: {q[3]:.17g}}}\n"
                    f"    timestamp: {t:.17g}\n")
        f.write("factors:\n  - type: PriorFactor\n    key: 0\n")
    for i, (t, R, p) in enumerate(kfs):
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, p
        sec = int(t)
        with open(os.path.join(out_dir, 'graph', '%06d' % i, 'data'), 'w') as f:
            f.write(f"stamp {sec} {int(round((t - sec) * 1e9))}\nestimate\n")
            f.write('\n'.join(' '.join(f'{v:.17g}' for v in row) for row in T) + '\n')
    with open(os.path.join(out_dir, 'graph', 'graph.g2o'), 'w') as f:
        for i in range(len(kfs)):
            f.write(f"VERTEX_SE3:QUAT {i} 0 0 0 0 0 0 1\n")
    print(f"simulated {n_scans} scans, {len(kfs)} keyframes -> {out_dir}")


PHASES = [('cruise', 0, 9.5), ('stop', 9.5, 16.5), ('weave', 16.5, 30.5), ('spin', 30.5, 37), ('exit', 37, 99)]


def evaluate(out):
    tiles = os.path.join(out, 'tiles')
    pts = np.concatenate([read_pcd_xyz(os.path.join(tiles, f)) for f in sorted(os.listdir(tiles))])
    e = surface_distance(pts)
    return len(pts), np.median(e), np.percentile(e, 95), np.percentile(e, 99), np.mean(e > 0.05)


def pose_errors(out, gt):
    """Per-scan (t, translation error m, rotation error deg) from scan_poses.txt."""
    P = np.loadtxt(os.path.join(out, 'scan_poses.txt'), ndmin=2)
    t = P[:, 0] - T0
    Rg, pg = gt.pose(t)
    et = np.linalg.norm(P[:, 1:4] - pg, axis=1)
    er = []
    for i, (x, y, z, w) in enumerate(P[:, 4:8]):
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        er.append(np.degrees(np.linalg.norm(so3_log(Rg[i].T @ R))))
    return t, et, np.array(er)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', default='/tmp/densify_sim')
    ap.add_argument('--kf-noise', type=float, default=0.0, help='keyframe pose noise, m (rot scaled 0.1deg/cm)')
    ap.add_argument('--tool', default='/workspace/mapping_ws/devel/lib/liorf/liorf_densify_map')
    ap.add_argument('--skip-sim', action='store_true')
    ap.add_argument('--ouster-order', action='store_true',
                    help='reorder each scan so point 0 is the latest (Ouster-style), header stamp stays the earliest')
    a = ap.parse_args()
    if not a.skip_sim:
        simulate(a.dir, a.kf_noise, np.random.default_rng(0), a.ouster_order)

    graph, bag = os.path.join(a.dir, 'graph'), os.path.join(a.dir, 'sim.bag')
    runs = [
        ('refine (default)', []),
        ('interpolation only', ['--no-refine']),
        ('refine, no imu', ['--no-imu']),
        ('data pose source', ['--pose-source', 'data']),
        ('wrong leveling (sanity)', ['--level-rpy', '0', '0', '0', '--no-refine']),
    ]
    kf_pts = np.concatenate([read_pcd_xyz(os.path.join(graph, d, 'cloud.pcd'))
                             for d in sorted(os.listdir(graph)) if d.isdigit()])
    print(f"\nkeyframe clouds only (what cloudGlobal.pcd holds): {len(kf_pts)} points")
    print(f"\n{'mode':28s} {'points':>9s} {'median':>8s} {'p95':>8s} {'p99':>8s} {'>5cm':>7s}"
          f" | pose p95 {'trans':>7s} {'rot':>6s}  per-phase trans p95 (" + ' '.join(p[0] for p in PHASES) + ")")
    results = {}
    gt = GT()
    for name, extra in runs:
        out = os.path.join(a.dir, 'out_' + name.split()[0])
        cmd = [a.tool, '--graph', graph, '--bag', bag, '--out', out, '--voxel', '0.02'] + extra
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout[-3000:], r.stderr[-3000:])
            sys.exit(f"{name}: tool failed")
        with open(out + '.log', 'w') as f:
            f.write(r.stdout)
        n, med, p95, p99, frac = evaluate(out)
        t, et, er = pose_errors(out, gt)
        results[name] = (med, p95, frac, np.percentile(et, 95))
        phase = ' '.join(f"{np.percentile(et[(t >= a) & (t < b)], 95) * 100:5.1f}" if np.any((t >= a) & (t < b)) else '    -'
                         for _, a, b in PHASES)
        print(f"{name:28s} {n:9d} {med * 100:7.2f}c {p95 * 100:7.2f}c {p99 * 100:7.2f}c {frac * 100:6.2f}%"
              f" |          {np.percentile(et, 95) * 100:6.2f}c {np.percentile(er, 95):5.2f}d  {phase}")

    ok = results['refine (default)'][1] < 0.04 and results['wrong leveling (sanity)'][1] > 0.10
    print("\nPASS" if ok else "\nFAIL")
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
