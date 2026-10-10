#!/usr/bin/env python3
"""Render a logged MuJoCo rig run (results/capleg_rig_log.csv) as a lateral-view GIF - no OpenGL needed.

Each visual mesh is split into its parts; per frame every part is posed with MuJoCo forward kinematics,
projected onto the sagittal plane and drawn as a shaded silhouette (painter's order, lateral side on top).

    python3 render_side_view.py --strides 2 --fps 30
"""
import argparse
import csv
import os
import xml.etree.ElementTree as ET

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402
from PIL import Image  # noqa: E402
from scipy.spatial import ConvexHull  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def parts_from_model(model_path):
    """[(body_name, hull_vertices_in_body_frame (N,3), rgba)] for every connected part of every visual mesh."""
    root = ET.parse(model_path).getroot()
    mesh_file = {m.get('name'): m.get('file') for m in root.iter('mesh')}
    mat_rgba = {m.get('name'): [float(v) for v in m.get('rgba').split()] for m in root.iter('material')
                if m.get('rgba')}
    parts = []
    for body in root.iter('body'):
        for g in body.findall('geom'):
            if g.get('mesh') is None:
                continue
            mesh = trimesh.load(os.path.join(HERE, 'assets', mesh_file[g.get('mesh')]), force='mesh')
            rgba = mat_rgba.get(g.get('material'), [0.5, 0.5, 0.5, 1])
            for comp in mesh.split(only_watertight=False):
                if len(comp.vertices) < 4:
                    continue
                try:
                    hv = comp.convex_hull.vertices
                except Exception:
                    hv = comp.vertices
                parts.append((body.get('name'), np.asarray(hv), rgba))
    return parts


def shade(rgba, k):
    c = np.array(rgba[:3]) * k + (1 - k) * 0.08
    return (*np.clip(c, 0, 1), min(1.0, rgba[3] + 0.25))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', default=os.path.join(HERE, 'results', 'capleg_rig_log.csv'))
    ap.add_argument('--out', default=os.path.join(HERE, 'results', 'capleg_rig_side_view.gif'))
    ap.add_argument('--strides', type=float, default=2.0)
    ap.add_argument('--fps', type=float, default=30.0)
    ap.add_argument('--stride-time', type=float, default=1.2)
    args = ap.parse_args()
    model_path = os.path.join(HERE, 'capleg_rig.xml')
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)
    parts = parts_from_model(model_path)
    with open(args.log, encoding='utf-8', newline='') as fh:
        rows = list(csv.DictReader(fh))
    t_end = float(rows[-1]['t'])
    t0 = t_end - args.strides * args.stride_time
    times = np.arange(t0, t_end, 1.0 / args.fps)
    ts = np.array([float(r['t']) for r in rows])
    frames = []
    fig = plt.figure(figsize=(4.8, 6.4), dpi=110)
    for tf in times:
        r = rows[int(np.argmin(np.abs(ts - tf)))]
        d.qpos[:] = [float(v) for v in r['qpos'].split()]
        mujoco.mj_kinematics(m, d)
        hip_x = d.body('capleg_thigh').xpos[0]
        fig.clf()
        ax = fig.add_axes([0, 0, 1, 1])
        ax.set_xlim(hip_x - 0.55, hip_x + 0.55)
        ax.set_ylim(-0.08, 1.38)
        ax.set_aspect('equal')
        ax.axis('off')
        # treadmill belt with moving stripes
        ax.add_patch(Polygon([[hip_x - 1, -0.08], [hip_x + 1, -0.08], [hip_x + 1, 0], [hip_x - 1, 0]],
                             closed=True, fc='#2a2a2c', ec='none', zorder=0))
        for s in np.arange(np.floor((hip_x - 1) / 0.1) * 0.1, hip_x + 1, 0.1):
            ax.plot([s, s - 0.02], [0, -0.035], c='#8d8f93', lw=1.0, zorder=1)
        polys = []
        for bname, hv, rgba in parts:
            b = d.body(bname)
            w = hv @ b.xmat.reshape(3, 3).T + b.xpos
            p2 = w[:, [0, 2]]
            try:
                h = ConvexHull(p2)
                poly = p2[h.vertices]
            except Exception:
                continue
            polys.append((w[:, 1].mean(), poly, rgba))
        for depth, poly, rgba in sorted(polys, key=lambda x: -x[0]):
            k = 0.62 + 0.38 * np.clip((0.05 - depth) / 0.1, 0, 1)      # lateral parts brighter
            ax.add_patch(Polygon(poly, closed=True, fc=shade(rgba, k), ec=(0, 0, 0, 0.35), lw=0.4, zorder=2))
        # GRF arrow
        gz, gx = float(r['grf_z']), float(r['grf_x'])
        if gz > 20:
            heel = d.site('heel').xpos
            ball = d.site('ball').xpos
            x0 = heel[0] + (ball[0] - heel[0]) * np.clip(float(r['phase_true']) / 0.55, 0, 1)
            sc = 0.45 / (75 * 9.81)
            ax.annotate('', xy=(x0 + gx * sc, gz * sc), xytext=(x0, 0.0),
                        arrowprops=dict(arrowstyle='->', color='#1f6fd1', lw=2), zorder=5)
        ph = float(r['phase_true'])
        ax.text(hip_x - 0.52, 1.33, 'CapLeg  |  MuJoCo treadmill rig', fontsize=8.5, color='#111111', weight='bold')
        ax.text(hip_x - 0.52, 1.285, 'gait %3d%%  %s   knee %5.1f°   ankle %5.1f°' % (
            ph * 100, 'stance' if ph < 0.62 else 'swing ', np.degrees(float(r['knee'])),
            np.degrees(float(r['ankle']))), fontsize=7.5, color='#333333', family='monospace')
        fig.canvas.draw()
        img = np.asarray(fig.canvas.buffer_rgba())[..., :3]
        frames.append(Image.fromarray(img.copy()))
    plt.close(fig)
    frames[0].save(args.out, save_all=True, append_images=frames[1:], duration=int(1000 / args.fps), loop=0,
                   optimize=True)
    print('wrote', args.out, len(frames), 'frames')


if __name__ == '__main__':
    main()
