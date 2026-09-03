#!/usr/bin/env python3
"""Render a ROS map (and optionally a driven path) to PNG - no GUI, no RViz.

    ros2 run zmr_tools render_map -- map.yaml out.png [path.csv]

Uses matplotlib with the Agg backend, so it works over SSH on a headless
machine. This is the substitute for opening RViz to look at a result.
"""

from __future__ import annotations

import csv
import os
import sys

import matplotlib
matplotlib.use('Agg')          # headless: must be set before pyplot is imported
import matplotlib.pyplot as plt   # noqa: E402
import numpy as np                # noqa: E402
import yaml                       # noqa: E402


def read_pgm(path: str) -> np.ndarray:
    with open(path, 'rb') as fh:
        data = fh.read()

    def tokens(buf):
        i = 0
        while i < len(buf):
            if buf[i:i + 1] == b'#':
                while i < len(buf) and buf[i:i + 1] not in b'\r\n':
                    i += 1
            elif buf[i:i + 1].isspace():
                i += 1
            else:
                j = i
                while j < len(buf) and not buf[j:j + 1].isspace():
                    j += 1
                yield buf[i:j], j
                i = j

    it = tokens(data)
    magic, _ = next(it)
    w, _ = next(it)
    h, _ = next(it)
    _maxv, end = next(it)
    w, h = int(w), int(h)
    if magic == b'P5':
        return np.frombuffer(data[end + 1:end + 1 + w * h],
                             dtype=np.uint8).reshape(h, w)
    if magic == b'P2':
        rest = data[end:].split()
        return np.array([int(v) for v in rest[:w * h]], dtype=np.uint8).reshape(h, w)
    raise ValueError(f'unsupported PGM magic {magic!r}')


def render(map_yaml: str, out_png: str, path_csv: str | None = None,
           title: str | None = None) -> str:
    with open(map_yaml) as fh:
        meta = yaml.safe_load(fh)
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), img_path)
    img = read_pgm(img_path)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    h, w = img.shape
    extent = [ox, ox + w * res, oy, oy + h * res]

    fig, ax = plt.subplots(figsize=(11, 11 * h / max(w, 1)), dpi=140)
    ax.imshow(img, cmap='gray', vmin=0, vmax=255, origin='upper',
              extent=extent, interpolation='nearest')

    if path_csv and os.path.exists(path_csv):
        xs, ys, gxs, gys = [], [], [], []
        with open(path_csv) as fh:
            for row in csv.DictReader(fh):
                try:
                    xs.append(float(row['x']))
                    ys.append(float(row['y']))
                except (ValueError, KeyError):
                    continue
                try:
                    gx, gy = float(row['gt_x']), float(row['gt_y'])
                    if not (np.isnan(gx) or np.isnan(gy)):
                        gxs.append(gx)
                        gys.append(gy)
                except (ValueError, KeyError):
                    pass
        if gxs:
            ax.plot(gxs, gys, '-', color='#1a7f4b', lw=2.0,
                    label='ground truth', zorder=3)
        if xs:
            ax.plot(xs, ys, '-', color='#1f6feb', lw=1.6, alpha=0.9,
                    label='estimated (TF)', zorder=4)
            ax.plot(xs[0], ys[0], 'o', color='#1a7f4b', ms=9,
                    label='start', zorder=5)
            ax.plot(xs[-1], ys[-1], 's', color='#d1451b', ms=9,
                    label='end', zorder=5)
        if xs or gxs:
            ax.legend(loc='upper right', fontsize=9, framealpha=0.92)

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(title or os.path.basename(map_yaml), fontsize=11)
    ax.grid(alpha=0.18, lw=0.6)
    ax.set_aspect('equal')
    d = os.path.dirname(os.path.abspath(out_png))
    if d:
        os.makedirs(d, exist_ok=True)
    fig.savefig(out_png, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    return out_png


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    # ros2 launch appends "--ros-args -r __node:=..." after the positional
    # arguments. Filtering token by token leaves "-r" and "__node:=..." behind,
    # and they then get read as path_csv and title. Truncate instead.
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    argv = [a for a in argv if a != '--']
    if len(argv) < 2:
        print('usage: render_map <map.yaml> <out.png> [path.csv] [title]')
        return 2
    out = render(argv[0], argv[1],
                 argv[2] if len(argv) > 2 else None,
                 argv[3] if len(argv) > 3 else None)
    print(f'wrote {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
