"""Colorize ORB map points and package each independent atlas map separately."""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np


def read_csv(path):
    with Path(path).open(newline='', encoding='utf-8-sig') as stream:
        return list(csv.DictReader(stream))


def frame_key(timestamp):
    return round(float(timestamp), 7)


def dataset_frames(dataset):
    with (Path(dataset)/'frames.tsv').open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream, delimiter='\t'))


def export(dataset, raw, output):
    dataset, raw, output = Path(dataset), Path(raw), Path(output)
    if output.exists() and any(output.iterdir()):
        raise ValueError(f'Replay exists: {output}; use a fresh output directory')
    manifest = dataset_frames(dataset)
    by_time = {frame_key(r['timestamp']): r for r in manifest}
    status = {frame_key(r['timestamp']): int(r['state']) for r in read_csv(raw/'tracking.csv')}
    if len(by_time) != len(manifest) or set(status) != set(by_time):
        raise ValueError('Tracking/manifest counts or timestamps mismatch')
    maps = read_csv(raw/'maps.csv')
    points = read_csv(raw/'points.csv')
    observations = read_csv(raw/'observations.csv')
    poses = read_csv(raw/'poses.csv')
    if not maps or not points:
        raise ValueError('ORB-SLAM3 produced no usable map: inspect initialization and tracking log')
    output.mkdir(parents=True, exist_ok=True)
    with (output/'frame_status.csv').open('w', newline='') as stream:
        writer = csv.writer(stream, lineterminator='\n')
        writer.writerow(['frame_index', 'timestamp', 'state'])
        writer.writerows((row['frame_index'], row['timestamp'], status[frame_key(row['timestamp'])])
                         for row in manifest)
    by_point = {(int(r['map_id']), int(r['id'])): i for i, r in enumerate(points)}
    sums, counts = np.zeros((len(points), 3), float), np.zeros(len(points), int)
    groups = {}
    for obs in observations:
        key = frame_key(obs['timestamp'])
        if key not in by_time:
            raise ValueError('Observation timestamp missing from input manifest')
        groups.setdefault(key, []).append(obs)
    for key, obs_list in groups.items():
        image = cv2.imread(str(dataset/by_time[key]['image_path']))
        if image is None: raise ValueError('Cannot read keyframe image')
        for obs in obs_list:
            i = by_point.get((int(obs['map_id']), int(obs['point_id'])))
            if i is None: continue
            x, y = float(obs['u']), float(obs['v'])
            if not np.isfinite([x, y]).all() or not (0 <= x < image.shape[1] and 0 <= y < image.shape[0]):
                continue
            # Sub-pixel sampling in the original distorted image.
            color = cv2.getRectSubPix(image, (1, 1), (x, y))[0, 0]
            sums[i] += color
            counts[i] += 1
    if (counts == 0).any():
        raise ValueError(f'{(counts == 0).sum()} points have no valid color observation')
    bgr = np.rint(sums/counts[:, None]).astype(np.uint8)
    pose_by_map = {}
    for row in poses:
        key = frame_key(row['timestamp'])
        # RECENTLY_LOST may contain a propagated stale pose: accept only state OK (2).
        if int(row['lost']) or status.get(key) != 2: continue
        values = [float(row[k]) for k in ['qw', 'qx', 'qy', 'qz', 'tx', 'ty', 'tz']]
        if not np.isfinite(values).all(): continue
        pose_by_map.setdefault(int(row['map_id']), {})[key] = values
    summary = []
    pose_header = ['qw', 'qx', 'qy', 'qz', 'tx', 'ty', 'tz']
    for map_record in maps:
        map_id = int(map_record['map_id'])
        indices = [i for i, row in enumerate(points) if int(row['map_id']) == map_id]
        samples = pose_by_map.get(map_id, {})
        if not indices or len(samples) < 2: continue
        folder = output/f'map_{map_id}'
        folder.mkdir()
        xyz = np.array([[float(points[i][k]) for k in ['x', 'y', 'z']] for i in indices])
        ids = np.array([int(points[i]['id']) for i in indices], dtype=np.int64)
        birth = np.array([float(points[i]['birth_time']) for i in indices])
        np.savez_compressed(folder/'map.npz', ids=ids, xyz=xyz, bgr=bgr[indices], birth_time=birth)
        with (folder/'poses.csv').open('w', newline='') as stream:
            writer = csv.writer(stream, lineterminator='\n')
            writer.writerow(['frame_index', 'timestamp', *pose_header])
            for key in sorted(samples):
                frame = by_time[key]
                writer.writerow([frame['frame_index'], frame['timestamp'], *samples[key]])
        with (folder/'map.ply').open('w') as stream:
            stream.write('ply\nformat ascii 1.0\nelement vertex '+str(len(indices))+'\n'
                         'property float x\nproperty float y\nproperty float z\n'
                         'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
            for pos, color in zip(xyz, bgr[indices]):
                stream.write(' '.join(map(str, [*pos, *color[::-1].tolist()]))+'\n')
        summary.append({'map_id': map_id, 'points': len(indices), 'tracked_frames': len(samples),
                        'keyframes': int(map_record['keyframes']), 'folder': folder.name})
    if not summary: raise ValueError('No map has at least two valid tracked frames')
    selected = max(summary, key=lambda row: (row['tracked_frames'], row['points']))
    states, state_counts = np.unique(list(status.values()), return_counts=True)
    report = {'backend': 'UZ-SLAMLab/ORB_SLAM3', 'selected_map': selected['map_id'], 'maps': summary,
              'input_frames': len(manifest), 'tracking_states': dict(zip(map(str, states.tolist()), state_counts.tolist())),
              'selected_map_coverage': selected['tracked_frames']/len(manifest),
              'coordinate_convention': 'Tcw: world -> camera; map points use the SAME original map origin',
              'progress_semantics': 'Final optimized geometry revealed at earliest RETAINED keyframe observation; offline replay',
              'map_policy': 'Independent maps are exported separately and never overlaid'}
    (output/'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for key in ['dataset', 'raw', 'output']: p.add_argument('--'+key, required=True)
    args = p.parse_args()
    export(args.dataset, args.raw, args.output)
