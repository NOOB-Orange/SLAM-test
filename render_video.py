"""Three-panel ORB-SLAM3 replay. Missing/foreign-map poses are never interpolated."""
from __future__ import annotations
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from export_map import dataset_frames, read_csv
from render_core import Camera, Pose, Model, GlobalView, first_person, label

STATE_NAMES = {-1: 'SYSTEM NOT READY', 0: 'NO IMAGES YET', 1: 'INITIALIZING',
               2: 'OK', 3: 'RECENTLY LOST', 4: 'LOST', 5: 'OK KLT'}


def render(dataset, replay, output, map_id=None, max_frames=None, width=1280, height=720):
    dataset, replay, output = Path(dataset), Path(replay), Path(output)
    if output.exists(): raise ValueError(f'Output already exists: {output}')
    if width < 640 or height < 360 or width % 4 or height % 4:
        raise ValueError('Output width/height must be >=640/360 and multiples of 4')
    summary = json.loads((replay/'summary.json').read_text())
    metadata = json.loads((dataset/'metadata.json').read_text())
    calibration = json.loads((dataset/'camera.json').read_text())
    if map_id is None: map_id = summary['selected_map']
    folder = replay/f'map_{map_id}'
    rows = read_csv(folder/'poses.csv')
    cloud = np.load(folder/'map.npz', allow_pickle=False)
    poses = [Pose(int(r['frame_index']), np.array([float(r[k]) for k in ['qw', 'qx', 'qy', 'qz']]),
                  np.array([float(r[k]) for k in ['tx', 'ty', 'tz']]), 1,
                  f"{int(r['frame_index']):06d}.jpg", np.empty((0, 3))) for r in rows]
    poses.sort(key=lambda p: p.id)
    by_index = {p.id: p for p in poses}
    states = {int(r['frame_index']): int(r['state']) for r in read_csv(replay/'frame_status.csv')}
    c = calibration
    camera = Camera(1, 'OPENCV' if c['k3'] == 0 else 'FULL_OPENCV', c['width'], c['height'],
                    np.array([c[k] for k in ['fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'p1', 'p2']]
                             + ([c['k3'], 0, 0, 0] if c['k3'] else []), float))
    model = Model({1: camera}, poses, cloud['ids'], cloud['xyz'], cloud['bgr'], cloud['birth_time'])
    cloud.close()
    view = GlobalView(model, width, height//2, 30, 35, 99.5)
    manifest = dataset_frames(dataset)
    if max_frames is not None:
        if max_frames <= 0: raise ValueError('max_frames must be positive')
        manifest = manifest[:max_frames]
    output.parent.mkdir(parents=True, exist_ok=True)
    partial = output.with_name(output.stem+'.partial.mp4')
    fps = metadata['source_reported_fps']
    writer = cv2.VideoWriter(str(partial), cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))
    if not writer.isOpened(): raise RuntimeError('MP4 writer could not open')
    segments, previous_index, current_segment = [], None, []
    cached_birth, cached_map = None, None
    point_births = np.sort(model.birth)
    successful, unavailable, written = 0, 0, 0
    try:
        for row in manifest:
            index, stamp = int(row['frame_index']), float(row['timestamp'])
            frame = cv2.imread(str(dataset/row['image_path']))
            if frame is None: raise ValueError(f'Cannot read frame {index}')
            active = model.birth <= stamp+1e-7
            birth_step = int(np.searchsorted(point_births, stamp+1e-7, side='right'))
            if birth_step != cached_birth:
                cached_map = view.map_image(model, active, 1)
                cached_birth = birth_step
            global_image = cached_map.copy()
            local = np.zeros((height//2, width//2, 3), np.uint8)
            pose = by_index.get(index)
            state = states.get(index, -1)
            visible = 0
            if pose is not None and state == 2:
                local, visible, _ = first_person(model, pose.R, pose.center, camera, frame,
                                                  width//2, height//2, active, 1)
                if previous_index is None or index != previous_index+1:
                    current_segment = []
                    segments.append(current_segment)
                current_segment.append(pose.center)
                previous_index = index
                successful += 1
            else:
                previous_index = None
                unavailable += 1
                message = STATE_NAMES.get(state, f'STATE {state}')
                if state == 2: message = f'OUTSIDE SELECTED MAP {map_id}'
                label(local, message, (12, height//4))
            # Do not connect trajectory across tracking failures or independent maps.
            for segment in segments:
                uv, depth = view.project(np.asarray(segment))
                if (depth > 0).all() and np.isfinite(uv).all():
                    cv2.polylines(global_image, [np.rint(uv).astype(np.int32)], False,
                                  (25, 225, 250), 2, cv2.LINE_AA)
            if pose is not None and state == 2:
                view.draw_motion(global_image, np.empty((0, 3)), pose.center, pose.R, camera)
            left = cv2.resize(frame, (width//2, height//2), interpolation=cv2.INTER_AREA)
            label(left, 'INPUT FRAME')
            label(local, 'FIRST-PERSON MAP - ORB-SLAM3')
            label(global_image, 'GLOBAL MAPPING PROGRESS - ORB-SLAM3')
            label(global_image, f'T+{stamp:05.1f}s  MAP {map_id}  POINTS {active.sum():,}',
                  (max(350, width-400), 22), .4)
            canvas = np.vstack((np.hstack((left, local)), global_image))
            cv2.line(canvas, (width//2, 0), (width//2, height//2), (200, 205, 195), 1)
            cv2.line(canvas, (0, height//2), (width-1, height//2), (200, 205, 195), 1)
            cv2.line(canvas, (16, height-9), (width-16, height-9), (45, 49, 42), 4)
            cv2.line(canvas, (16, height-9), (16+round((width-32)*(written+1)/len(manifest)), height-9),
                     (25, 225, 250), 4)
            writer.write(canvas)
            written += 1
            if written in {1, len(manifest)//2, len(manifest)}:
                cv2.imwrite(str(output.with_name(output.stem+f'_frame_{index:06d}.jpg')), canvas)
            if written % 150 == 0 or written == len(manifest):
                print(f'Render {written}/{len(manifest)} visible={visible} tracked={successful}', flush=True)
    finally:
        writer.release()
    cap = cv2.VideoCapture(str(partial))
    decoded, encoded_fps = 0, cap.get(cv2.CAP_PROP_FPS)
    while True:
        ok, frame = cap.read()
        if not ok: break
        if frame.shape[:2] != (height, width): raise RuntimeError('Wrong encoded dimensions')
        decoded += 1
    cap.release()
    if decoded != written or abs(encoded_fps-fps) > max(.001, fps*1e-4):
        raise RuntimeError('Output failed full decode/FPS verification')
    partial.replace(output)
    report = {'backend': 'ORB-SLAM3', 'map_id': map_id, 'frames': written, 'fps': encoded_fps,
              'valid_pose_frames': successful, 'missing_or_other_map_frames': unavailable,
              'points': len(model.xyz), 'trajectory_segments': len(segments),
              'interpolation_over_lost_frames': False, 'summary': summary}
    output.with_suffix('.report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(f'Saved {output}; valid pose coverage {successful}/{written}')
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for key in ['dataset', 'replay', 'output']: p.add_argument('--'+key, required=True)
    p.add_argument('--map-id', type=int)
    p.add_argument('--max-frames', type=int)
    p.add_argument('--width', type=int, default=1280)
    p.add_argument('--height', type=int, default=720)
    args = p.parse_args()
    render(args.dataset, args.replay, args.output, args.map_id, args.max_frames, args.width, args.height)
