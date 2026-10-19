"""Extract ALL input frames and write a matching ORB-SLAM3 camera configuration."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent


def camera_yaml(camera, fps):
    c = camera
    values = {
        'File.version': '"1.0"', 'Camera.type': '"PinHole"',
        'Camera1.fx': c['fx'], 'Camera1.fy': c['fy'], 'Camera1.cx': c['cx'], 'Camera1.cy': c['cy'],
        **{f'Camera1.{k}': c[k] for k in ['k1', 'k2', 'p1', 'p2', 'k3']},
        'Camera.fps': fps, 'Camera.RGB': 0, 'Camera.width': c['width'], 'Camera.height': c['height'],
        'ORBextractor.nFeatures': c['n_features'], 'ORBextractor.scaleFactor': c['scale_factor'],
        'ORBextractor.nLevels': c['n_levels'], 'ORBextractor.iniThFAST': c['ini_fast'],
        'ORBextractor.minThFAST': c['min_fast'],
        'Viewer.KeyFrameSize': 0.05, 'Viewer.KeyFrameLineWidth': 1.0,
        'Viewer.GraphLineWidth': 0.9, 'Viewer.PointSize': 2.0, 'Viewer.CameraSize': 0.08,
        'Viewer.CameraLineWidth': 3.0, 'Viewer.ViewpointX': 0.0, 'Viewer.ViewpointY': -0.7,
        'Viewer.ViewpointZ': -1.8, 'Viewer.ViewpointF': 500.0}
    return '%YAML:1.0\n---\n'+''.join(f'{key}: {value}\n' for key, value in values.items())


def prepare(video, output, calibration, width=1280, max_frames=None):
    video, output = Path(video).resolve(), Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError(f'Dataset already exists: {output}. Use a new directory.')
    camera = json.loads(Path(calibration).read_text(encoding='utf-8'))
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise ValueError(f'Cannot open {video}')
    fps = cap.get(cv2.CAP_PROP_FPS)
    if not np.isfinite(fps) or fps <= 0:
        raise ValueError('Video has invalid FPS')
    if width <= 0 or max_frames is not None and max_frames <= 0:
        raise ValueError('Width/frame limit must be positive')
    (output/'rgb').mkdir(parents=True, exist_ok=True)
    records, times = [], []
    index = 0
    try:
        while max_frames is None or index < max_frames:
            ok, frame = cap.read()
            if not ok:
                break
            if index == 0:
                h, w = frame.shape[:2]
                if abs(w/h-camera['width']/camera['height']) > .005:
                    raise ValueError('Video aspect differs from calibration; supply matching calibration')
                height = round(h*width/w)
                sx, sy = width/camera['width'], height/camera['height']
                for key in ['fx', 'cx']: camera[key] *= sx
                for key in ['fy', 'cy']: camera[key] *= sy
                camera['width'], camera['height'] = width, height
            if frame.shape[1::-1] != (width, height):
                frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
            timestamp = cap.get(cv2.CAP_PROP_POS_MSEC)/1000
            relative = f'rgb/{index:06d}.jpg'
            if not cv2.imwrite(str(output/relative), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                raise RuntimeError('Image write failed')
            records.append((index, timestamp, relative))
            times.append(timestamp)
            index += 1
    finally:
        cap.release()
    if not records:
        raise ValueError('Video has no decodable frames')
    reliable = bool(np.isfinite(times).all() and (len(times) == 1 or (np.diff(times) > 0).all()))
    if not reliable:
        records = [(i, i/fps, name) for i, _, name in records]
    effective_fps = 1/float(np.median(np.diff([r[1] for r in records]))) if len(records) > 1 else fps
    with (output/'frames.tsv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream, delimiter='\t', lineterminator='\n')
        writer.writerow(['frame_index', 'timestamp', 'image_path'])
        writer.writerows(records)
    (output/'camera.json').write_text(json.dumps(camera, indent=2), encoding='utf-8')
    (output/'camera.yaml').write_text(camera_yaml(camera, effective_fps), encoding='utf-8')
    sha = hashlib.sha256()
    with video.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''): sha.update(block)
    metadata = {'video': str(video), 'video_sha256': sha.hexdigest(), 'frames': len(records),
                'source_reported_fps': fps, 'timestamp_fps': effective_fps,
                'decoder_timestamps_used': reliable, 'width': width, 'height': height,
                'input_geometry': 'None: ORB-SLAM3 computes every pose and map point',
                'calibration': str(Path(calibration).resolve())}
    (output/'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    print(f'Prepared {len(records)} frames at {width}x{height}; timestamps {effective_fps:.6f} Hz')
    return metadata


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--video', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--calibration', default=ROOT/'configs/camera.json')
    p.add_argument('--width', type=int, default=1280)
    p.add_argument('--max-frames', type=int)
    args = p.parse_args()
    prepare(args.video, args.output, args.calibration, args.width, args.max_frames)
