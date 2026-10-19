"""Local regression tests use SYNTHETIC exports, not fabricated ORB tracking results."""
import csv
import importlib.util
import json
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from prepare_video import prepare
from export_map import export, dataset_frames
from render_video import render
from render_core import q_to_R, splat


def write_csv(path, header, rows):
    with Path(path).open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def fixture(root):
    source = root/'source.mp4'
    writer = cv2.VideoWriter(str(source), cv2.VideoWriter_fourcc(*'mp4v'), 12., (320, 180))
    if not writer.isOpened(): raise RuntimeError('Test video codec unavailable')
    for i in range(7):
        image = np.full((180, 320, 3), [30, 40, 210], np.uint8)
        writer.write(image)
    writer.release()
    dataset = root/'dataset'
    prepare(source, dataset, ROOT/'configs/camera.json', width=320)
    raw = root/'raw'
    raw.mkdir()
    times = [float(row['timestamp']) for row in dataset_frames(dataset)]
    write_csv(raw/'tracking.csv', ['frame_index', 'timestamp', 'state', 'tracking_ms'],
              [(i, t, state, 10) for i, (t, state) in enumerate(zip(times, [1, 2, 2, 4, 2, 2, 2]))])
    write_csv(raw/'maps.csv', ['map_id', 'keyframes', 'points'], [(0, 3, 55), (1, 2, 55)])
    x, y = np.meshgrid(np.linspace(-1, 1, 11), np.linspace(-.5, .5, 5))
    points = []
    observations = []
    for map_id in [0, 1]:
        first = 1 if map_id == 0 else 5
        for i, (xx, yy) in enumerate(zip(x.ravel(), y.ravel())):
            points.append([i, map_id, xx, yy, 3., times[first], 2])
            observations.append([i, map_id, times[first], 160, 90])
    write_csv(raw/'points.csv', ['id', 'map_id', 'x', 'y', 'z', 'birth_time', 'observations'], points)
    write_csv(raw/'observations.csv', ['point_id', 'map_id', 'timestamp', 'u', 'v'], observations)
    poses = [[times[i], 0 if i < 5 else 1, 1, 0, 0, 0, -.02*i, 0, 0, 0] for i in [1, 2, 3, 4, 5, 6]]
    # Include a plausible but INVALID state-4 pose: exporter must remove it.
    write_csv(raw/'poses.csv', ['timestamp', 'map_id', 'qw', 'qx', 'qy', 'qz', 'tx', 'ty', 'tz', 'lost'], poses)
    return dataset, raw


class PipelineTests(unittest.TestCase):
    def test_camera_scaling_and_all_frame_timestamps(self):
        with tempfile.TemporaryDirectory() as td:
            dataset, _ = fixture(Path(td))
            camera = json.loads((dataset/'camera.json').read_text())
            self.assertAlmostEqual(camera['fx'], 559.94053152396134/4)
            self.assertEqual((camera['width'], camera['height']), (320, 180))
            manifest = dataset_frames(dataset)
            self.assertEqual(len(manifest), 7)
            self.assertEqual([int(row['frame_index']) for row in manifest], list(range(7)))
            np.testing.assert_allclose([float(row['timestamp']) for row in manifest], np.arange(7)/12)
            fs = cv2.FileStorage(str(dataset/'camera.yaml'), cv2.FILE_STORAGE_READ)
            self.assertTrue(fs.isOpened())
            self.assertEqual(fs.getNode('Camera.RGB').real(), 0)
            self.assertAlmostEqual(fs.getNode('Camera1.fx').real(), camera['fx'])
            fs.release()

    def test_separate_maps_color_and_tracking_state_filter(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            dataset, raw = fixture(root)
            report = export(dataset, raw, root/'replay')
            self.assertEqual(report['selected_map'], 0)
            self.assertEqual(len(report['maps']), 2)
            self.assertEqual(report['maps'][0]['tracked_frames'], 3)
            cloud = np.load(root/'replay/map_0/map.npz')
            self.assertEqual(len(cloud['xyz']), 55)
            self.assertTrue(np.all(np.abs(cloud['bgr'].astype(float)-[30, 40, 210]) < 8))
            cloud.close()
            self.assertTrue((root/'replay/map_1/map.ply').exists())

    def test_render_never_bridges_lost_frames(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            dataset, raw = fixture(root)
            export(dataset, raw, root/'replay')
            report = render(dataset, root/'replay', root/'demo.mp4', width=640, height=360)
            self.assertEqual(report['frames'], 7)
            self.assertEqual(report['valid_pose_frames'], 3)
            self.assertEqual(report['missing_or_other_map_frames'], 4)
            self.assertEqual(report['trajectory_segments'], 2)
            self.assertFalse(report['interpolation_over_lost_frames'])
            self.assertAlmostEqual(report['fps'], 12.)

    def test_coordinate_and_depth_conventions(self):
        R = q_to_R(np.array([.8, .1, .2, .3]))
        center = np.array([1., 2., 3.])
        point = np.array([[2., 1., 5.]])
        np.testing.assert_allclose((point-center)@R.T, point@R.T-R@center)
        colors = np.array([[0, 0, 255], [255, 0, 0]], np.uint8)
        image, _ = splat(np.array([[3., 3.], [3., 3.]]), np.array([4., 1.]), colors, 8, 8, 1)
        np.testing.assert_array_equal(image[3, 3], [255, 0, 0])

    def test_upstream_patch_idempotence(self):
        spec = importlib.util.spec_from_file_location('patch_upstream', ROOT/'scripts/patch_upstream.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as td:
            source = Path(td)
            (source/'include').mkdir()
            (source/'src').mkdir()
            (source/'include/System.h').write_text('class System {\npublic:\nprivate:\n};\n')
            (source/'CMakeLists.txt').write_text('set(CMAKE_CXX_FLAGS "-std=c++11")\n')
            module.patch(source)
            module.patch(source)
            self.assertEqual((source/'include/System.h').read_text().count('ExportMonocularReplay'), 1)
            self.assertEqual((source/'CMakeLists.txt').read_text().count('target_sources'), 1)
            self.assertIn('-std=c++17', (source/'CMakeLists.txt').read_text())

    def test_archive_path_traversal_rejected(self):
        spec = importlib.util.spec_from_file_location('fetch_dependencies', ROOT/'scripts/fetch_dependencies.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            archive = root/'unsafe.tar.gz'
            with tarfile.open(archive, 'w:gz') as tar:
                member = tarfile.TarInfo('package/../../escape')
                tar.addfile(member)
            with self.assertRaisesRegex(ValueError, 'Unsafe archive path'):
                module.extract_archive(archive, root/'target')
            self.assertFalse((root/'escape').exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)
