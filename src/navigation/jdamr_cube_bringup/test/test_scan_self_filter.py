"""Self-filter for the bimanual frame columns in the LiDAR plane (2026-10-05)."""

import base64
import importlib.util
import json
import math
import numpy as np
from pathlib import Path
from types import SimpleNamespace

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_bringup.scan_self_filter import (
    blind_sectors, filter_ranges, load_boxes, ScanSelfFilter)
from launch import LaunchContext
from launch.actions import OpaqueFunction
import launch.logging
import pytest
import rclpy
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy
import yaml
import zlib

PACKAGE = Path(__file__).resolve().parents[1]
MODEL = PACKAGE / 'config/self_filter_hold_flow_model.yaml'
LASER = (0.10, 0.0, 0.0)   # hold_flow model laser_link in base_footprint


def _boxes():
    return load_boxes(__import__('json').dumps(yaml.safe_load(MODEL.read_text())['self_boxes']))


def _ranges_for(points, increment=math.radians(1.0)):
    ranges = [5.0] * 360
    for x, y in points:
        bearing = math.atan2(y - LASER[1], x - LASER[0])
        ranges[int(round((bearing + math.pi) / increment)) % 360] = math.hypot(
            x - LASER[0], y - LASER[1])
    return ranges


def test_returns_on_a_column_become_no_return_and_the_rest_stay():
    ranges = _ranges_for([(0.15, 0.205), (-0.15, -0.205), (0.60, 0.0)])
    ranges[10] = math.nan
    out, dropped = filter_ranges(ranges, -math.pi, math.radians(1.0), LASER, _boxes(), 0.01)
    assert dropped == 2
    assert sum(math.isinf(r) for r in out) == 2
    assert out.count(5.0) == ranges.count(5.0)                      # far returns kept
    assert any(abs(r - 0.5) < 1e-9 for r in out)                    # the obstacle ahead
    assert math.isnan(out[10])                                      # invalid stays invalid


def test_a_return_just_outside_a_column_is_kept():
    out, dropped = filter_ranges(_ranges_for([(0.15, 0.24)]), -math.pi, math.radians(1.0),
                                 LASER, _boxes(), 0.01)
    assert dropped == 0


def test_blind_sectors_are_reported_per_column():
    sectors = {name: (start, end, near) for name, start, end, near in
               blind_sectors(LASER, _boxes(), 0.01)}
    start, end, near = sectors['front_left_column']
    # 4x4 cm with the margin, 0.19 m away: 13 deg of bearing is dropped.
    assert start < 76.3 < end and 10.0 < end - start < 16.0
    assert near == pytest.approx(0.19, abs=0.01)
    start, end, _near = sectors['rear_right_column']
    assert start < -140.6 < end


def test_invalid_boxes_are_refused():
    with pytest.raises(ValueError):
        load_boxes('[{"center_xy_m": [0.1, 0.0], "size_xy_m": [0.0, 0.02]}]')
    assert load_boxes('') == []


def _lidar_nodes(config):
    spec = importlib.util.spec_from_file_location(
        'real_bringup', PACKAGE / 'launch/real_bringup.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.Node = lambda **kwargs: SimpleNamespace(**kwargs)    # capture what gets built
    description = module.generate_launch_description()
    function = next(entity for entity in description.entities
                    if isinstance(entity, OpaqueFunction))
    context = LaunchContext()
    context.launch_configurations['scan_self_filter_config'] = config
    context.launch_configurations['lidar_port'] = '/dev/ydlidar_g4'
    return function.execute(context)


def test_launch_keeps_the_driver_on_scan_without_a_config():
    nodes = _lidar_nodes('')
    assert [node.executable for node in nodes] == ['ydlidar_g4_node']
    assert nodes[0].parameters[0]['scan_topic'] == 'scan'


def test_launch_inserts_the_filter_between_scan_raw_and_scan():
    nodes = _lidar_nodes(str(MODEL))
    assert [node.executable for node in nodes] == ['ydlidar_g4_node', 'scan_self_filter']
    assert nodes[0].parameters[0]['scan_topic'] == 'scan_raw'
    filtered = nodes[1].parameters[0]
    assert (filtered['input_topic'], filtered['output_topic']) == ('scan_raw', 'scan')
    assert len(load_boxes(filtered['self_boxes_json'])) == 4


def test_launch_preserves_the_configured_self_return_coordinate_frame():
    document = PACKAGE / 'config/self_filter_columns_20261012.yaml'
    nodes = _lidar_nodes(str(document))
    assert nodes[1].parameters[0]['base_frame'] == 'laser_link'


def test_sensor_frame_columns_preserve_the_original_empirical_beam_mask():
    document = yaml.safe_load((PACKAGE / 'config/self_filter_columns_20261012.yaml').read_text())
    source = document['provenance']['coordinate_reframe']
    previous = load_boxes(json.dumps(source['previous_self_boxes']))
    current = load_boxes(json.dumps(document['self_boxes']))
    update = document['provenance']['rear_right_boundary_update']
    reframed_boxes = json.loads(json.dumps(document['self_boxes']))
    right = next(box for box in reframed_boxes if box['name'] == 'rear_right_column_returns')
    right['center_xy_m'] = update['previous_center_xy_m']
    right['size_xy_m'] = update['previous_size_xy_m']
    left_update = document['provenance']['rear_left_boundary_update']
    left = next(box for box in reframed_boxes if box['name'] == 'rear_left_column_returns')
    left['center_xy_m'] = left_update['previous_center_xy_m']
    left['size_xy_m'] = left_update['previous_size_xy_m']
    reframed = load_boxes(json.dumps(reframed_boxes))
    # A dense set of ray endpoints exercises both near ROI boundaries and
    # distant returns. Frame conversion is exact; the later boundary correction
    # may remove only points in its recorded extension.
    total = 0
    for radius in (.32, .33, .34, .35, 5.):
        ranges = np.full(72001, radius)
        for angle_min, increment in [(-math.pi, 2*math.pi/len(ranges)),
                                     (.9, -2*math.pi/len(ranges))]:
            old, old_count = filter_ranges(
                ranges, angle_min, increment, (-.01, 0., math.pi),
                previous, document['margin_m'])
            same, same_count = filter_ranges(
                ranges, angle_min, increment, (0., 0., 0.),
                reframed, document['margin_m'])
            total += old_count
            assert old_count == same_count
            np.testing.assert_array_equal(same, old)
            new, _ = filter_ranges(ranges, angle_min, increment, (0., 0., 0.),
                                   current, document['margin_m'])
            assert np.all(np.isinf(new)[np.isinf(old)])
            changed = np.isinf(new) & np.isfinite(old)
            angles = angle_min + np.arange(len(ranges)) * increment
            x, y = ranges * np.cos(angles), ranges * np.sin(angles)
            right_extension = ((x >= .256) & (x <= .290)
                               & (y > .217) & (y <= .220))
            left_extension = ((x >= .227) & (x <= .257)
                              & (y >= -.248) & (y <= -.211)
                              & ((x > .255) | (y > -.212)))
            assert np.all((right_extension | left_extension)[changed])
    assert total > 0


def test_recorded_post_yaw_self_returns_are_removed_and_rear_box_stays():
    fixture = PACKAGE.parent / 'jdamr_cube_navigation/test/fixtures/rear_laser_yaw_20261020.json'
    evidence = json.loads(fixture.read_text())['self_filter_replay']
    metadata = evidence['scan_metadata']
    raw = json.loads(zlib.decompress(base64.b64decode(evidence['ranges_zlib_base64'])))
    ranges = [math.inf if value is None else value for value in raw]
    document = yaml.safe_load((PACKAGE / 'config/self_filter_columns_20261012.yaml').read_text())
    out, dropped = filter_ranges(ranges, metadata['angle_min'], metadata['angle_increment'],
                                 (0., 0., 0.), load_boxes(json.dumps(document['self_boxes'])),
                                 document['margin_m'])
    lx, ly, yaw = evidence['laser_pose_m_rad']
    points = [(lx+r*math.cos(yaw+metadata['angle_min']+i*metadata['angle_increment']),
               ly+r*math.sin(yaw+metadata['angle_min']+i*metadata['angle_increment']))
              if math.isfinite(r) else (math.inf, math.inf)
              for i, r in enumerate(ranges)]
    for hit in evidence['observed_blocking_points_base_xy_m']:
        index = min(range(len(points)), key=lambda i: math.dist(points[i], hit))
        assert math.dist(points[index], hit) < 1e-9
        assert math.isinf(out[index])
    rear = [i for i, (x, _y) in enumerate(points) if math.isfinite(x) and x < -.4]
    assert rear and dropped >= len(evidence['observed_blocking_points_base_xy_m'])
    assert all(out[i] == ranges[i] for i in rear)


def test_launch_refuses_bad_boxes_with_a_clear_message(tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text('self_boxes:\n- {name: a, center_xy_m: [0, 0], size_xy_m: [0.0, 0.02]}\n')
    with pytest.raises(RuntimeError, match='invalid scan_self_filter_config'):
        _lidar_nodes(str(bad))
    missing = tmp_path / 'no_boxes.yaml'
    missing.write_text('margin_m: 0.01\n')
    with pytest.raises(RuntimeError, match='self_boxes'):
        _lidar_nodes(str(missing))


def test_launch_resolves_a_bare_file_name_in_the_installed_config(tmp_path, monkeypatch):
    share = tmp_path / 'share'
    (share / 'config').mkdir(parents=True)
    (share / 'config/mine.yaml').write_text(MODEL.read_text())
    real = get_package_share_directory
    monkeypatch.setattr('ament_index_python.packages.get_package_share_directory',
                        lambda name: str(share) if name == 'jdamr_cube_bringup' else real(name))
    nodes = _lidar_nodes('mine.yaml')
    assert nodes[1].executable == 'scan_self_filter'


def test_launch_warns_about_an_unmeasured_model(monkeypatch):
    warnings = []
    monkeypatch.setattr(launch.logging, 'get_logger',
                        lambda _name: SimpleNamespace(warning=warnings.append))
    _lidar_nodes(str(MODEL))
    assert any('model_not_measured' in message for message in warnings)


class _StubTf:
    def __init__(self, available):
        self.available, self.lookups = available, 0

    def lookup_transform(self, _target, _source, _time):
        from tf2_ros import LookupException
        self.lookups += 1
        if not self.available:
            raise LookupException('no laser_link')
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.1, y=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))


class _Listener:
    unregistered = 0

    def unregister(self):
        self.unregistered += 1


@pytest.fixture
def node():
    rclpy.init()
    instance = ScanSelfFilter()
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def _scan():
    from sensor_msgs.msg import LaserScan
    scan = LaserScan()
    scan.header.frame_id = 'laser_link'
    scan.angle_min, scan.angle_increment = -math.pi, math.radians(1.0)
    scan.ranges = [5.0] * 360
    return scan


def test_filtered_scan_is_reliable_keep_last_10_like_the_driver(node):
    qos = node.publisher.qos_profile
    assert qos.reliability == QoSReliabilityPolicy.RELIABLE
    assert qos.history == QoSHistoryPolicy.KEEP_LAST and qos.depth == 10


def test_tf_subscription_is_dropped_once_the_laser_pose_is_cached(node):
    node.tf, node.tf_listener = _StubTf(True), _Listener()
    sent = []
    node.publisher = SimpleNamespace(publish=sent.append)
    node._scan(_scan())
    node._scan(_scan())
    assert node.tf_listener.unregistered == 1 and len(sent) == 2
    assert node.tf.lookups == 1


def test_missing_laser_tf_drops_scans_and_warns_with_the_frames(node, monkeypatch):
    node.tf, node.tf_listener = _StubTf(False), _Listener()
    sent, warnings = [], []
    node.publisher = SimpleNamespace(publish=sent.append)
    monkeypatch.setattr(type(node.get_logger()), 'warning',
                        lambda _self, message, **kwargs: warnings.append((message, kwargs)))
    node._scan(_scan())
    assert sent == [] and node.tf_listener.unregistered == 0
    message, kwargs = warnings[0]
    assert 'laser_link' in message and node.base_frame in message
    assert kwargs['throttle_duration_sec'] == 5.0
