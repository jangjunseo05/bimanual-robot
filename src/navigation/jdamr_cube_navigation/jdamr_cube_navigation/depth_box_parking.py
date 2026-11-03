"""Publish perception-only box-parking observations from Astra depth."""

from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from jdamr_cube_navigation.box_top_detection import (
    BoxTopConfig,
    CameraIntrinsics,
    detect_box_front,
    detect_box_top,
    DetectionStability,
)
from jdamr_cube_navigation.camera_info_history import CameraInfoHistory, stamp_nanoseconds
from jdamr_cube_navigation.dedicated_tf import DedicatedTfListener
from jdamr_cube_navigation.depth_obstacle_core import _depth_view
from jdamr_cube_navigation.depth_obstacle_filter import camera_intrinsics, transform_matrix
from jdamr_cube_navigation.observer_camera_mount import load_observer_camera_mount
from jdamr_cube_navigation.table_edge_detection import detect_table_edge, TableEdgeConfig
from jdamr_cube_navigation.table_surface_model import detect_table_scene
from jdamr_cube_navigation.table_surface_tracking import (
    candidates_in_region, TableSurfaceTracker, TargetRegion, TrackConfig,
)
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException


LATEST_SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def decode_depth_observation(
        info, image, optical_frame, calibration_model,
        now_s, max_image_age_s):
    """Validate synchronized geometry, freshness and the ROS depth buffer."""
    stamp_s = (image.header.stamp.sec
               + image.header.stamp.nanosec * 1e-9)
    age_s = now_s - stamp_s
    if (not math.isfinite(stamp_s) or stamp_s <= 0.0
            or not math.isfinite(age_s)
            or not 0.0 <= age_s <= max_image_age_s):
        raise ValueError('stale_or_future_depth')
    if str(image.encoding).upper() != '16UC1':
        raise ValueError(f'unsupported_depth_encoding:{image.encoding}')
    info_stamp_ns = stamp_nanoseconds(info.header.stamp)
    image_stamp_ns = stamp_nanoseconds(image.header.stamp)
    if info_stamp_ns > image_stamp_ns:
        raise ValueError('future_camera_info')
    if info_stamp_ns and image_stamp_ns - info_stamp_ns > round(max_image_age_s * 1e9):
        raise ValueError('stale_camera_info')
    values = camera_intrinsics(
        info, image, optical_frame, calibration_model)
    intrinsics = CameraIntrinsics(
        width=image.width,
        height=image.height,
        focal_x_px=values[0],
        focal_y_px=values[1],
        center_x_px=values[2],
        center_y_px=values[3],
    )
    depth_mm = _depth_view(
        image.data, image.width, image.height, image.step,
        image.encoding, image.is_bigendian)
    return depth_mm, intrinsics, stamp_s, age_s


class DepthBoxParkingNode(Node):
    """Observe a box top without owning or publishing velocity commands."""

    def __init__(self):
        """Configure a read-only observer with bounded input freshness."""
        super().__init__('jdamr_depth_box_parking')
        defaults = BoxTopConfig()
        for name, value in (
            ('minimum_depth_m', defaults.minimum_depth_m),
            ('maximum_depth_m', defaults.maximum_depth_m),
            ('desired_standoff_m', defaults.desired_standoff_m),
            ('pixel_step', defaults.pixel_step),
            ('plane_distance_m', defaults.plane_distance_m),
            ('minimum_normal_y', defaults.minimum_normal_y),
            ('minimum_front_normal_z', defaults.minimum_front_normal_z),
            ('camera_roll_rad', defaults.camera_roll_rad),
            ('camera_pitch_rad', defaults.camera_pitch_rad),
            ('maximum_top_y_m', defaults.maximum_top_y_m),
            ('minimum_width_m', defaults.minimum_width_m),
            ('maximum_width_m', defaults.maximum_width_m),
            ('minimum_height_m', defaults.minimum_height_m),
            ('maximum_height_m', defaults.maximum_height_m),
            ('minimum_depth_extent_m', defaults.minimum_depth_extent_m),
            ('minimum_inliers', defaults.minimum_inliers),
            ('ransac_iterations', defaults.ransac_iterations),
            ('maximum_points', defaults.maximum_points),
            ('random_seed', defaults.random_seed),
            ('processing_hz', 5.0),
            ('max_image_age_s', 0.5),
            ('perception_enabled', True),
            ('optical_frame', 'camera_color_optical_frame'),
            ('calibration_model', 'rectified_projection'),
            ('stable_frames', 8),
            ('stable_distance_m', 0.03),
            ('stable_lateral_m', 0.03),
            ('stable_angle_deg', 3.0),
            ('surface_mode', 'front'),
            ('table_components', False),
            ('table_tracking', False),
            ('table_request_enabled', False),
            ('table_target_region_map', ''),
            ('table_calibration_id', ''),
        ):
            self.declare_parameter(name, value)
        table_defaults = TableEdgeConfig()
        for name in TableEdgeConfig.__dataclass_fields__:
            if name not in BoxTopConfig.__dataclass_fields__:
                self.declare_parameter(name, getattr(table_defaults, name))
        self.declare_parameter(
            'camera_mount_file', '', ParameterDescriptor(read_only=True))
        mount_path = str(self.get_parameter('camera_mount_file').value).strip()
        if mount_path:
            # A selected provenance file owns startup geometry. Explicit geometry
            # parameters remain available when camera_mount_file is empty.
            config_directory = (Path(get_package_share_directory('jdamr_cube_navigation'))
                                / 'config' if not Path(mount_path).expanduser().is_absolute()
                                else Path('.'))
            mount_values = load_observer_camera_mount(mount_path, config_directory)
            result = self.set_parameters_atomically([
                Parameter(name, value=value) for name, value in mount_values.items()])
            if not result.successful:
                raise ValueError(f'cannot apply observer camera mount: {result.reason}')
        self._config = BoxTopConfig(**{
            field: self.get_parameter(field).value
            for field in BoxTopConfig.__dataclass_fields__
        })
        table_values = {
            name: self.get_parameter(name).value
            for name in TableEdgeConfig.__dataclass_fields__
        }
        table_values['minimum_normal_y'] = max(
            self._config.minimum_normal_y, table_defaults.minimum_normal_y)
        self._table_config = TableEdgeConfig(**table_values)
        self._table_components = bool(self.get_parameter('table_components').value)
        self._table_tracking = bool(self.get_parameter('table_tracking').value)
        self._table_request_enabled = bool(self.get_parameter('table_request_enabled').value)
        if self._table_tracking and not self._table_components:
            raise ValueError('table_tracking requires table_components')
        if self._table_request_enabled and not self._table_components:
            raise ValueError('table_request_enabled requires table_components')
        self._table_tracker = None
        self._table_region_map = self._parse_table_region(
            str(self.get_parameter('table_target_region_map').value))
        self._table_region_odom = None
        self._table_calibration_id = str(self.get_parameter('table_calibration_id').value)
        if self._table_request_enabled and (
                self._table_region_map is None or not self._table_calibration_id.strip()):
            raise ValueError(
                'pointcloud request requires a selected region and calibration identity')
        if self._table_components:
            self._table_tf = Buffer()
            # The 50 ms stamped lookups in _on_depth sleep-poll; on this node's own
            # single-threaded spin no /tf callback could run meanwhile.
            self._table_tf_listener = DedicatedTfListener(
                self._table_tf, self, 'jdamr_depth_box_parking_tf')
        if self._table_tracking:
            self._table_tracker = TableSurfaceTracker(TrackConfig(
                required_frames=int(self.get_parameter('stable_frames').value)))
        # The configured range is the ceiling; the executor narrows it per table.
        self._depth_ceiling_m = float(self._config.maximum_depth_m)
        self._stability = DetectionStability(
            required_frames=int(self.get_parameter('stable_frames').value),
            maximum_distance_spread_m=float(
                self.get_parameter('stable_distance_m').value),
            maximum_lateral_spread_m=float(
                self.get_parameter('stable_lateral_m').value),
            maximum_angle_spread_deg=float(
                self.get_parameter('stable_angle_deg').value),
        )
        self._surface_mode = str(
            self.get_parameter('surface_mode').value)
        if self._surface_mode not in {'front', 'top', 'table_edge'}:
            raise ValueError('surface_mode must be front, top or table_edge')
        processing_hz = float(self.get_parameter('processing_hz').value)
        self._max_image_age_s = float(
            self.get_parameter('max_image_age_s').value)
        self._perception_enabled = bool(
            self.get_parameter('perception_enabled').value)
        if not math.isfinite(processing_hz) or processing_hz <= 0.0:
            raise ValueError('processing_hz must be finite and positive')
        if (not math.isfinite(self._max_image_age_s)
                or self._max_image_age_s <= 0.0):
            raise ValueError('max_image_age_s must be finite and positive')
        self._optical_frame = str(
            self.get_parameter('optical_frame').value)
        self._calibration_model = str(
            self.get_parameter('calibration_model').value)
        self._minimum_period_s = 1.0 / processing_hz
        self._last_processed_s = -math.inf
        self._processing_duration_estimate_s = None
        self._freshness_skip_pending = False
        self._precalculation_skips = 0
        self._last_depth_callback_s = time.monotonic()
        self._camera_info_history = CameraInfoHistory()
        self._camera_geometry = None
        self._status = self.create_publisher(
            String, '/box_parking/perception_status', 1)
        self._create_depth_subscriptions()
        self.create_timer(
            self._max_image_age_s,
            self._recover_silent_depth_subscriptions)
        self.add_on_set_parameters_callback(self._on_parameters)
        self.get_logger().info(
            'depth box parking perception started; '
            'velocity output is disabled')

    def _create_depth_subscriptions(self) -> None:
        """Create the paired calibration and depth subscriptions."""
        self._camera_info_subscription = self.create_subscription(
            CameraInfo, '/camera/depth/camera_info',
            self._on_camera_info, LATEST_SENSOR_QOS)
        self._depth_subscription = self.create_subscription(
            Image, '/camera/depth/image_raw',
            self._on_depth_with_watchdog_activity, LATEST_SENSOR_QOS)

    def _on_depth_with_watchdog_activity(self, message: Image) -> None:
        """Keep a long-running callback from looking like a silent DDS reader."""
        self._depth_callback_active = True
        try:
            self._on_depth(message)
        finally:
            self._last_depth_callback_s = time.monotonic()
            self._depth_callback_active = False

    def _recover_silent_depth_subscriptions(self) -> None:
        """Rematch a stalled local DDS reader while a depth writer is present."""
        depth_subscription = getattr(self, '_depth_subscription', None)
        if (depth_subscription is None
                or depth_subscription.get_publisher_count() == 0):
            return
        if getattr(self, '_depth_callback_active', False):
            return
        now_s = time.monotonic()
        if now_s - self._last_depth_callback_s <= 4.0 * self._max_image_age_s:
            return
        self.destroy_subscription(self._camera_info_subscription)
        self.destroy_subscription(depth_subscription)
        self._camera_info_history = CameraInfoHistory()
        self._camera_geometry = None
        self._last_depth_callback_s = now_s
        self._create_depth_subscriptions()
        self._reject('depth_subscription_recreated_after_silence')

    def destroy_node(self):
        """Stop the dedicated TF thread before destroying this node."""
        listener = getattr(self, '_table_tf_listener', None)
        if listener is not None:
            self._table_tf_listener = None
            listener.shutdown()
        return super().destroy_node()

    def _on_parameters(self, parameters) -> SetParametersResult:
        """Apply a bounded depth window or a complete camera/surface update."""
        updates = {}
        mode = getattr(self, '_surface_mode', 'front')
        region = getattr(self, '_table_region_map', None)
        calibration_id = getattr(self, '_table_calibration_id', '')
        request_enabled = getattr(self, '_table_request_enabled', False)
        perception_enabled = getattr(self, '_perception_enabled', True)
        for parameter in parameters:
            value = parameter.value
            name = parameter.name
            if name == 'perception_enabled':
                if not isinstance(value, bool):
                    return SetParametersResult(
                        successful=False, reason='perception flag must be bool')
                perception_enabled = value
                continue
            if name == 'table_request_enabled':
                if not isinstance(value, bool):
                    return SetParametersResult(
                        successful=False, reason='request flag must be bool')
                if value and not getattr(self, '_table_components', False):
                    return SetParametersResult(
                        successful=False, reason='table components must be enabled at startup')
                request_enabled = value
                continue
            if name == 'table_target_region_map':
                if value and not getattr(self, '_table_components', False):
                    return SetParametersResult(
                        successful=False, reason='table components must be enabled at startup')
                try:
                    region = self._parse_table_region(value)
                except (ValueError, TypeError):
                    return SetParametersResult(
                        successful=False, reason='invalid table target region')
                continue
            if name == 'table_calibration_id':
                if not isinstance(value, str):
                    return SetParametersResult(
                        successful=False, reason='invalid calibration identity')
                calibration_id = value
                continue
            if name == 'surface_mode':
                if not isinstance(value, str) or value not in {'front', 'top', 'table_edge'}:
                    return SetParametersResult(successful=False, reason='invalid surface_mode')
                mode = value
                continue
            if name not in {'maximum_depth_m', 'camera_roll_rad',
                            'camera_pitch_rad', 'camera_height_m'}:
                return SetParametersResult(
                    successful=False, reason=f'{name} is fixed at start')
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value)):
                return SetParametersResult(successful=False, reason=f'{name} must be finite')
            if (name == 'maximum_depth_m'
                    and not self._config.minimum_depth_m < value <= self._depth_ceiling_m):
                return SetParametersResult(
                    successful=False,
                    reason='maximum_depth_m must lie in (minimum_depth_m, configured maximum]')
            if name == 'camera_height_m' and value <= 0.0:
                return SetParametersResult(
                    successful=False, reason='camera_height_m must be positive')
            updates[name] = float(value)
        if not request_enabled:
            region, calibration_id = None, ''
        elif region is None or not calibration_id.strip():
            return SetParametersResult(
                successful=False, reason='pointcloud request needs region and calibration ID')
        table_config = getattr(self, '_table_config', None)
        if table_config is None:
            table_values = {
                name: getattr(self._config, name) for name in BoxTopConfig.__dataclass_fields__
            }
            table_values['minimum_normal_y'] = max(
                self._config.minimum_normal_y, TableEdgeConfig().minimum_normal_y)
            table_config = TableEdgeConfig(**table_values)
        candidate_table = replace(table_config, **updates)
        if mode == 'table_edge':
            if candidate_table.camera_height_m <= 0.0:
                return SetParametersResult(
                    successful=False, reason='table_edge requires a camera height')
            if candidate_table.maximum_depth_m <= candidate_table.minimum_depth_m:
                return SetParametersResult(
                    successful=False, reason='table_edge depth window is empty')
        common_updates = {name: value for name, value in updates.items()
                          if name in BoxTopConfig.__dataclass_fields__}
        reference_changed = (
            mode != getattr(self, '_surface_mode', 'front')
            or region != getattr(self, '_table_region_map', None)
            or request_enabled != getattr(self, '_table_request_enabled', False)
            or perception_enabled != getattr(self, '_perception_enabled', True)
            or calibration_id != getattr(self, '_table_calibration_id', '')
            or any(getattr(candidate_table, name) != getattr(table_config, name)
                   for name in ('camera_height_m', 'camera_roll_rad', 'camera_pitch_rad')))
        changed = candidate_table != table_config or reference_changed
        perception_changed = (
            perception_enabled != getattr(self, '_perception_enabled', True))
        if parameters:
            self._config = replace(self._config, **common_updates)
            self._table_config = candidate_table
            self._surface_mode = mode
            self._perception_enabled = perception_enabled
            self._table_region_map = region
            self._table_request_enabled = request_enabled
            self._table_calibration_id = calibration_id
            if reference_changed:
                self._table_region_odom = None
            if changed:
                self._processing_duration_estimate_s = None
                self._freshness_skip_pending = False
                self._stability.update(None)
                if getattr(self, '_table_tracker', None) is not None:
                    self._table_tracker.reset()
            if perception_changed and hasattr(self, '_status'):
                self._publish({
                    'detected': False,
                    'stable': False,
                    'stable_frame_count': 0,
                    'reason': ('perception_enabled_waiting_for_depth'
                               if perception_enabled else 'perception_disabled'),
                })
        return SetParametersResult(successful=True)

    @staticmethod
    def _parse_table_region(value):
        """Decode an explicit map-frame center and radius, or no selection prior."""
        if not isinstance(value, str):
            raise ValueError('table region must be JSON text')
        if not value:
            return None
        region = json.loads(value)
        if (not isinstance(region, list) or len(region) != 3
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) for v in region) or region[2] <= 0.):
            raise ValueError('table region must be finite [map_x, map_y, radius]')
        return tuple(float(v) for v in region)

    def _on_camera_info(self, message: CameraInfo) -> None:
        if not hasattr(self, '_camera_info_history'):
            self._camera_info_history = CameraInfoHistory()
        try:
            self._camera_info_history.add(message)
        except ValueError as error:
            self._reject(str(error))

    def _publish(self, document: dict, *, timing_markers=None) -> None:
        """Publish status and retain output costs for the following sample."""
        if timing_markers is not None:
            callback_started_s, calculation_started_s, calculation_finished_s = timing_markers
            publish_boundary_s = time.monotonic()
            processing_duration_s = publish_boundary_s - callback_started_s
            self._processing_duration_estimate_s = processing_duration_s
            document = {
                **document,
                'input_decode_duration_s': calculation_started_s - callback_started_s,
                'calculation_duration_s': calculation_finished_s - calculation_started_s,
                'postprocess_duration_s': publish_boundary_s - calculation_finished_s,
                'processing_duration_s': processing_duration_s,
                'precalculation_skipped_frames': getattr(
                    self, '_precalculation_skips', 0),
                'previous_status_serialization_duration_s': getattr(
                    self, '_last_status_serialization_duration_s', None),
                'previous_status_publish_duration_s': getattr(
                    self, '_last_status_publish_duration_s', None),
            }
            self._precalculation_skips = 0
        document = {**document, 'control_ready': False}
        message = String()
        serialization_started_s = time.monotonic()
        message.data = json.dumps(
            document, ensure_ascii=False, separators=(',', ':'))
        self._last_status_serialization_duration_s = (
            time.monotonic() - serialization_started_s)
        publish_started_s = time.monotonic()
        try:
            self._status.publish(message)
        except RCLError:
            if rclpy.ok():
                raise
        finally:
            self._last_status_publish_duration_s = (
                time.monotonic() - publish_started_s)

    def _reject(self, reason: str, **details) -> None:
        """Clear consecutive stability after any unusable observation."""
        self._reset_measurements()
        self._publish({
            'detected': False,
            'stable': False,
            'stable_frame_count': 0,
            'reason': reason,
            **details,
        })

    def _reset_measurements(self) -> None:
        """Discard accumulated observations after input geometry or time changes."""
        self._freshness_skip_pending = False
        self._stability.update(None)
        if getattr(self, '_table_tracker', None) is not None:
            self._table_tracker.reset()

    def _table_observation(self, depth_mm, intrinsics, stamp_s, frame_id):
        """Adapt optional component/tracking output to the existing observer contract."""
        if (not getattr(self, '_table_components', False)
                or not getattr(self, '_table_request_enabled', False)):
            plane = {}
            detection = detect_table_edge(
                depth_mm, intrinsics, self._table_config, plane_result=plane)
            fields = {'tabletop_plane': plane} if plane else {}
            return detection, fields, None
        region_map = getattr(self, '_table_region_map', None)
        needs_transform = self._table_tracker is not None or region_map is not None
        transform, transform_error = None, None
        if needs_transform:
            try:
                stamped = self._table_tf.lookup_transform(
                    'odom', frame_id, Time(seconds=stamp_s), timeout=Duration(seconds=.05))
                transform = transform_matrix(stamped.transform)
                if region_map is not None and getattr(self, '_table_region_odom', None) is None:
                    map_tf = self._table_tf.lookup_transform(
                        'odom', 'map', Time(seconds=stamp_s), timeout=Duration(seconds=.05))
                    map_to_odom = transform_matrix(map_tf.transform)
                    center = map_to_odom @ np.array([*region_map[:2], 0., 1.])
                    self._table_region_odom = TargetRegion(tuple(center[:2]), region_map[2])
            except (TransformException, ValueError) as error:
                transform_error = error
        region = getattr(self, '_table_region_odom', None)
        priority = {}
        if transform_error is None and transform is not None and region is not None:
            priority = {'target_region_odom': region, 'odom_from_optical': transform}
        scene = detect_table_scene(depth_mm, intrinsics, self._table_config, **priority)
        fields = {
            'table_component_count': len(scene.component_sizes),
            'table_candidate_count': len(scene.candidates),
            'table_evaluated_components': scene.evaluated_components,
            'table_input_point_count': scene.input_point_count,
            'table_height_point_count': scene.height_point_count,
            'table_region_prioritized_components': getattr(
                scene, 'region_prioritized_components', 0),
        }
        if not needs_transform:
            return (scene.candidates[0] if len(scene.candidates) == 1 else None), fields, None
        if transform_error is not None:
            # Keep the completed detector evidence even when tracking cannot run.
            if self._table_tracker is not None:
                self._table_tracker.reset()
            fields.update({
                'table_track_state': 'transform_unavailable',
                'table_track_frame_count': 0,
                'reason': ('table_tracking_transform_unavailable:'
                           if isinstance(transform_error, TransformException)
                           else 'table_tracking_transform_invalid:') + str(transform_error),
            })
            return None, fields, False
        if self._table_tracker is None:
            selected = candidates_in_region(scene.candidates, transform, region)
            return (selected[0] if len(selected) == 1 else None), fields, None
        calibration_id = hashlib.sha256(json.dumps({
            'table_config': asdict(self._table_config),
            'intrinsics': asdict(intrinsics), 'frame_id': frame_id,
            'mount_revision': getattr(self, '_table_calibration_id', ''),
        }, sort_keys=True).encode()).hexdigest()
        track = self._table_tracker.update(
            scene.candidates, stamp_s, transform, calibration_id, region)
        fields.update({
            'table_track_state': track.state,
            'table_track_id': track.track_id,
            'table_measurement_stamp_s': track.measurement_stamp_s,
            'table_track_frame_count': track.stable_frames,
            'table_edge_endpoints_odom_m': track.endpoints_odom_m,
        })
        # Predicted geometry is diagnostic only. It must not become a fresh
        # detected/stable observation consumed by the current parking executor.
        stable = (track.state == 'observed'
                  and track.stable_frames >= self._table_tracker.config.required_frames)
        return track.detection, fields, stable

    def _skip_for_fresher_depth(self, stamp_s, now_s):
        """Yield one predicted-late callback so depth=1 can supply its newest frame."""
        estimate_s = getattr(self, '_processing_duration_estimate_s', None)
        if (not isinstance(estimate_s, (int, float))
                or isinstance(estimate_s, bool) or not math.isfinite(estimate_s)
                or estimate_s <= 0.0):
            self._freshness_skip_pending = False
            return False
        age_s = now_s - stamp_s
        if (not math.isfinite(age_s) or not 0.0 <= age_s <= self._max_image_age_s):
            self._freshness_skip_pending = False
            return False
        # Reserve half one configured processing period for JSON publication,
        # DDS delivery and the control callback to consume the result.
        delivery_headroom_s = min(
            self._minimum_period_s / 2.0, self._max_image_age_s / 2.0)
        completion_budget_s = max(0.0, self._max_image_age_s - delivery_headroom_s)
        if age_s + estimate_s <= completion_budget_s:
            self._freshness_skip_pending = False
            return False
        if getattr(self, '_freshness_skip_pending', False):
            # Never starve a slow or remotely delayed stream. One quick return is
            # enough for KEEP_LAST depth=1 to replace this callback with its newest.
            self._freshness_skip_pending = False
            return False
        self._freshness_skip_pending = True
        self._precalculation_skips = getattr(self, '_precalculation_skips', 0) + 1
        return True

    def _on_depth(self, message: Image) -> None:
        if not getattr(self, '_perception_enabled', True):
            return
        now_s = time.monotonic()
        self._last_depth_callback_s = now_s
        if now_s - self._last_processed_s < self._minimum_period_s:
            return
        try:
            table_fields, table_stable = {}, None
            ros_now_s = self.get_clock().now().nanoseconds * 1e-9
            stamp_s = (message.header.stamp.sec
                       + message.header.stamp.nanosec * 1e-9)
            if self._skip_for_fresher_depth(stamp_s, ros_now_s):
                return
            self._last_processed_s = now_s
            info = self._camera_info_history.select(
                stamp_nanoseconds(message.header.stamp),
                maximum_lag_ns=round(self._max_image_age_s * 1e9))
            if info is None:
                self._reject('camera_info_unavailable_for_depth_stamp')
                return
            depth_mm, intrinsics, stamp_s, age_s = decode_depth_observation(
                info, message, self._optical_frame,
                self._calibration_model, ros_now_s,
                self._max_image_age_s)
            geometry = (intrinsics, message.header.frame_id, self._calibration_model)
            previous_geometry = getattr(self, '_camera_geometry', None)
            if (self._camera_info_history.rewound
                    or (previous_geometry is not None and geometry != previous_geometry)):
                self._processing_duration_estimate_s = None
                self._reset_measurements()
            self._camera_geometry = geometry
            calculation_started_s = time.monotonic()
            if self._surface_mode == 'table_edge':
                detection, table_fields, table_stable = self._table_observation(
                    depth_mm, intrinsics, stamp_s, message.header.frame_id)
            else:
                detector = detect_box_front if self._surface_mode == 'front' else detect_box_top
                detection = detector(depth_mm, intrinsics, self._config)
            calculation_finished_s = time.monotonic()
        except (AttributeError, ValueError, TypeError,
                OverflowError, IndexError) as error:
            self._reject(str(error) or type(error).__name__)
            return
        except np.linalg.LinAlgError as error:
            self._reject(f'numerical_detection_failure:{error}')
            return
        except TransformException as error:
            self._reject(f'table_tracking_transform_unavailable:{error}')
            return
        input_age_s = age_s
        age_s = self.get_clock().now().nanoseconds * 1e-9 - stamp_s
        info_stamp_s = stamp_nanoseconds(info.header.stamp) * 1e-9
        timing = {
            'input_age_s': input_age_s,
            'camera_info_stamp_s': info_stamp_s,
            'camera_info_age_s': stamp_s - info_stamp_s if info_stamp_s else None,
            'camera_info_mode': 'stamped' if info_stamp_s else 'static',
        }
        if (not math.isfinite(age_s)
                or not 0.0 <= age_s <= self._max_image_age_s):
            self._reset_measurements()
            self._publish({
                'stamp_s': stamp_s,
                'age_s': age_s,
                **timing,
                **table_fields,
                'frame_id': message.header.frame_id,
                'detected': False,
                'stable': False,
                'stable_frame_count': 0,
                'reason': 'expired_during_processing',
                'control_ready': False,
            }, timing_markers=(
                now_s, calculation_started_s, calculation_finished_s))
            return
        stable = self._stability.update(detection)
        if table_stable is not None:
            stable = table_stable
        stable_frame_count = table_fields.get(
            'table_track_frame_count', self._stability.frame_count)
        if detection is None:
            self._publish({
                'stamp_s': stamp_s,
                'age_s': age_s,
                **timing,
                **table_fields,
                'frame_id': message.header.frame_id,
                'detected': False,
                'stable': False,
                'stable_frame_count': stable_frame_count,
                'reason': table_fields.get(
                    'reason', 'no_table_edge_candidate' if self._surface_mode == 'table_edge'
                    else 'no_box_surface_candidate'),
                'control_ready': False,
            }, timing_markers=(
                now_s, calculation_started_s, calculation_finished_s))
            return
        edge_fields = {}
        if detection.surface_kind == 'table_edge':
            edge_fields = {
                'edge_endpoints_optical_m': detection.edge_endpoints_optical_m,
                'table_height_m': detection.table_height_m,
                'edge_rms_m': detection.edge_rms_m,
                'edge_basis': 'tabletop_front_boundary',
                'camera_height_m': self._table_config.camera_height_m,
                'camera_roll_rad': self._table_config.camera_roll_rad,
                'camera_pitch_rad': self._table_config.camera_pitch_rad,
            }
        self._publish({
            'stamp_s': stamp_s,
            'age_s': age_s,
            **timing,
            **table_fields,
            'frame_id': message.header.frame_id,
            'detected': True,
            'stable': stable,
            'stable_frame_count': stable_frame_count,
            'surface_kind': detection.surface_kind,
            **edge_fields,
            'front_distance_m': detection.front_distance_m,
            'desired_standoff_m': self._config.desired_standoff_m,
            'maximum_depth_m': self._config.maximum_depth_m,
            'standoff_error_m': (
                detection.front_distance_m
                - self._config.desired_standoff_m),
            'lateral_error_m': detection.center_x_m,
            'vertical_position_m': detection.center_y_m,
            'edge_angle_deg': math.degrees(detection.edge_angle_rad),
            'width_m': detection.width_m,
            'height_m': detection.height_m,
            'depth_extent_m': detection.depth_extent_m,
            'plane_normal': list(detection.plane_normal),
            'plane_rms_m': detection.plane_rms_m,
            'inlier_count': detection.inlier_count,
            'candidate_count': detection.candidate_count,
            'confidence': detection.confidence,
            'reason': 'stable_detection' if stable else 'collecting_stability',
            'control_ready': False,
            'control_blocker': 'camera_extrinsic_and_physical_validation',
        }, timing_markers=(
            now_s, calculation_started_s, calculation_finished_s))


def main(args=None):
    """Run the perception-only depth box-parking observer."""
    rclpy.init(args=args)
    node = DepthBoxParkingNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
