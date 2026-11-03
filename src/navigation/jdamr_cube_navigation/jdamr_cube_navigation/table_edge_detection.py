"""Detect a table's visible front edge from its horizontal top surface."""

from dataclasses import dataclass
import math

import numpy as np

from .box_top_detection import (
    _fit_plane,
    BoxTopConfig,
    BoxTopDetection,
    CameraIntrinsics,
    depth_points,
)


def _plane_from_triplet(sample: np.ndarray):
    """Fit a plane from three points without the generic ``np.cross`` path."""
    first, second, third = sample
    first_edge = second - first
    second_edge = third - first
    normal = np.array([
        first_edge[1] * second_edge[2] - first_edge[2] * second_edge[1],
        first_edge[2] * second_edge[0] - first_edge[0] * second_edge[2],
        first_edge[0] * second_edge[1] - first_edge[1] * second_edge[0],
    ])
    norm = math.sqrt(float(normal @ normal))
    if norm < 1e-9:
        return None
    normal /= norm
    return normal, -float(normal @ first)


@dataclass(frozen=True)
class TableEdgeConfig(BoxTopConfig):
    """Geometry and quality bounds for a visible table-front edge."""

    minimum_normal_y: float = 0.90
    maximum_background_depth_m: float = 2.0
    camera_height_m: float = 0.0
    minimum_table_height_m: float = 0.50
    maximum_table_height_m: float = 1.05
    minimum_edge_length_m: float = 0.25
    maximum_edge_rms_m: float = 0.01


@dataclass(frozen=True, kw_only=True)
class TableEdgeDetection(BoxTopDetection):
    """Table-top observation whose position is the observed front edge."""

    edge_endpoints_optical_m: tuple[
        tuple[float, float, float], tuple[float, float, float]
    ]
    table_height_m: float
    edge_rms_m: float


def _optical_to_level_rotation(config: TableEdgeConfig) -> np.ndarray:
    """Return optical-to-level-camera rotation, excluding mount yaw."""
    roll = config.camera_roll_rad
    pitch = config.camera_pitch_rad
    roll_cosine = math.cos(roll)
    roll_sine = math.sin(roll)
    pitch_cosine = math.cos(pitch)
    pitch_sine = math.sin(pitch)
    camera_to_level = np.array([
        [pitch_cosine, pitch_sine * roll_sine,
         pitch_sine * roll_cosine],
        [0.0, roll_cosine, -roll_sine],
        [-pitch_sine, pitch_cosine * roll_sine,
         pitch_cosine * roll_cosine],
    ])
    optical_to_camera = np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ])
    return camera_to_level @ optical_to_camera


def _validate_table_config(config: TableEdgeConfig) -> None:
    if (not math.isfinite(config.camera_height_m)
            or config.camera_height_m <= 0.0):
        raise ValueError('camera_height_m must be finite and positive')
    if (not math.isfinite(config.maximum_background_depth_m)
            or config.maximum_background_depth_m
            < config.maximum_depth_m):
        raise ValueError(
            'maximum_background_depth_m must be finite and no smaller '
            'than maximum_depth_m')
    if (not 0.0 < config.minimum_table_height_m
            < config.maximum_table_height_m):
        raise ValueError('table height bounds are invalid')
    if (not math.isfinite(config.minimum_edge_length_m)
            or config.minimum_edge_length_m <= 0.0):
        raise ValueError('minimum_edge_length_m must be finite and positive')
    if (not math.isfinite(config.maximum_edge_rms_m)
            or config.maximum_edge_rms_m <= 0.0):
        raise ValueError('maximum_edge_rms_m must be finite and positive')


def _horizontal_plane_is_valid(
        normal: np.ndarray, points: np.ndarray,
        config: TableEdgeConfig) -> bool:
    normal = normal / np.linalg.norm(normal)
    if abs(float(normal[2])) < config.minimum_normal_y:
        return False
    table_height_m = config.camera_height_m + float(np.median(points[:, 2]))
    return (
        config.minimum_table_height_m
        <= table_height_m <= config.maximum_table_height_m
    )


def _strongest_table_plane(
        points_level: np.ndarray,
        config: TableEdgeConfig) -> tuple[np.ndarray, float] | None:
    """Fit the strongest bounded horizontal plane in level coordinates."""
    random = np.random.default_rng(config.random_seed)
    point_heights_m = config.camera_height_m + points_level[:, 2]
    height_mask = (
        (point_heights_m >= config.minimum_table_height_m)
        & (point_heights_m <= config.maximum_table_height_m)
    )
    search_points = points_level[height_mask]
    if len(search_points) < config.minimum_inliers:
        return None
    if len(search_points) > config.maximum_points:
        stride = math.ceil(len(search_points) / config.maximum_points)
        search_points = search_points[::stride]
    best_plane = None
    best_score = -math.inf
    for _ in range(config.ransac_iterations):
        sample = search_points[
            random.choice(len(search_points), size=3, replace=False)]
        plane = _plane_from_triplet(sample)
        if plane is None:
            continue
        normal, offset = plane
        distances = np.abs(search_points @ normal + offset)
        mask = distances <= config.plane_distance_m
        count = int(np.count_nonzero(mask))
        if count < config.minimum_inliers:
            continue
        residual = float(np.sqrt(np.mean(distances[mask] ** 2)))
        score = count / max(residual, 0.001)
        if score <= best_score:
            continue
        candidate = search_points[mask]
        if not _horizontal_plane_is_valid(normal, candidate, config):
            continue
        if score > best_score:
            best_plane = normal, offset
            best_score = score
    return best_plane


def _has_depth_discontinuity_below(
        depth_mm: np.ndarray, row: int, column: int,
        intrinsics: CameraIntrinsics, rotation: np.ndarray,
        plane_normal: np.ndarray, plane_offset: float,
        config: TableEdgeConfig) -> bool:
    """Require a measured point below, outside the fitted tabletop plane."""
    step = config.pixel_step
    edge_depth_m = float(depth_mm[row, column]) * 0.001
    # The tabletop's front apron can occupy the first valid pixels below the
    # plane. Continue to measured background without treating it as a rejection.
    for offset in range(step, 17 * step, step):
        below = row + offset
        if below >= depth_mm.shape[0]:
            return False
        raw_depth_mm = float(depth_mm[below, column])
        if (not math.isfinite(raw_depth_mm)
                or raw_depth_mm <= 0.0
                or raw_depth_mm >= 65535.0):
            continue
        depth_m = raw_depth_mm * 0.001
        if not (config.minimum_depth_m
                < depth_m < config.maximum_background_depth_m):
            return False
        optical_point = np.array([
            (column - intrinsics.center_x_px) * depth_m
            / intrinsics.focal_x_px,
            (below - intrinsics.center_y_px) * depth_m
            / intrinsics.focal_y_px,
            depth_m,
        ])
        level_point = rotation @ optical_point
        signed_plane_distance_m = float(
            level_point @ plane_normal + plane_offset)
        if signed_plane_distance_m < -2.0 * config.plane_distance_m:
            return True
        if abs(depth_m - edge_depth_m) > 2.0 * config.plane_distance_m:
            return False
        if signed_plane_distance_m >= -config.plane_distance_m:
            return False
    return False


def _visible_front_edge(
        depth_mm: np.ndarray, points_optical: np.ndarray,
        points_level: np.ndarray, columns: np.ndarray, rows: np.ndarray,
        plane_mask: np.ndarray, config: TableEdgeConfig,
        intrinsics: CameraIntrinsics, rotation: np.ndarray,
        plane_normal: np.ndarray, plane_offset: float,
        ) -> tuple[np.ndarray, np.ndarray, float] | None:
    """Fit the lower visible boundary of the table plane in level XY."""
    edge_indices = []
    plane_indices = np.flatnonzero(plane_mask)
    plane_columns = columns[plane_mask].astype(np.int64)
    plane_rows = rows[plane_mask].astype(np.int64)
    if len(plane_indices) == 0:
        return None
    # Group once by column instead of scanning every plane point for every
    # column.  ``order`` retains each point's local index into ``plane_indices``.
    order = np.lexsort((plane_rows, plane_columns))
    ordered_columns = plane_columns[order]
    column_breaks = np.flatnonzero(np.diff(ordered_columns)) + 1
    for ordered in np.split(order, column_breaks):
        column = plane_columns[ordered[0]]
        breaks = np.flatnonzero(
            np.diff(plane_rows[ordered]) > 2 * config.pixel_step) + 1
        # A background wall can cross the top plane in a separate thin band.
        # Follow the dominant connected top surface rather than that lower band.
        if len(breaks):
            boundaries = np.empty(len(breaks) + 2, dtype=np.int64)
            boundaries[0], boundaries[-1] = 0, len(ordered)
            boundaries[1:-1] = breaks
            run_index = int(np.argmax(np.diff(boundaries)))
            top_run = ordered[boundaries[run_index]:boundaries[run_index + 1]]
        else:
            top_run = ordered
        if len(top_run) < 4:
            continue
        local_index = int(top_run[-1])
        point_index = int(plane_indices[local_index])
        row = int(rows[point_index])
        point_depth_m = float(points_optical[point_index, 2])
        if row < 2 * config.pixel_step:
            continue
        if row >= depth_mm.shape[0] - 2 * config.pixel_step:
            continue
        if not (
                point_depth_m
                > config.minimum_depth_m + config.plane_distance_m
                and point_depth_m
                < config.maximum_depth_m - config.plane_distance_m):
            continue
        if not _has_depth_discontinuity_below(
                depth_mm, row, int(column), intrinsics, rotation,
                plane_normal, plane_offset, config):
            continue
        edge_indices.append(point_index)
    minimum_support = max(8, config.minimum_inliers // 20)
    if len(edge_indices) < minimum_support:
        return None

    edge_points_level = points_level[np.asarray(edge_indices)]
    planar = edge_points_level[:, :2]
    center = np.mean(planar, axis=0)
    _, _, vectors = np.linalg.svd(planar - center, full_matrices=False)
    residuals = np.abs((planar - center) @ vectors[1])
    line_mask = residuals <= max(
        2.0 * config.maximum_edge_rms_m, config.plane_distance_m)
    if int(np.count_nonzero(line_mask)) < minimum_support:
        return None
    planar = planar[line_mask]
    center = np.mean(planar, axis=0)
    _, _, vectors = np.linalg.svd(planar - center, full_matrices=False)
    basis = vectors[0]
    signed = (planar - center) @ basis
    endpoint_level = np.array([
        center + basis * np.min(signed),
        center + basis * np.max(signed),
    ])
    edge_length_m = float(np.linalg.norm(
        endpoint_level[1] - endpoint_level[0]))
    residuals = np.abs((planar - center) @ vectors[1])
    edge_rms_m = float(np.sqrt(np.mean(residuals ** 2)))
    if edge_length_m < config.minimum_edge_length_m:
        return None
    if edge_rms_m > config.maximum_edge_rms_m:
        return None

    # Reconstruct each endpoint on the fitted plane.  A single median z loses
    # the plane's x/y slope and changes the optical geometry used downstream.
    endpoint_z = (
        -(plane_normal[0] * endpoint_level[:, 0]
          + plane_normal[1] * endpoint_level[:, 1]
          + plane_offset) / plane_normal[2]
    )
    endpoint_level_3d = np.column_stack((endpoint_level, endpoint_z))
    endpoint_optical = endpoint_level_3d @ rotation
    return endpoint_optical, line_mask, edge_rms_m


def detect_table_edge(
        depth_mm: np.ndarray, intrinsics: CameraIntrinsics,
        config: TableEdgeConfig, *,
        plane_result: dict | None = None) -> TableEdgeDetection | None:
    """Detect a table front edge using only its visible horizontal top."""
    if plane_result is not None:
        plane_result.clear()
    _validate_table_config(config)
    points_optical, columns, rows = depth_points(
        depth_mm, intrinsics, config)
    return _detect_table_edge_points(
        depth_mm, intrinsics, config, points_optical, columns, rows,
        plane_result=plane_result)


def _detect_table_edge_points(depth_mm, intrinsics, config,
                              points_optical, columns, rows,
                              plane_result=None):
    """Fit one component while retaining the original background evidence."""
    if plane_result is not None:
        plane_result.clear()
    if len(points_optical) < config.minimum_inliers:
        return None

    rotation = _optical_to_level_rotation(config)
    points_level = points_optical @ rotation.T
    plane = _strongest_table_plane(points_level, config)
    if plane is None:
        return None
    normal_level, offset_level = plane
    initial_distances = np.abs(points_level @ normal_level + offset_level)
    initial_mask = initial_distances <= config.plane_distance_m
    if int(np.count_nonzero(initial_mask)) < config.minimum_inliers:
        return None
    refined_normal, refined_offset = _fit_plane(points_level[initial_mask])
    if refined_normal[2] < 0.0:
        refined_normal = -refined_normal
        refined_offset = -refined_offset
    distances = np.abs(points_level @ refined_normal + refined_offset)
    plane_mask = distances <= config.plane_distance_m
    refined = points_level[plane_mask]
    if len(refined) < config.minimum_inliers:
        return None
    if not _horizontal_plane_is_valid(refined_normal, refined, config):
        return None

    plane_rms_m = float(np.sqrt(np.mean(distances[plane_mask] ** 2)))
    normal_optical = rotation.T @ refined_normal
    normal_optical /= np.linalg.norm(normal_optical)
    plane_centroid_level = np.mean(refined, axis=0)
    plane_centroid_optical = plane_centroid_level @ rotation
    table_height_m = (
        config.camera_height_m + float(plane_centroid_level[2]))
    if plane_result is not None:
        plane_result.update({
            'source': 'current_depth_tabletop_fit',
            'plane_normal_optical': [
                float(value) for value in normal_optical],
            'plane_point_optical_m': [
                float(value) for value in plane_centroid_optical],
            'plane_rms_m': plane_rms_m,
            'inlier_count': int(len(refined)),
            'candidate_count': int(len(points_optical)),
            'table_height_m': table_height_m,
        })

    edge = _visible_front_edge(
        depth_mm, points_optical, points_level, columns, rows, plane_mask,
        config, intrinsics, rotation, refined_normal, refined_offset)
    if edge is None:
        return None
    endpoints_optical, _edge_mask, edge_rms_m = edge
    edge_center = np.mean(endpoints_optical, axis=0)
    edge_vector = endpoints_optical[1] - endpoints_optical[0]
    edge_length_m = float(np.linalg.norm(edge_vector))
    edge_center_level = edge_center @ rotation.T
    table_height_m = config.camera_height_m + float(edge_center_level[2])
    edge_angle_rad = math.atan2(float(edge_vector[2]), float(edge_vector[0]))
    inlier_ratio = len(refined) / len(points_optical)
    residual_quality = max(
        0.0, 1.0 - plane_rms_m / config.plane_distance_m)
    edge_quality = max(
        0.0, 1.0 - edge_rms_m / config.maximum_edge_rms_m)
    support_quality = min(1.0, inlier_ratio / 0.20)
    confidence = (
        0.40 * residual_quality
        + 0.35 * edge_quality
        + 0.25 * support_quality
    )
    return TableEdgeDetection(
        center_x_m=float(edge_center[0]),
        center_y_m=float(edge_center[1]),
        front_distance_m=float(edge_center[2]),
        width_m=edge_length_m,
        depth_extent_m=float(np.ptp(refined[:, 0])),
        edge_angle_rad=edge_angle_rad,
        plane_normal=tuple(float(value) for value in normal_optical),
        plane_rms_m=plane_rms_m,
        inlier_count=int(len(refined)),
        candidate_count=int(len(points_optical)),
        confidence=float(confidence),
        surface_kind='table_edge',
        height_m=table_height_m,
        edge_endpoints_optical_m=tuple(
            tuple(float(value) for value in endpoint)
            for endpoint in endpoints_optical),
        table_height_m=table_height_m,
        edge_rms_m=edge_rms_m,
    )
