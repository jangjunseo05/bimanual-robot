"""Locate two dark strips whose upper ends meet the tabletop front edge."""

import math
import time
from itertools import combinations
from typing import Mapping

import cv2
import numpy as np

from jdamr_cube_navigation.rgbd_parking_reference import CameraIntrinsics, PlanarBaseErrorEstimate


class TableTapeReference:
    """Project strip upper ends onto the calibrated tabletop, not strip centres."""

    def __init__(self, rgb_info, basis, config):
        cv2.setNumThreads(1)
        self.intrinsics = CameraIntrinsics.from_value(rgb_info)
        self.rays_from_pixels = basis.R_base_from_optical @ np.linalg.inv(self.intrinsics.matrix)
        self.basis = basis
        self.config = dict(config)
        # Acquisition bounds cover a displaced arrival; final parking accuracy
        # is enforced by the station controller, not this candidate filter.
        self.config.setdefault('maximum_lateral_m', .40)
        for key in ('spacing_m', 'width_m', 'goal_front_from_base_m',
                    'spacing_tolerance_m', 'width_tolerance_m', 'maximum_yaw_rad',
                    'maximum_lateral_m'):
            value = self.config[key]
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError('invalid_tape_' + key)

    def tracking_checkpoint(self):
        return None

    def restore_tracking(self, checkpoint):
        pass  # Stateless detector; rejected observations leave no tracking state.

    def _point(self, u, v):
        ray = self.rays_from_pixels @ np.array([u, v, 1.])
        if ray[2] >= -1e-6:
            return None
        point = ray * (-self.basis.plane_offset_m / ray[2])
        return point[:2] + np.asarray(self.basis.camera_lever_xy_m)

    @staticmethod
    def _live_projection(surface, inverse_intrinsics):
        if not isinstance(surface, Mapping):
            raise ValueError('invalid_tape_surface')
        required = (
            'plane_normal_optical', 'plane_point_optical_m',
            'R_base_from_optical', 'camera_lever_xy_m', 'stamp_s',
            'rgb_stamp_s', 'source',
        )
        if any(key not in surface for key in required):
            raise ValueError('invalid_tape_surface')
        normal = np.asarray(
            surface['plane_normal_optical'], dtype=np.float64)
        plane_point = np.asarray(
            surface['plane_point_optical_m'], dtype=np.float64)
        rotation = np.asarray(
            surface['R_base_from_optical'], dtype=np.float64)
        lever = np.asarray(surface['camera_lever_xy_m'], dtype=np.float64)
        try:
            stamp = float(surface['stamp_s'])
            rgb_stamp = float(surface['rgb_stamp_s'])
        except (TypeError, ValueError):
            raise ValueError('invalid_tape_surface') from None
        if (normal.shape != (3,) or plane_point.shape != (3,)
                or rotation.shape != (3, 3) or lever.shape != (2,)
                or not np.all(np.isfinite(normal))
                or not np.all(np.isfinite(plane_point))
                or not np.all(np.isfinite(rotation))
                or not np.all(np.isfinite(lever))
                or not math.isfinite(stamp) or not math.isfinite(rgb_stamp)
                or not np.isclose(np.linalg.norm(normal), 1., atol=1e-6)
                or not np.allclose(
                    rotation @ rotation.T, np.eye(3), atol=1e-6, rtol=0.)
                or not np.isclose(np.linalg.det(rotation), 1., atol=1e-6)
                or surface['source'] != 'live_depth_table_plane'):
            raise ValueError('invalid_tape_surface')
        return {
            'normal': normal,
            'plane_point': plane_point,
            'rotation': rotation,
            'lever': lever,
            'inverse_intrinsics': inverse_intrinsics,
            'stamp_s': stamp,
            'rgb_stamp_s': rgb_stamp,
            'source': surface['source'],
        }

    @staticmethod
    def _live_point(u, v, projection):
        ray = projection['inverse_intrinsics'] @ np.array([u, v, 1.])
        denominator = float(projection['normal'] @ ray)
        if not math.isfinite(denominator) or abs(denominator) <= 1e-9:
            return None
        distance = float(
            projection['normal'] @ projection['plane_point']) / denominator
        if not math.isfinite(distance) or distance <= 0.:
            return None
        optical_point = distance * ray
        base_point = projection['rotation'] @ optical_point
        if not np.all(np.isfinite(base_point)):
            return None
        return base_point[:2] + projection['lever']

    def observe(self, image, *, surface=None, rgb_info=None):
        """Observe tape using either the saved basis or a live table plane."""
        started = time.perf_counter()
        quality = {'observation_method': 'table_front_tape', 'candidates': [],
                   'maximum_lateral_m': self.config['maximum_lateral_m'],
                   'lateral_rejected_candidates': []}

        def invalid(reason):
            quality['processing_s'] = time.perf_counter() - started
            return PlanarBaseErrorEstimate(False, reason), quality

        if image.ndim != 3 or image.dtype != np.uint8:
            return invalid('invalid_tape_image')
        if rgb_info is None:
            inverse_intrinsics = np.linalg.inv(self.intrinsics.matrix)
        else:
            try:
                frame_intrinsics = CameraIntrinsics.from_value(rgb_info)
            except (TypeError, ValueError, np.linalg.LinAlgError):
                return invalid('invalid_tape_rgb_info')
            if (frame_intrinsics.width != image.shape[1]
                    or frame_intrinsics.height != image.shape[0]):
                return invalid('invalid_tape_rgb_info_dimensions')
            inverse_intrinsics = np.linalg.inv(frame_intrinsics.matrix)
        projection = None
        if surface is not None:
            try:
                projection = self._live_projection(
                    surface, inverse_intrinsics)
            except (TypeError, ValueError, np.linalg.LinAlgError):
                return invalid('invalid_tape_surface')
            quality.update(
                projection_basis=projection['source'],
                depth_measurement_stamp_s=projection['stamp_s'],
                rgb_stamp_s=projection['rgb_stamp_s'],
                rgb_depth_delta_s=abs(
                    projection['rgb_stamp_s'] - projection['stamp_s']),
            )

        def point_from_pixel(u, v):
            if projection is not None:
                return self._live_point(u, v, projection)
            if rgb_info is None:
                return self._point(u, v)
            ray = self.basis.R_base_from_optical @ (
                inverse_intrinsics @ np.array([u, v, 1.]))
            if ray[2] >= -1e-6:
                return None
            point = ray * (-self.basis.plane_offset_m / ray[2])
            return point[:2] + np.asarray(self.basis.camera_lever_xy_m)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Isolate the upper transition: near contact the bottom of a strip can
        # merge with the dark chassis, so full-component height is not stable.
        mask = np.zeros_like(gray, dtype=np.uint8)
        mask[4:] = ((gray[4:] < 65) & (gray[:-4] > 120)).astype(np.uint8)
        candidates = []
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        full_mask = (gray < 65).astype(np.uint8)
        contours = list(contours) + list(cv2.findContours(
            full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0])
        seen_top_edges = set()
        for contour in contours:
            _, y, component_width, _ = cv2.boundingRect(contour)
            # The upper edge cannot be wider than its bounding box. Reject
            # tiny components before allocating arrays for every image speck.
            if component_width < 4 or y < 3 or y + 2 >= gray.shape[0]:
                continue
            upper = contour[:, 0, :]
            upper = upper[upper[:, 1] <= y + 1]
            x = int(upper[:, 0].min())
            w = int(upper[:, 0].max()) - x + 1
            h = 2
            top_edge = (x, y, w)
            if top_edge in seen_top_edges:
                continue
            # Transition and full-dark masks can describe the same strip top.
            # Every remaining test depends only on this geometry and source
            # pixels, so the duplicate would repeat the same work and outcome.
            seen_top_edges.add(top_edge)
            # Reject clipped components, arm silhouettes and isolated pixels.
            if not (4 <= w <= 90
                    and x >= w and x + 2 * w < gray.shape[1] and y >= 3
                    and y + h < gray.shape[0]
                    and np.mean(full_mask[y:y+3, x:x+w]) >= .60):
                continue
            above = gray[y-3:y, x:x+w]
            sides = np.concatenate((gray[y:y+h, x-w:x].ravel(),
                                    gray[y:y+h, x+w:x+2*w].ravel()))
            if np.median(above) < 100 or np.median(sides) < 95:
                continue
            u = float(x + (w - 1) / 2)
            point = point_from_pixel(u, float(y))
            ends = [point_from_pixel(x, y), point_from_pixel(x+w, y)]
            if point is None or any(p is None for p in ends):
                continue
            width = float(np.linalg.norm(ends[1] - ends[0]))
            if abs(width - self.config['width_m']) > self.config['width_tolerance_m']:
                continue
            if abs(point[1]) > self.config['maximum_lateral_m']:
                if len(quality['lateral_rejected_candidates']) < 4:
                    quality['lateral_rejected_candidates'].append({
                        'pixel': [u, y], 'base_xy_m': point.tolist()})
                continue
            if any(np.linalg.norm(point-c[2]) < .005 for c in candidates):
                continue
            candidates.append((u, float(y), point, width))
            quality['candidates'].append({'pixel': [u, y], 'base_xy_m': point.tolist(),
                                          'width_m': width})
        pairs = []
        for a, b in combinations(sorted(candidates, key=lambda c: c[0]), 2):
            vector = a[2] - b[2]  # image-left is base-left
            spacing = float(np.linalg.norm(vector))
            if abs(spacing - self.config['spacing_m']) > self.config['spacing_tolerance_m']:
                continue
            yaw = math.atan2(-vector[0], vector[1])
            if abs(yaw) > self.config['maximum_yaw_rad']:
                continue
            pairs.append((a, b, spacing, yaw))
        if len(pairs) != 1:
            return invalid('tape_pair_missing' if not pairs else 'tape_pair_ambiguous')
        a, b, spacing, yaw = pairs[0]
        center = (a[2] + b[2]) / 2
        goal = center - self.config['goal_front_from_base_m'] * np.array(
            [math.cos(yaw), math.sin(yaw)])
        quality.update(tape_pixels=[[a[0], a[1]], [b[0], b[1]]],
                       tape_spacing_m=spacing, tape_center_base_xy_m=center.tolist(),
                       processing_s=time.perf_counter()-started)
        return PlanarBaseErrorEstimate(
            True, 'observed_table_front_tape', forward_m=float(goal[0]),
            left_m=float(goal[1]), yaw_rad=yaw,
            basis_source=(projection['source'] if projection is not None
                          else self.basis.basis_source),
            camera_yaw_rad=(None if projection is not None
                            else self.basis.camera_yaw_rad),
            camera_lever_xy_m=(tuple(projection['lever'])
                               if projection is not None
                               else self.basis.camera_lever_xy_m)), quality
