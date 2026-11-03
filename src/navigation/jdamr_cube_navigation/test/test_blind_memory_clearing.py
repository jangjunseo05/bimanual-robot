import math
from types import SimpleNamespace

import pytest

from jdamr_cube_navigation.reverse_parking import remembered_blind_points


def _scan(rays, *, stamp_s=None, frame_id='laser'):
    header = None if stamp_s is None else SimpleNamespace(
        frame_id=frame_id,
        stamp=SimpleNamespace(sec=math.floor(stamp_s),
                              nanosec=round(stamp_s % 1 * 1e9)))
    return SimpleNamespace(ranges=rays, angle_min=-.02, angle_increment=.01,
                           range_min=.28, range_max=12., header=header)


def test_blind_memory_requires_newer_valid_free_space():
    old=_scan([math.nan, math.nan, .5, math.nan, math.nan])
    pose=(0.,0.,0.); laser=(0.,0.,0.); current=(.3,0.,0.)
    assert remembered_blind_points([(pose,old)],current,laser,.28)
    for rays, cleared in [([1.]*5,True), ([math.inf]*5,False),
                          ([math.nan]*5,False), ([.4]*5,False),
                          ([.51]*5,False), ([1.,1.,.5,1.,1.],False)]:
        found=remembered_blind_points([(pose,old),(pose,_scan(rays))],current,laser,.28)
        assert bool(found) != cleared, (rays,found)
    # The same point cannot be cleared after it has entered the blind range.
    assert remembered_blind_points([(pose,old),(current,_scan([1.]*5))],current,laser,.28)
    # An obstacle observed again after free-space evidence is retained.
    assert remembered_blind_points([(pose,old),(pose,_scan([1.]*5)),(pose,old)],current,laser,.28)
    # A 90-degree vehicle rotation uses the newer laser frame for its ray evidence.
    rot=SimpleNamespace(ranges=[1.]*5,angle_min=-math.pi/2-.02,angle_increment=.01,
                        range_min=.28,range_max=12.)
    assert not remembered_blind_points([(pose,old),((0.,0.,math.pi/2),rot)],current,laser,.28)


def test_visible_free_space_is_remembered_before_point_enters_blind_range():
    source = _scan([math.nan, math.nan, .5, math.nan, math.nan], stamp_s=1.)
    free = _scan([1.] * 5, stamp_s=2.)
    source_pose = (0., 0., 0.)
    laser = (0., 0., 0.)
    cleared = set()

    # At x=.175 the old return is still visible, so three farther finite rays
    # prove free space even though it does not yet need blind-range memory.
    assert remembered_blind_points(
        [(source_pose, source)], (.175, 0., 0.), laser, .28,
        current_scan=free, current_scan_pose=(.175, 0., 0.),
        cleared_sources=cleared) == []
    assert cleared

    # Later, after the same odom point enters range_min, only that proven source
    # stays cleared. Without the retained evidence it would block again.
    blind_pose = (.3, 0., 0.)
    assert remembered_blind_points(
        [(source_pose, source)], blind_pose, laser, .28)
    assert remembered_blind_points(
        [(source_pose, source)], blind_pose, laser, .28,
        current_scan=_scan([math.nan] * 5, stamp_s=3.),
        cleared_sources=cleared) == []


def test_unproven_current_scan_does_not_clear_blind_memory():
    source_pose = (0., 0., 0.)
    source = _scan([math.nan, math.nan, .5, math.nan, math.nan], stamp_s=2.)
    laser = (0., 0., 0.)
    visible_pose = (.15, 0., 0.)
    blind_pose = (.3, 0., 0.)
    cases = (
        _scan([math.inf] * 5, stamp_s=3.),
        _scan([math.nan] * 5, stamp_s=3.),
        _scan([.36] * 5, stamp_s=3.),  # less than the 2 cm clearance margin
        _scan([1.] * 5, stamp_s=1.),  # older than the source
        _scan([1.] * 5, stamp_s=3., frame_id='other_laser'),
    )
    for current in cases:
        cleared = set()
        remembered_blind_points(
            [(source_pose, source)], visible_pose, laser, .28,
            current_scan=current, current_scan_pose=visible_pose,
            cleared_sources=cleared)
        assert not cleared
        assert remembered_blind_points(
            [(source_pose, source)], blind_pose, laser, .28,
            cleared_sources=cleared)

    cleared = set()
    assert remembered_blind_points(
        [(source_pose, source)], blind_pose, laser, .28,
        current_scan=_scan([1.] * 5, stamp_s=3.),
        current_scan_pose=blind_pose, cleared_sources=cleared)
    assert not cleared


def test_last_visible_clearance_failure_survives_entry_into_blind_range():
    source_pose = (0., 0., 0.)
    source = _scan([math.nan, math.nan, .5, math.nan, math.nan], stamp_s=1.)
    blocked = _scan([.34] * 5, stamp_s=2.)
    laser = (0., 0., 0.)
    failures = {}

    remembered_blind_points(
        [(source_pose, source)], (.175, 0., 0.), laser, .28,
        current_scan=blocked, current_scan_pose=(.175, 0., 0.),
        clearance_failures=failures)
    diagnostic = next(iter(failures.values()))
    assert diagnostic['reason'] == 'three_finite_rays_not_beyond_point'
    assert diagnostic['projected_distance_m'] == pytest.approx(.325)
    assert diagnostic['rays_m'] == pytest.approx([.34, .34, .34])

    source_diagnostics = []
    found = remembered_blind_points(
        [(source_pose, source)], (.3, 0., 0.), laser, .28,
        current_scan=_scan([math.nan] * 5, stamp_s=3.),
        current_scan_pose=(.3, 0., 0.), clearance_failures=failures,
        source_diagnostics=source_diagnostics)
    assert found
    assert source_diagnostics == [diagnostic]


def test_completed_run_turn_projection_uses_pre_blind_free_space_evidence():
    """Replay the 2026-11-02 remembered point geometry that paused for 99 s."""
    laser = (-.01, 0., -3.0802057575642006)
    source_pose = (2.9444931735253106, -1.47144875674165, -.8076501805279691)
    final_pose = (3.289715937557809, -.9051394261347896, -2.0920433900366833)
    source_ranges = [math.nan] * 900
    source_ranges[57] = .9169999957084656
    source = SimpleNamespace(
        ranges=source_ranges, angle_min=-.8393942713737488,
        angle_increment=-.006734389346092939, range_min=.28, range_max=16.,
        header=SimpleNamespace(frame_id='scan', stamp=SimpleNamespace(
            sec=1793598814, nanosec=250439400)))
    # Ninety percent through the measured straight approach the old odom point
    # is still 31.5 cm from the laser and therefore observable.
    initial_pose = (3.072882484038009, -1.2719780732145782, -2.1395721452459147)
    visible_pose = tuple(
        initial_pose[i] + .9 * (final_pose[i] - initial_pose[i]) for i in range(3))
    free_ranges = [math.nan] * 900
    # These three rays correspond to the old point bearing and the fresh dock
    # surface about 9 cm behind it in the completed-run geometry.
    free_ranges[748:751] = [.405, .404, .403]
    free = SimpleNamespace(
        ranges=free_ranges, angle_min=source.angle_min,
        angle_increment=source.angle_increment, range_min=.28, range_max=16.,
        header=SimpleNamespace(frame_id='scan', stamp=SimpleNamespace(
            sec=1793598834, nanosec=0)))
    cleared = set()

    remembered_blind_points(
        [(source_pose, source)], visible_pose, laser, .28,
        current_scan=free, current_scan_pose=visible_pose,
        cleared_sources=cleared)
    projected_without_evidence = remembered_blind_points(
        [(source_pose, source)], final_pose, laser, .28)
    assert projected_without_evidence[0] == pytest.approx(
        (-.24993977514127388, -.13904403619373676))
    assert remembered_blind_points(
        [(source_pose, source)], final_pose, laser, .28,
        cleared_sources=cleared) == []
