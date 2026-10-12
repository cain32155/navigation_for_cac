"""crater_detector: hits seen before the map frame exists are kept (in odom) and
moved into the map at the first marker fix, with the usual min_hits rule."""
import math

import numpy as np
import pytest
import rclpy

from cac_sim.crater_detector import CraterDetector


@pytest.fixture
def det():
    rclpy.init()
    node = CraterDetector()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _patch(cx, cy, n=6, size=0.10):
    g = np.linspace(-size / 2, size / 2, n)
    X, Y = np.meshgrid(cx + g, cy + g)
    return np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, -0.05)])


def _T(x, y, yaw):
    T = np.eye(4)
    T[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    T[:2, 3] = [x, y]
    return T


def _centres(det):
    cs = det.get_parameter('cell_size').value
    return np.array([[(i + 0.5) * cs, (j + 0.5) * cs] for i, j in det._confirmed])


def test_pre_fix_hits_land_in_map_after_fix(det):
    for _ in range(2):                                   # two frames -> min_hits
        det._count_hits(det._odom_hits, _patch(1.0, 0.5))
    assert not det._confirmed                            # nothing published before the fix
    T_map_odom = _T(-3.3, 1.2, math.pi / 2)
    det._merge_odom_hits(T_map_odom)
    assert det._confirmed and not det._odom_hits
    expect = (T_map_odom @ [1.0, 0.5, 0.0, 1.0])[:2]     # (-3.8, 2.2)
    d = np.hypot(*(_centres(det) - expect).T)
    assert d.max() < 0.10 + 0.05                         # patch half-diagonal + one cell


def test_single_pre_fix_frame_is_not_confirmed(det):
    det._count_hits(det._odom_hits, _patch(1.0, 0.5))
    det._merge_odom_hits(_T(0.0, 0.0, 0.0))
    assert not det._confirmed and det._hits


def test_pre_fix_and_post_fix_frames_add_up(det):
    det._count_hits(det._odom_hits, _patch(1.0, 0.5))
    det._merge_odom_hits(_T(0.0, 0.0, 0.0))
    det._count_hits(det._hits, _patch(1.0, 0.5))
    det._confirmed |= {k for k, n in det._hits.items() if n >= det.get_parameter('min_hits').value}
    assert det._confirmed
