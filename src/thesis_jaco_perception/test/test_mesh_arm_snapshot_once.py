import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "mesh_arm_snapshot_once.py"
spec = importlib.util.spec_from_file_location("mesh_arm_snapshot_once", SCRIPT)
snapshot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(snapshot)


def message(nanoseconds):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
        sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)))


def test_sync_boundary_and_nearest():
    cloud = message(10_000_000_000)
    inside = message(10_033_000_000)
    outside = message(10_033_000_001)
    assert abs(snapshot.delta_ns(cloud, inside)) == snapshot.MAX_DELTA_NS
    assert abs(snapshot.delta_ns(cloud, outside)) > snapshot.MAX_DELTA_NS
    assert snapshot.nearest(cloud, [outside, inside]) is inside


def test_projection_and_behind_camera():
    transform = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0., y=0., z=0.),
        rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    info = SimpleNamespace(k=[100., 0., 320., 0., 100., 240., 0., 0., 1.], d=[])
    pixels = snapshot.project_world_points(
        np.array([[0., 0., 1.], [0.1, -0.2, 1.], [0., 0., -1.]]), transform, info)
    np.testing.assert_allclose(pixels[:2], [[320., 240.], [330., 220.]])
    assert np.isnan(pixels[2]).all()


def test_manual_mask_validation():
    mask = np.array([[0, 255], [0, 0]], dtype=np.uint8)
    assert snapshot.validate_manual_mask(mask, 2, 2)
    with pytest.raises(ValueError):
        snapshot.validate_manual_mask(np.array([[0, 1], [0, 0]], dtype=np.uint8), 2, 2)
    with pytest.raises(ValueError):
        snapshot.validate_manual_mask(mask, 3, 2)
    with pytest.raises(ValueError):
        snapshot.validate_manual_mask(np.zeros((2, 2), dtype=np.uint8), 2, 2)
