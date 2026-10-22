"""Static validation tests for the shared JACO2 self-filter profile."""

import importlib.util
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch" / "jaco_rgbd_shadow.launch.py"
PROFILE = ROOT / "config" / "jaco_self_filter_capsules.yaml"


def _module():
    spec = importlib.util.spec_from_file_location("shadow_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _profile():
    return yaml.safe_load(PROFILE.read_text(encoding="utf-8"))


def _write_profile(tmp_path, profile):
    path = tmp_path / "capsules.yaml"
    path.write_text(yaml.safe_dump(profile), encoding="utf-8")
    return path


def test_profile_transcribes_the_core_jaco_capsule_geometry():
    module = _module()
    params = module._validate_capsule_profile(str(PROFILE))
    assert len(params["capsule_start_frames"]) == 6
    assert params["capsule_start_frames"][0] == "j2n6s300_link_base"
    assert params["capsule_end_frames"][-1] == "j2n6s300_end_effector"
    assert params["capsule_inner_radii_m"] == [0.105, 0.105, 0.085, 0.075, 0.085, 0.09]


@pytest.mark.parametrize("mutation", ("empty", "unequal", "nonpositive", "duplicate"))
def test_rejects_invalid_capsule_profiles(tmp_path, mutation):
    profile = _profile()
    params = profile["jaco_self_filter_node"]["ros__parameters"]
    if mutation == "empty":
        params["capsule_start_frames"] = []
    elif mutation == "unequal":
        params["capsule_end_frames"] = params["capsule_end_frames"][:-1]
    elif mutation == "nonpositive":
        params["capsule_inner_radii_m"][0] = 0.0
    else:
        params["capsule_end_frames"][1] = params["capsule_end_frames"][0]
        params["capsule_start_frames"][1] = params["capsule_start_frames"][0]
    with pytest.raises(RuntimeError):
        _module()._validate_capsule_profile(str(_write_profile(tmp_path, profile)))
