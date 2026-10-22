import importlib.util
from pathlib import Path
import struct

import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


SCRIPT = Path(__file__).resolve().parents[3] / "tools/perception/mesh_arm_geometric_snapshot_once.py"
spec = importlib.util.spec_from_file_location("mesh_arm_geometric_snapshot_once", SCRIPT)
snapshot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(snapshot)


def stamped(message, frame, stamp=123):
    message.header.frame_id = frame
    message.header.stamp.sec = stamp
    return message


def transform(target, source, stamp=123):
    message = stamped(TransformStamped(), target, stamp)
    message.child_frame_id = source
    message.transform.rotation.w = 1.0
    return message


def packet(messages):
    raw = bytearray(b"MESH8R1")
    for message in messages:
        serialized = serialize_message(message)
        raw += struct.pack("<I", len(serialized)) + serialized
    return bytes(raw)


def test_effective_description_and_meshes_are_reproducible():
    xml, manifest, frozen, links, provenance = snapshot.effective_xml_and_manifest()
    assert len(manifest) == len(links) == 13
    assert len(frozen) == 8
    assert len(set(links)) == 13
    assert snapshot.sha256(xml) == "b2d9d0d5f1366915ebe70583b8ebb048a7c445b97353b6b9424148f5644352d2"
    assert provenance["mesh_parameter_file"].startswith("/tmp/launch_params_")
    for entry in manifest:
        assert entry["sha256"] == snapshot.sha256(
            frozen[entry["uri"].removeprefix("package://thesis_description/")])


def test_packet_accepts_only_same_stamp_for_every_link_tf():
    links = ["link_a", "link_b"]
    messages = [
        stamped(Image(), "d435i_color_optical_frame"),
        stamped(CameraInfo(), "d435i_color_optical_frame"),
        stamped(PointCloud2(), "world"),
        transform("world", "d435i_depth_optical_frame"),
        transform("d435i_color_optical_frame", "world"),
        transform("world", "d435i_color_optical_frame"),
        transform("world", links[0]), transform("world", links[1]),
    ]
    parsed, rgb_delta, info_delta = snapshot.parse_packet(packet(messages), links)
    assert len(parsed) == 8
    assert rgb_delta == info_delta == 0
    messages[-1].header.stamp.nanosec = 1
    with pytest.raises(RuntimeError, match="TF frame or stamp mismatch"):
        snapshot.parse_packet(packet(messages), links)
