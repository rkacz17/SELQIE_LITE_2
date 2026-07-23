#!/usr/bin/env python3
"""
Unit tests for the CubeMars servo-mode CAN protocol helpers.

These tests exercise the pure protocol layer (``servo_protocol``) with no ROS
or python-can dependency: bit packing against the manual's reference routines,
unit conversions (rad<->deg, rad/s<->ERPM, Nm<->A), round-trips, and feedback
parsing.
"""

import math
import struct

import pytest

from cubemars_v2_ros import servo_protocol as sp


# ------------------------- reference implementations -------------------------
# Mirror the manual's C routines (buffer_append_int32 is big-endian) so the
# tests are independent of servo_protocol's own packing code.


def ref_int32_be(value):
    v = int(value) & 0xFFFFFFFF
    return bytes([(v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF])


def ref_int16_be(value):
    v = int(value) & 0xFFFF
    return bytes([(v >> 8) & 0xFF, v & 0xFF])


# ------------------------------- CAN id layout -------------------------------


def test_can_id_layout():
    # CAN ID = (packet_id << 8) | node_id
    assert sp.servo_can_id(sp.CAN_PACKET_SET_POS, 1) == (4 << 8) | 1
    assert sp.servo_can_id(sp.CAN_PACKET_SET_RPM, 0x07) == (3 << 8) | 7
    # node id occupies only the low byte
    assert sp.servo_can_id(sp.CAN_PACKET_SET_CURRENT, 0x1FF) & 0xFF == 0xFF


@pytest.mark.parametrize(
    "packet_id,node_id",
    [(0, 0), (4, 1), (3, 7), (6, 255), (0x29, 3)],
)
def test_status_id_roundtrip(packet_id, node_id):
    can_id = sp.servo_can_id(packet_id, node_id)
    pid, nid = sp.parse_status_id(can_id)
    assert pid == packet_id
    assert nid == node_id


# ------------------------------- packing --------------------------------


def test_pack_duty_matches_manual():
    # buffer_append_int32(buffer, (int32_t)(duty * 100000.0))
    can_id, data = sp.pack_duty(3, 0.5)
    assert can_id == sp.servo_can_id(sp.CAN_PACKET_SET_DUTY, 3)
    assert data == ref_int32_be(int(0.5 * 100000.0))


def test_pack_current_matches_manual():
    # buffer_append_int32(buffer, (int32_t)(current * 1000.0))
    can_id, data = sp.pack_current(1, 12.5, max_a=60.0)
    assert can_id == sp.servo_can_id(sp.CAN_PACKET_SET_CURRENT, 1)
    assert data == ref_int32_be(int(12.5 * 1000.0))


def test_pack_current_negative():
    _, data = sp.pack_current(1, -3.0, max_a=60.0)
    assert data == ref_int32_be(int(-3.0 * 1000.0) & 0xFFFFFFFF)
    # decodes back to the negative value
    assert struct.unpack(">i", data)[0] == -3000


def test_pack_current_clamped_to_limit():
    _, data = sp.pack_current(1, 100.0, max_a=15.0)
    assert struct.unpack(">i", data)[0] == 15000  # 15 A -> 15000


def test_pack_rpm_matches_manual():
    # buffer_append_int32(buffer, (int32_t)rpm)
    can_id, data = sp.pack_rpm(7, 4200.0)
    assert can_id == sp.servo_can_id(sp.CAN_PACKET_SET_RPM, 7)
    assert struct.unpack(">i", data)[0] == 4200


def test_pack_rpm_clamped():
    _, data = sp.pack_rpm(7, 1e9)
    assert struct.unpack(">i", data)[0] == int(sp.RPM_MAX_ERPM)


def test_pack_pos_matches_manual():
    # buffer_append_int32(buffer, (int32_t)(pos * 10000.0))  pos in degrees
    can_id, data = sp.pack_pos(2, 90.0)
    assert can_id == sp.servo_can_id(sp.CAN_PACKET_SET_POS, 2)
    assert struct.unpack(">i", data)[0] == int(90.0 * 10000.0)


def test_pack_pos_negative_and_clamp():
    _, data = sp.pack_pos(2, -45.0)
    assert struct.unpack(">i", data)[0] == int(-45.0 * 10000.0)
    _, data = sp.pack_pos(2, 999999.0)  # beyond ±36000°
    assert struct.unpack(">i", data)[0] == int(sp.POS_MAX_DEG * 10000.0)


def test_pack_origin():
    can_id, data = sp.pack_origin(5, 1)
    assert can_id == sp.servo_can_id(sp.CAN_PACKET_SET_ORIGIN_HERE, 5)
    assert data == bytes([1])
    _, data0 = sp.pack_origin(5, 0)
    assert data0 == bytes([0])


def test_pack_pos_spd_layout():
    # 4-byte pos (deg*10000) + 2-byte speed (erpm/10) + 2-byte accel (a/10),
    # matching the manual's Position-Velocity transmit-data table.
    can_id, data = sp.pack_pos_spd(1, 10.0, 5000.0, 40000.0)
    assert can_id == sp.servo_can_id(sp.CAN_PACKET_SET_POS_SPD, 1)
    assert len(data) == 8
    assert struct.unpack(">i", data[0:4])[0] == int(10.0 * 10000.0)   # position 25..0 bits
    assert struct.unpack(">h", data[4:6])[0] == int(5000.0 / 10.0)    # speed high/low
    assert struct.unpack(">h", data[6:8])[0] == int(40000.0 / 10.0)   # accel high/low


def test_pack_pos_spd_signed_speed():
    # Speed is a signed int16 (-327680..327670 ERPM).
    _, data = sp.pack_pos_spd(1, 0.0, -12345.0, 10000.0)
    assert struct.unpack(">h", data[4:6])[0] == int(-12345.0 / 10.0)


def test_pack_pos_spd_accel_is_non_negative():
    # The accel field is unsigned (0..32767); a negative accel must clamp to 0,
    # never encode as a negative int16 the firmware would misread.
    _, data = sp.pack_pos_spd(1, 0.0, 1000.0, -5000.0)
    assert struct.unpack(">h", data[6:8])[0] == 0


def test_pack_pos_spd_accel_clamped_to_max():
    _, data = sp.pack_pos_spd(1, 0.0, 1000.0, 1e9)
    assert struct.unpack(">h", data[6:8])[0] == int(sp.POS_SPD_ACCEL_MAX / 10.0)
    assert struct.unpack(">h", data[6:8])[0] == 32767  # int16 max


# ------------------------------ conversions -----------------------------


def test_rad_deg_roundtrip():
    for rad in (-math.pi, -1.0, 0.0, 0.5, math.pi, 3.0 * math.pi):
        assert sp.deg_to_rad(sp.rad_to_deg(rad)) == pytest.approx(rad, abs=1e-9)
    assert sp.rad_to_deg(math.pi) == pytest.approx(180.0)
    assert sp.deg_to_rad(180.0) == pytest.approx(math.pi)


def test_rads_erpm_known_value():
    # 1 output rev/s = 60 output RPM.  With gear=10, pole_pairs=21:
    #   ERPM = 60 * 10 * 21 = 12600
    erpm = sp.rads_to_erpm(2.0 * math.pi, gear_ratio=10, pole_pairs=21)
    assert erpm == pytest.approx(60.0 * 10 * 21)


def test_rads_erpm_roundtrip():
    for rad_s in (-30.0, -1.0, 0.0, 5.5, 45.5):
        erpm = sp.rads_to_erpm(rad_s, 10, 21)
        back = sp.erpm_to_rads(erpm, 10, 21)
        assert back == pytest.approx(rad_s, abs=1e-9)


def test_erpm_to_rads_zero_denominator():
    assert sp.erpm_to_rads(1000.0, 0, 0) == 0.0


def test_torque_current_known_value():
    # AK40-10: Kt=0.056 Nm/A, gear=10 -> output torque per amp = 0.56 Nm/A
    cur = sp.torque_to_current(5.0, kt=0.056, gear_ratio=10)
    assert cur == pytest.approx(5.0 / (0.056 * 10))
    torque = sp.current_to_torque(cur, kt=0.056, gear_ratio=10)
    assert torque == pytest.approx(5.0, abs=1e-9)


def test_torque_current_zero_kt():
    assert sp.torque_to_current(5.0, kt=0.0, gear_ratio=10) == 0.0


# ---------------------------- feedback parsing --------------------------


def test_parse_status_matches_manual_scaling():
    # Build a frame: pos=1234 (=123.4deg), spd=200 (=2000 ERPM),
    # cur=-150 (=-1.5A), temp=42, err=0
    pos_raw = 1234
    spd_raw = 200
    cur_raw = -150
    data = (
        struct.pack(">h", pos_raw)
        + struct.pack(">h", spd_raw)
        + struct.pack(">h", cur_raw)
        + bytes([42 & 0xFF, 0])
    )
    pos_deg, spd_erpm, cur_a, temp, err = sp.parse_status(data)
    assert pos_deg == pytest.approx(pos_raw * 0.1)
    assert spd_erpm == pytest.approx(spd_raw * 10.0)
    assert cur_a == pytest.approx(cur_raw * 0.01)
    assert temp == 42
    assert err == 0


def test_parse_status_negative_temperature():
    data = struct.pack(">hhh", 0, 0, 0) + bytes([(-20) & 0xFF, 3])
    _, _, _, temp, err = sp.parse_status(data)
    assert temp == -20
    assert err == 3


def test_parse_status_bad_length():
    assert sp.parse_status(bytes([0, 0, 0])) is None
    assert sp.parse_status(None) is None


def test_parse_status_full_negative_position():
    # -32000 raw -> -3200 degrees (min feedback position)
    data = struct.pack(">hhh", -32000, -32000, -6000) + bytes([0, 0])
    pos_deg, spd_erpm, cur_a, _, _ = sp.parse_status(data)
    assert pos_deg == pytest.approx(-3200.0)
    assert spd_erpm == pytest.approx(-320000.0)
    assert cur_a == pytest.approx(-60.0)


# --------------------- end-to-end command/feedback loop ------------------


def test_position_command_feedback_consistency():
    """A commanded output angle should read back as the same angle.

    Command path: rad -> deg -> int32 (pos*10000).  Feedback path uses a
    coarser int16 (*0.1 deg) scale, so allow 0.1 deg of quantization.
    """
    node_id = 1
    target_rad = math.radians(37.5)
    _, cmd_data = sp.pack_pos(node_id, sp.rad_to_deg(target_rad))
    cmd_deg = struct.unpack(">i", cmd_data)[0] / sp.POS_SCALE
    assert cmd_deg == pytest.approx(37.5, abs=1e-4)

    # Emulate the motor echoing that position in a status frame (int16 * 0.1).
    fb_raw = int(round(cmd_deg / sp.FB_POS_SCALE))
    fb = struct.pack(">hhh", fb_raw, 0, 0) + bytes([25, 0])
    pos_deg, _, _, _, _ = sp.parse_status(fb)
    assert sp.deg_to_rad(pos_deg) == pytest.approx(target_rad, abs=math.radians(0.1))
