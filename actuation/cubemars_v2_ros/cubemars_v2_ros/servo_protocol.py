#!/usr/bin/env python3
"""
CubeMars AK-series *Servo Mode* CAN protocol helpers.

This module implements the CubeMars "Servo Mode" communication protocol as
described in the *AK Series Module Driver Manual* (V1.0.18), section 5.

It is intentionally free of any ROS 2 / python-can dependencies so that the
pure protocol logic (bit packing, unit conversion, feedback parsing) can be
unit-tested on its own.  ``motor_node.py`` imports from here.

Servo Mode vs. MIT Mode
-----------------------
Servo mode is a completely different protocol from the MIT ("Mini-Cheetah")
protocol.  The two most important differences for this codebase are:

1. **No Kp / Kd.**  The position/velocity control gains live inside the driver
   (configured over R-LINK) and are *not* part of any CAN frame.  Each frame
   commands exactly one quantity: duty, current, ERPM, or position.

2. **Different notation and units.**  MIT mode speaks radians / rad·s⁻¹ / N·m
   directly at the output shaft.  Servo mode uses:

   * **Position**  – degrees at the **output shaft** (gearbox already applied
     by the firmware).  Command range ±36000°, feedback range ±3200°.
   * **Speed**     – **ERPM** (electrical RPM of the rotor), i.e.
     ``mechanical_output_rpm × gear_ratio × pole_pairs``.
   * **Current**   – amperes of motor phase current *Iq* (pre-gearbox).
     Output torque ``τ = Iq × Kt × gear_ratio``.

The whole SELQIE leg-control stack works in radians / rad·s⁻¹ / N·m at the
output joint, so the node keeps its ROS interface unchanged and converts to
and from servo units here.

Frame format
------------
All servo command/feedback frames are **CAN 2.0 extended (29-bit)** frames::

    CAN ID = (packet_id << 8) | source_node_id      # 21-bit packet id, 8-bit node id

Command payloads are big-endian (MSB first), matching ``buffer_append_int32``
/ ``buffer_append_int16`` in the manual.
"""

import math

# ===================== SERVO PACKET IDS (manual §5.1) =====================

CAN_PACKET_SET_DUTY = 0           # Duty cycle mode
CAN_PACKET_SET_CURRENT = 1        # Current (Iq / torque) loop mode
CAN_PACKET_SET_CURRENT_BRAKE = 2  # Current brake mode
CAN_PACKET_SET_RPM = 3            # Speed (ERPM) loop mode
CAN_PACKET_SET_POS = 4            # Position loop mode
CAN_PACKET_SET_ORIGIN_HERE = 5    # Set origin (0 = temporary, 1 = permanent)
CAN_PACKET_SET_POS_SPD = 6        # Position-velocity loop mode

# Feedback (status) frames uploaded by the motor (manual §5.2.1).
CAN_PACKET_STATUS = 0x29          # Real-time servo state feedback

# ===================== COMMAND SCALING (manual §5.1) =====================

DUTY_SCALE = 100000.0             # duty  -> int32, buffer = duty  * 100000
CURRENT_SCALE = 1000.0            # A     -> int32, buffer = amps  * 1000  (mA)
RPM_SCALE = 1.0                   # ERPM  -> int32, buffer = erpm  * 1
POS_SCALE = 10000.0               # deg   -> int32, buffer = deg   * 10000
POS_SPD_SPEED_SCALE = 10.0        # ERPM      -> int16, buffer = erpm  / 10 (signed)
POS_SPD_ACCEL_SCALE = 10.0        # ERPM/s^2  -> int16, buffer = accel / 10 (unsigned)

# Encoding limits (int32 command fields) from the manual.
CURRENT_MAX_A = 60.0              # ±60 A  -> ±60000
RPM_MAX_ERPM = 100000.0          # ±100000 ERPM
POS_MAX_DEG = 36000.0            # ±36000° (±100 turns of the output shaft)
POS_SPD_SPEED_MAX_ERPM = 327680.0  # signed int16 * 10 (-327680..327670)
POS_SPD_ACCEL_MAX = 327670.0       # unsigned int16 * 10 (0..327670); 1 unit == 10 ERPM/s^2

# ===================== FEEDBACK SCALING (manual §5.2.1) =====================

FB_POS_SCALE = 0.1                # int16 -> degrees   (-32000..32000 = -3200..3200°)
FB_SPD_SCALE = 10.0               # int16 -> ERPM      (-32000..32000 = -320000..320000)
FB_CUR_SCALE = 0.01               # int16 -> amperes   (-6000..6000   = -60..60 A)
# Feedback position wraps within the int16 range: -3200°..+3200°.
FB_POS_SPAN_DEG = (32000 - (-32000)) * FB_POS_SCALE  # 6400.0 degrees

_INT16_MIN = -32768
_INT16_MAX = 32767
_INT32_MIN = -2147483648
_INT32_MAX = 2147483647


# ===================== LOW LEVEL HELPERS =====================


def clamp(x, lo, hi):
    """Clamp ``x`` to the inclusive range ``[lo, hi]``."""
    return lo if x < lo else hi if x > hi else x


def servo_can_id(packet_id, node_id):
    """Build the 29-bit extended CAN id for a servo frame.

    ``CAN ID = (packet_id << 8) | node_id`` (manual §5.1).
    """
    return ((int(packet_id) << 8) | (int(node_id) & 0xFF)) & 0x1FFFFFFF


def _pack_int32_be(value):
    """Pack a signed 32-bit integer big-endian, saturating on overflow."""
    value = int(clamp(int(round(value)), _INT32_MIN, _INT32_MAX))
    return (value & 0xFFFFFFFF).to_bytes(4, "big")


def _pack_int16_be(value):
    """Pack a signed 16-bit integer big-endian, saturating on overflow."""
    value = int(clamp(int(round(value)), _INT16_MIN, _INT16_MAX))
    return (value & 0xFFFF).to_bytes(2, "big")


def _to_int16(hi, lo):
    """Combine two bytes into a signed 16-bit integer (big-endian)."""
    v = ((hi & 0xFF) << 8) | (lo & 0xFF)
    return v - 0x10000 if v & 0x8000 else v


def _to_int8(b):
    """Interpret a byte as a signed 8-bit integer."""
    return b - 0x100 if b & 0x80 else b


# ===================== UNIT CONVERSIONS =====================
#
# Position: output-shaft radians  <-> output-shaft degrees   (no gear factor).
# Velocity: output-shaft rad/s     <-> rotor ERPM             (gear * pole_pairs).
# Torque:   output-shaft N·m       <-> phase current Iq (A)   (Kt * gear).


def rad_to_deg(rad):
    """Output-shaft radians -> output-shaft degrees."""
    return rad * (180.0 / math.pi)


def deg_to_rad(deg):
    """Output-shaft degrees -> output-shaft radians."""
    return deg * (math.pi / 180.0)


def rads_to_erpm(rad_s, gear_ratio, pole_pairs):
    """Output-shaft rad·s⁻¹ -> rotor electrical RPM (ERPM).

    ``ERPM = rad_s * (60 / 2π) * gear_ratio * pole_pairs``
    """
    return rad_s * (60.0 / (2.0 * math.pi)) * gear_ratio * pole_pairs


def erpm_to_rads(erpm, gear_ratio, pole_pairs):
    """Rotor ERPM -> output-shaft rad·s⁻¹ (inverse of :func:`rads_to_erpm`)."""
    denom = gear_ratio * pole_pairs
    if denom == 0:
        return 0.0
    return erpm * (2.0 * math.pi / 60.0) / denom


def torque_to_current(torque_nm, kt, gear_ratio):
    """Output-shaft torque (N·m) -> motor phase current Iq (A).

    ``τ = Iq * Kt * gear_ratio``  ->  ``Iq = τ / (Kt * gear_ratio)``
    """
    denom = kt * gear_ratio
    if denom == 0:
        return 0.0
    return torque_nm / denom


def current_to_torque(current_a, kt, gear_ratio):
    """Motor phase current Iq (A) -> output-shaft torque (N·m)."""
    return current_a * kt * gear_ratio


# ===================== COMMAND PACKING =====================
#
# Each function returns ``(can_id, data_bytes)`` for an extended CAN frame.


def pack_duty(node_id, duty):
    """Duty-cycle mode frame.  ``duty`` in [-1, 1]."""
    data = _pack_int32_be(clamp(duty, -1.0, 1.0) * DUTY_SCALE)
    return servo_can_id(CAN_PACKET_SET_DUTY, node_id), data


def pack_current(node_id, current_a, max_a=CURRENT_MAX_A):
    """Current-loop (torque) mode frame.  ``current_a`` in amperes."""
    current_a = clamp(current_a, -max_a, max_a)
    data = _pack_int32_be(current_a * CURRENT_SCALE)
    return servo_can_id(CAN_PACKET_SET_CURRENT, node_id), data


def pack_current_brake(node_id, current_a, max_a=CURRENT_MAX_A):
    """Current-brake mode frame.  ``current_a`` in amperes (>= 0)."""
    current_a = clamp(current_a, 0.0, max_a)
    data = _pack_int32_be(current_a * CURRENT_SCALE)
    return servo_can_id(CAN_PACKET_SET_CURRENT_BRAKE, node_id), data


def pack_rpm(node_id, erpm, max_erpm=RPM_MAX_ERPM):
    """Speed-loop mode frame.  ``erpm`` is electrical RPM."""
    erpm = clamp(erpm, -max_erpm, max_erpm)
    data = _pack_int32_be(erpm * RPM_SCALE)
    return servo_can_id(CAN_PACKET_SET_RPM, node_id), data


def pack_pos(node_id, pos_deg, max_deg=POS_MAX_DEG):
    """Position-loop mode frame.  ``pos_deg`` is output-shaft degrees."""
    pos_deg = clamp(pos_deg, -max_deg, max_deg)
    data = _pack_int32_be(pos_deg * POS_SCALE)
    return servo_can_id(CAN_PACKET_SET_POS, node_id), data


def pack_origin(node_id, mode=0):
    """Set-origin frame.  ``mode``: 0=temporary, 1=permanent, 2=restore."""
    data = bytes([int(mode) & 0xFF])
    return servo_can_id(CAN_PACKET_SET_ORIGIN_HERE, node_id), data


def pack_pos_spd(node_id, pos_deg, speed_erpm, accel):
    """Position-velocity mode frame.

    Per the manual's transmit-data table:
      * position: int32, ``pos_deg * 10000`` (deg, output shaft), signed.
      * speed:    int16, ``speed_erpm / 10`` (ERPM), signed (-327680..327670).
      * accel:    int16, ``accel / 10``, non-negative (0..327670); the manual
        gives its unit as electrical RPM/s^2.
    """
    pos_deg = clamp(pos_deg, -POS_MAX_DEG, POS_MAX_DEG)
    speed_erpm = clamp(speed_erpm, -POS_SPD_SPEED_MAX_ERPM, POS_SPD_SPEED_MAX_ERPM)
    accel = clamp(accel, 0.0, POS_SPD_ACCEL_MAX)  # accel field is unsigned (0..32767)
    data = (
        _pack_int32_be(pos_deg * POS_SCALE)
        + _pack_int16_be(speed_erpm / POS_SPD_SPEED_SCALE)
        + _pack_int16_be(accel / POS_SPD_ACCEL_SCALE)
    )
    return servo_can_id(CAN_PACKET_SET_POS_SPD, node_id), data


# ===================== FEEDBACK PARSING (manual §5.2.1) =====================


def parse_status_id(can_id):
    """Split an extended feedback CAN id into ``(packet_id, node_id)``."""
    return (can_id >> 8) & 0x1FFFFF, can_id & 0xFF


def parse_status(data):
    """Parse a servo-mode status (0x29) payload.

    Returns ``(pos_deg, spd_erpm, current_a, temp_c, error_code)`` or ``None``
    if the payload is not 8 bytes long.

    * position  int16 * 0.1  -> degrees
    * speed     int16 * 10    -> ERPM
    * current   int16 * 0.01  -> amperes
    * temp      int8          -> °C
    * error     uint8
    """
    if data is None or len(data) != 8:
        return None
    pos_deg = _to_int16(data[0], data[1]) * FB_POS_SCALE
    spd_erpm = _to_int16(data[2], data[3]) * FB_SPD_SCALE
    current_a = _to_int16(data[4], data[5]) * FB_CUR_SCALE
    temp_c = _to_int8(data[6])
    error = data[7] & 0xFF
    return pos_deg, spd_erpm, current_a, temp_c, error
