#!/usr/bin/env python3
"""
Integration tests for ``MotorNode`` servo-mode command handling.

``rclpy`` and ``python-can`` are not required to run these: lightweight stub
modules are injected into ``sys.modules`` so the real ``MotorNode`` class can be
constructed with a fake CAN bus that records transmitted frames.  This verifies
the full command -> servo-frame path (units and packet selection) that the leg
trajectory stack depends on.
"""

import math
import struct
import sys
import types

import pytest

from cubemars_v2_ros import servo_protocol as sp


# ------------------------------ module stubs ------------------------------


class _FakeMessage:
    def __init__(self, arbitration_id=0, data=b"", is_extended_id=False):
        self.arbitration_id = arbitration_id
        self.data = bytes(data)
        self.is_extended_id = is_extended_id


class _FakeBus:
    """Records sent frames; recv blocks-free returns nothing."""

    def __init__(self, *args, **kwargs):
        self.sent = []

    def set_filters(self, filters):
        self.filters = filters

    def send(self, msg):
        self.sent.append(msg)

    def recv(self, timeout=0.0):
        return None

    def shutdown(self):
        pass


def _install_stubs():
    if "can" in sys.modules and getattr(sys.modules["can"], "_selqie_stub", False):
        return

    can_mod = types.ModuleType("can")
    can_mod._selqie_stub = True
    can_mod.Message = _FakeMessage
    can_mod.CanError = type("CanError", (Exception,), {})
    iface = types.ModuleType("can.interface")
    iface.Bus = _FakeBus
    can_mod.interface = iface
    sys.modules["can"] = can_mod
    sys.modules["can.interface"] = iface

    # rclpy / rclpy.node
    rclpy_mod = types.ModuleType("rclpy")
    rclpy_mod.init = lambda *a, **k: None
    rclpy_mod.shutdown = lambda *a, **k: None
    rclpy_mod.spin = lambda *a, **k: None
    node_mod = types.ModuleType("rclpy.node")

    class _Param:
        def __init__(self, value):
            self.value = value

    class _Logger:
        def info(self, *a, **k):
            pass

        def warn(self, *a, **k):
            pass

        def error(self, *a, **k):
            pass

    class _Node:
        def __init__(self, name):
            if not hasattr(self, "_params"):
                self._params = {}
            if not hasattr(self, "_log"):
                self._log = _Logger()

        def declare_parameter(self, name, default):
            self._params.setdefault(name, default)

        def set_parameter_value(self, name, value):
            self._params[name] = value

        def get_parameter(self, name):
            return _Param(self._params.get(name))

        def get_logger(self):
            return self._log

        def create_publisher(self, *a, **k):
            return types.SimpleNamespace(publish=lambda *a, **k: None)

        def create_subscription(self, *a, **k):
            return None

        def create_timer(self, *a, **k):
            return None

    node_mod.Node = _Node
    rclpy_mod.node = node_mod
    sys.modules["rclpy"] = rclpy_mod
    sys.modules["rclpy.node"] = node_mod

    # message packages
    def _msg_class(fields, consts=None):
        def __init__(self):
            for f, v in fields.items():
                setattr(self, f, v)
        cls = type("Msg", (), {"__init__": __init__})
        for k, v in (consts or {}).items():
            setattr(cls, k, v)
        return cls

    actuation_msgs = types.ModuleType("actuation_msgs")
    actuation_msgs_msg = types.ModuleType("actuation_msgs.msg")
    MotorCommand = _msg_class(
        dict(control_mode=0, input_mode=0, pos_setpoint=0.0,
             vel_setpoint=0.0, torq_setpoint=0.0),
        dict(CONTROL_MODE_TORQUE=1, CONTROL_MODE_VELOCITY=2, CONTROL_MODE_POSITION=3,
             INPUT_MODE_PASSTHROUGH=1),
    )
    MotorEstimate = _msg_class(
        dict(pos_estimate=0.0, vel_estimate=0.0, torq_estimate=0.0))
    actuation_msgs_msg.MotorCommand = MotorCommand
    actuation_msgs_msg.MotorEstimate = MotorEstimate
    actuation_msgs.msg = actuation_msgs_msg
    sys.modules["actuation_msgs"] = actuation_msgs
    sys.modules["actuation_msgs.msg"] = actuation_msgs_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Float64MultiArray = _msg_class(dict(data=[]))
    std_msgs_msg.String = _msg_class(dict(data=""))
    std_msgs_msg.Int32 = _msg_class(dict(data=0))
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg

    motor_interfaces = types.ModuleType("motor_interfaces")
    motor_interfaces_msg = types.ModuleType("motor_interfaces.msg")
    motor_interfaces_msg.MotorState = _msg_class(
        dict(name="", position=0.0, abs_position=0.0, velocity=0.0,
             torque=0.0, current=0.0, temperature=0))
    motor_interfaces.msg = motor_interfaces_msg
    sys.modules["motor_interfaces"] = motor_interfaces
    sys.modules["motor_interfaces.msg"] = motor_interfaces_msg


@pytest.fixture
def make_node():
    _install_stubs()
    from cubemars_v2_ros import motor_node as mn

    created = []

    def _factory(**params):
        node = mn.MotorNode.__new__(mn.MotorNode)
        defaults = dict(
            can_interface="can0", can_id=1, motor_type="AK40-10",
            control_hz=100.0, joint_name="motor0", auto_start=True,
            reverse_polarity=False, pole_pairs=0, gear_ratio=0.0,
            cmd_timeout=0.0,
        )
        defaults.update(params)
        # Seed params before __init__ so the stub Node preserves them
        # (declare_parameter uses setdefault, so these overrides win).
        node._params = dict(defaults)
        mn.MotorNode.__init__(node)
        node._stop = True  # stop the rx thread promptly
        created.append(node)
        return node, mn

    yield _factory
    for node in created:
        node._stop = True


def _last_frame(node):
    return node.bus.sent[-1]


def test_position_command_emits_degrees(make_node):
    node, mn = make_node(motor_type="AK40-10", position_mode="pos")
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_POSITION
    cmd.pos_setpoint = 3.141592653589793 / 2  # 90 degrees
    node.on_motor_command(cmd)
    node._tick_control()

    frame = _last_frame(node)
    assert frame.is_extended_id
    pid, nid = sp.parse_status_id(frame.arbitration_id)
    assert pid == sp.CAN_PACKET_SET_POS
    assert nid == 1
    deg = struct.unpack(">i", frame.data)[0] / sp.POS_SCALE
    assert deg == pytest.approx(90.0, abs=1e-3)


def test_position_mode_defaults_to_pos(make_node):
    # No position_mode override -> node default should be plain SET_POS.
    node, mn = make_node(motor_type="AK40-10")
    assert node.position_mode == "pos"
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_POSITION
    cmd.pos_setpoint = 3.141592653589793 / 2
    node.on_motor_command(cmd)
    node._tick_control()

    frame = _last_frame(node)
    pid, _ = sp.parse_status_id(frame.arbitration_id)
    assert pid == sp.CAN_PACKET_SET_POS
    assert len(frame.data) == 4
    deg = struct.unpack(">i", frame.data)[0] / sp.POS_SCALE
    assert deg == pytest.approx(90.0, abs=1e-3)


def test_pos_spd_first_move_uses_min_speed(make_node):
    # pos_spd first move has no feed-forward history, but the min-speed floor
    # must still apply so the motor actually travels to the target.
    node, mn = make_node(motor_type="AK40-10", position_mode="pos_spd")
    assert node.position_mode == "pos_spd"
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_POSITION
    cmd.pos_setpoint = 3.141592653589793 / 2
    node.on_motor_command(cmd)
    node._tick_control()

    frame = _last_frame(node)
    pid, _ = sp.parse_status_id(frame.arbitration_id)
    assert pid == sp.CAN_PACKET_SET_POS_SPD
    assert len(frame.data) == 8
    deg = struct.unpack(">i", frame.data[0:4])[0] / sp.POS_SCALE
    assert deg == pytest.approx(90.0, abs=1e-3)
    speed_erpm = struct.unpack(">h", frame.data[4:6])[0] * sp.POS_SPD_SPEED_SCALE
    assert speed_erpm == pytest.approx(node._min_speed_erpm, abs=sp.POS_SPD_SPEED_SCALE)


def test_pos_spd_static_hold_uses_min_speed(make_node):
    """A held (unchanging) position setpoint -- e.g. the 'stand' pose -- must
    still command a non-zero speed so the motor moves to and holds the target.
    """
    node, mn = make_node(motor_type="AK40-10", control_hz=100.0, position_mode="pos_spd")
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_POSITION
    cmd.pos_setpoint = 0.30  # constant stand-like target

    node.on_motor_command(cmd)
    node._tick_control()          # first frame (seeds history)
    node.on_motor_command(cmd)    # identical setpoint again
    node._tick_control()          # second frame: feed-forward would be 0

    frame = _last_frame(node)
    pid, _ = sp.parse_status_id(frame.arbitration_id)
    assert pid == sp.CAN_PACKET_SET_POS_SPD
    speed_erpm = struct.unpack(">h", frame.data[4:6])[0] * sp.POS_SPD_SPEED_SCALE
    assert speed_erpm == pytest.approx(node._min_speed_erpm, abs=sp.POS_SPD_SPEED_SCALE)
    assert speed_erpm > 0


def test_pos_spd_velocity_feedforward(make_node):
    # gear=10, pole_pairs=14, control_hz=100
    node, mn = make_node(motor_type="AK40-10", control_hz=100.0, position_mode="pos_spd")
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_POSITION

    cmd.pos_setpoint = 0.10
    node.on_motor_command(cmd)
    node._tick_control()  # first frame: speed 0, seeds feed-forward

    cmd.pos_setpoint = 0.15
    node.on_motor_command(cmd)
    node._tick_control()  # second frame: speed from (0.15-0.10)*100 = 5 rad/s

    frame = _last_frame(node)
    pid, _ = sp.parse_status_id(frame.arbitration_id)
    assert pid == sp.CAN_PACKET_SET_POS_SPD
    deg = struct.unpack(">i", frame.data[0:4])[0] / sp.POS_SCALE
    assert deg == pytest.approx(math.degrees(0.15), abs=1e-3)
    speed_on_wire = struct.unpack(">h", frame.data[4:6])[0]
    expected_erpm = sp.rads_to_erpm(5.0, 10, 14)  # 6683.6 ERPM
    assert speed_on_wire == pytest.approx(round(expected_erpm / 10.0), abs=1)
    accel_on_wire = struct.unpack(">h", frame.data[6:8])[0]
    assert accel_on_wire == pytest.approx(round(node.pos_spd_accel / 10.0), abs=1)


def test_pos_spd_speed_never_exceeds_cap(make_node):
    node, mn = make_node(motor_type="AK40-10", control_hz=100.0, position_mode="pos_spd")
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_POSITION
    # Huge jump between ticks -> feed-forward would exceed V_MAX; must be capped.
    cmd.pos_setpoint = 0.0
    node.on_motor_command(cmd)
    node._tick_control()
    cmd.pos_setpoint = 50.0  # absurd single-step
    node.on_motor_command(cmd)
    node._tick_control()

    frame = _last_frame(node)
    speed_erpm = struct.unpack(">h", frame.data[4:6])[0] * sp.POS_SPD_SPEED_SCALE
    # Capped at V_MAX (allow one int16 quantization step of 10 ERPM), not the
    # ~6.7M ERPM the raw 50 rad/tick feed-forward would demand.
    assert speed_erpm <= node._speed_cap_erpm + sp.POS_SPD_SPEED_SCALE


def test_velocity_command_emits_erpm(make_node):
    node, mn = make_node(motor_type="AK40-10")  # gear=10, pole_pairs=14
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_VELOCITY
    cmd.vel_setpoint = 6.283185307179586  # 1 rev/s of output shaft = 2pi rad/s
    node.on_motor_command(cmd)
    node._tick_control()

    frame = _last_frame(node)
    pid, _ = sp.parse_status_id(frame.arbitration_id)
    assert pid == sp.CAN_PACKET_SET_RPM
    erpm = struct.unpack(">i", frame.data)[0]
    # 60 output RPM * gear(10) * pole_pairs(14) = 8400 ERPM
    assert erpm == pytest.approx(8400, abs=1)


def test_torque_command_emits_current(make_node):
    node, mn = make_node(motor_type="AK40-10")  # Kt=0.056, gear=10
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_TORQUE
    cmd.torq_setpoint = 2.8  # Nm at output shaft
    node.on_motor_command(cmd)
    node._tick_control()

    frame = _last_frame(node)
    pid, _ = sp.parse_status_id(frame.arbitration_id)
    assert pid == sp.CAN_PACKET_SET_CURRENT
    milli_amps = struct.unpack(">i", frame.data)[0]
    # I = tau / (Kt*gear) = 2.8 / 0.56 = 5.0 A -> 5000 mA
    assert milli_amps == pytest.approx(5000, abs=1)


def test_torque_command_clamped_to_motor_limit(make_node):
    node, mn = make_node(motor_type="AK40-10")  # T_MAX = 4.1 Nm (peak torque)
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_TORQUE
    cmd.torq_setpoint = 100.0  # way over the 4.1 Nm limit
    node.on_motor_command(cmd)
    node._tick_control()

    frame = _last_frame(node)
    milli_amps = struct.unpack(">i", frame.data)[0]
    # 4.1 Nm / (0.056*10) = 7.321 A -> ~7321 mA (matches datasheet 7.3 A peak)
    assert milli_amps == pytest.approx(int(4.1 / (0.056 * 10) * 1000), abs=2)


def test_reverse_polarity_negates_position(make_node):
    node, mn = make_node(motor_type="AK40-10", reverse_polarity=True, position_mode="pos")
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_POSITION
    cmd.pos_setpoint = 3.141592653589793 / 2
    node.on_motor_command(cmd)
    node._tick_control()

    deg = struct.unpack(">i", _last_frame(node).data)[0] / sp.POS_SCALE
    assert deg == pytest.approx(-90.0, abs=1e-3)


def test_not_started_releases_current(make_node):
    node, mn = make_node(motor_type="AK40-10", auto_start=False)
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_POSITION
    cmd.pos_setpoint = 1.0
    node.on_motor_command(cmd)
    node._tick_control()

    frame = _last_frame(node)
    pid, _ = sp.parse_status_id(frame.arbitration_id)
    assert pid == sp.CAN_PACKET_SET_CURRENT  # released, not moved
    assert struct.unpack(">i", frame.data)[0] == 0


def test_start_then_move(make_node):
    node, mn = make_node(motor_type="AK40-10", auto_start=False, position_mode="pos")
    special = mn.String()
    special.data = "start"
    node.on_special(special)
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_POSITION
    cmd.pos_setpoint = 1.0
    node.on_motor_command(cmd)
    node._tick_control()

    pid, _ = sp.parse_status_id(_last_frame(node).arbitration_id)
    assert pid == sp.CAN_PACKET_SET_POS


def test_zero_sends_origin(make_node):
    node, mn = make_node(motor_type="AK40-10")
    special = mn.String()
    special.data = "zero"
    node.on_special(special)
    frame = _last_frame(node)
    pid, _ = sp.parse_status_id(frame.arbitration_id)
    assert pid == sp.CAN_PACKET_SET_ORIGIN_HERE
    assert frame.data == bytes([0])


def test_cmd_timeout_releases(make_node):
    node, mn = make_node(motor_type="AK40-10", cmd_timeout=0.001)
    cmd = mn.MotorCommand()
    cmd.control_mode = mn.MotorCommand.CONTROL_MODE_POSITION
    cmd.pos_setpoint = 1.0
    node.on_motor_command(cmd)
    import time
    time.sleep(0.01)
    node._tick_control()
    frame = _last_frame(node)
    pid, _ = sp.parse_status_id(frame.arbitration_id)
    assert pid == sp.CAN_PACKET_SET_CURRENT
    assert struct.unpack(">i", frame.data)[0] == 0


def test_gear_and_pole_override(make_node):
    node, mn = make_node(motor_type="AK40-10", pole_pairs=10, gear_ratio=5.0)
    assert node.pole_pairs == 10
    assert node.gear_ratio == 5.0
