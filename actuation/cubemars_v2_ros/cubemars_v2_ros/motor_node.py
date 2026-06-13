#!/usr/bin/env python3
"""
ROS2 Node for Cubemars Actuator Control

This module provides a ROS2 interface for controlling Cubemars (AK series)
motor actuators via CAN bus. It supports multiple motor types and implements
the MIT control protocol for commanding position, velocity, and torque.

The node publishes motor state data including position, velocity, torque,
temperature, and error codes, and subscribes to command topics for motor control.
"""

import math
import threading

import can
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from actuation_msgs.msg import MotorCommand, MotorEstimate, MotorInfo
from std_msgs.msg import Float64MultiArray, String
from motor_interfaces.msg import MotorState

# TODO: Add additional motors and their specifications

# ===================== MOTOR SPECIFICATIONS AND CONSTANTS =====================

# Motor limits for different actuator models (from CubeMars documentation)
# P: position limits (rad), V: velocity limits (rad/s), T: torque limits (Nm)
# KP: position gain limits, KD: velocity gain limits
LIMITS = {
    # Position is ±12.5 rad for all models; Kp=[0,500], Kd=[0,5]
    "AK10-9": dict(
        P_MIN=-12.5,
        P_MAX=12.5,
        V_MIN=-50.0,
        V_MAX=50.0,
        T_MIN=-65.0,
        T_MAX=65.0,
        KP_MIN=0.0,
        KP_MAX=500.0,
        KD_MIN=0.0,
        KD_MAX=5.0,
    ),
    "AK60-6": dict(
        P_MIN=-12.5,
        P_MAX=12.5,
        V_MIN=-45.0,
        V_MAX=45.0,
        T_MIN=-15.0,
        T_MAX=15.0,
        KP_MIN=0.0,
        KP_MAX=500.0,
        KD_MIN=0.0,
        KD_MAX=5.0,
    ),
    "AK70-10": dict(
        P_MIN=-12.5,
        P_MAX=12.5,
        V_MIN=-50.0,
        V_MAX=50.0,
        T_MIN=-25.0,
        T_MAX=25.0,
        KP_MIN=0.0,
        KP_MAX=500.0,
        KD_MIN=0.0,
        KD_MAX=5.0,
    ),
    "AK80-6": dict(
        P_MIN=-12.5,
        P_MAX=12.5,
        V_MIN=-76.0,
        V_MAX=76.0,
        T_MIN=-12.0,
        T_MAX=12.0,
        KP_MIN=0.0,
        KP_MAX=500.0,
        KD_MIN=0.0,
        KD_MAX=5.0,
    ),
    "AK80-9": dict(
        P_MIN=-12.5,
        P_MAX=12.5,
        V_MIN=-50.0,
        V_MAX=50.0,
        T_MIN=-18.0,
        T_MAX=18.0,
        KP_MIN=0.0,
        KP_MAX=500.0,
        KD_MIN=0.0,
        KD_MAX=5.0,
    ),
    "AK80-64": dict(
        P_MIN=-12.5,
        P_MAX=12.5,
        V_MIN=-8.0,
        V_MAX=8.0,
        T_MIN=-144.0,
        T_MAX=144.0,
        KP_MIN=0.0,
        KP_MAX=500.0,
        KD_MIN=0.0,
        KD_MAX=5.0,
    ),
    "AK80-8": dict(
        P_MIN=-12.5,
        P_MAX=12.5,
        V_MIN=-37.5,
        V_MAX=37.5,
        T_MIN=-32.0,
        T_MAX=32.0,
        KP_MIN=0.0,
        KP_MAX=500.0,
        KD_MIN=0.0,
        KD_MAX=5.0,
    ),
    "AK40-10": dict(
        P_MIN=-12.5,
        P_MAX=12.5,
        V_MIN=-45.5,
        V_MAX=45.5,
        T_MIN=-5.0,
        T_MAX=5.0,
        KP_MIN=0.0,
        KP_MAX=500.0,
        KD_MIN=0.0,
        KD_MAX=5.0,
    ),
}

# Torque constants (Nm/A) for converting between current and torque
TORQUE_CONSTANTS = {
    "AK10-9": 0.198,  # Nm/A
    "AK70-10": 0.123,  # Nm/A
    "AK80-64": 0.136,  # Nm/A
    "AK40-10": 0.056,
}

# Motor error code mapping for human-readable messages
MOTOR_ERROR_CODES = {
    0: "No fault - Motor operating normally",
    1: "Motor over-temperature fault - Motor temperature exceeds safe operating limits",
    2: "Over-current fault - Current draw exceeds maximum allowable threshold",
    3: "Over-voltage fault - Input voltage exceeds maximum operating voltage",
    4: "Under-voltage fault - Input voltage below minimum operating voltage",
    5: "Encoder fault - Position feedback sensor malfunction or disconnection",
    6: "MOSFET over-temperature fault - Power transistor temperature exceeds safe limits",
    7: "Motor lock-up - Motor shaft mechanically blocked or seized",
}

# ===================== UTILITY FUNCTIONS =====================


def clamp(x, lo, hi):
    """Clamp a value between lower and upper bounds"""
    return lo if x < lo else hi if x > hi else x


def f2u(x, lo, hi, bits):
    """
    Convert float value to unsigned integer representation (used in CAN protocol).

    Args:
        x: Float value to convert
        lo: Lower bound of float range
        hi: Upper bound of float range
        bits: Number of bits for integer representation

    Returns:
        Integer representation of the float value
    """
    x = clamp(x, lo, hi)
    return int((x - lo) * (((1 << bits) - 1) / (hi - lo)))


def u2f(u, lo, hi, bits):
    """
    Convert unsigned integer back to float value (used in CAN protocol).

    Args:
        u: Integer value to convert
        lo: Lower bound of target float range
        hi: Upper bound of target float range
        bits: Number of bits in the integer representation

    Returns:
        Float value corresponding to the integer representation
    """
    u = max(0, min(u, (1 << bits) - 1))
    return lo + (u / ((1 << bits) - 1)) * (hi - lo)


def pack_mit(p, v, kp, kd, t, R):
    """
    Pack MIT control message parameters into CAN frame data bytes.

    Args:
        p: Position setpoint (rad)
        v: Velocity setpoint (rad/s)
        kp: Position gain
        kd: Velocity gain
        t: Torque feedforward (Nm)
        R: Motor limits dictionary

    Returns:
        Byte array for CAN message
    """
    # Convert float values to integers with appropriate bit width
    p_i = f2u(p, R["P_MIN"], R["P_MAX"], 16)  # 16 bits for position
    v_i = f2u(v, R["V_MIN"], R["V_MAX"], 12)  # 12 bits for velocity
    kp_i = f2u(kp, R["KP_MIN"], R["KP_MAX"], 12)  # 12 bits for position gain
    kd_i = f2u(kd, R["KD_MIN"], R["KD_MAX"], 12)  # 12 bits for velocity gain
    t_i = f2u(t, R["T_MIN"], R["T_MAX"], 12)  # 12 bits for torque

    # Pack values into 8 bytes following MIT protocol format
    return bytes(
        [
            (p_i >> 8) & 0xFF,  # Position (high byte)
            p_i & 0xFF,  # Position (low byte)
            (v_i >> 4) & 0xFF,  # Velocity (high byte)
            ((v_i & 0x0F) << 4)
            | ((kp_i >> 8) & 0x0F),  # Velocity (low nibble) + Kp (high nibble)
            kp_i & 0xFF,  # Kp (low byte)
            (kd_i >> 4) & 0xFF,  # Kd (high byte)
            ((kd_i & 0x0F) << 4)
            | ((t_i >> 8) & 0x0F),  # Kd (low nibble) + Torque (high nibble)
            t_i & 0xFF,  # Torque (low byte)
        ]
    )


def pack_mit_cubemars(p, v, kp, kd, t, R):
    """Pack the official CubeMars AK-series extended-frame MIT command."""
    p_i = f2u(p, R["P_MIN"], R["P_MAX"], 16)
    v_i = f2u(v, R["V_MIN"], R["V_MAX"], 12)
    kp_i = f2u(kp, R["KP_MIN"], R["KP_MAX"], 12)
    kd_i = f2u(kd, R["KD_MIN"], R["KD_MAX"], 12)
    t_i = f2u(t, R["T_MIN"], R["T_MAX"], 12)

    return bytes([
        (kp_i >> 4) & 0xFF,
        ((kp_i & 0x0F) << 4) | ((kd_i >> 8) & 0x0F),
        kd_i & 0xFF,
        (p_i >> 8) & 0xFF,
        p_i & 0xFF,
        (v_i >> 4) & 0xFF,
        ((v_i & 0x0F) << 4) | ((t_i >> 8) & 0x0F),
        t_i & 0xFF,
    ])

def parse_reply(b, R):
    """
    Parse the CAN reply from the motor controller.

    Args:
        b: Byte array from CAN message
        R: Motor limits dictionary

    Returns:
        Tuple of (driver_id, position, velocity, torque, temperature, error_code)
        or None if invalid data
    """
    if len(b) != 8:
        return None

    drv = b[0]  # Driver ID (motor controller ID)
    p_int = (b[1] << 8) | b[2]  # Position (16 bits)
    v_int = (b[3] << 4) | (b[4] >> 4)  # Velocity (12 bits)
    i_int = ((b[4] & 0x0F) << 8) | b[5]  # Current/Torque (12 bits)
    temp = b[6] - 40  # Temperature in Celsius for AK 2.0 MIT feedback
    err = b[7]  # Error code (8 bits)

    # Convert integer values back to physical units
    p = u2f(p_int, R["P_MIN"], R["P_MAX"], 16)  # rad
    v = u2f(v_int, R["V_MIN"], R["V_MAX"], 12)  # rad/s
    tau = u2f(i_int, R["T_MIN"], R["T_MAX"], 12)  # Nm

    return drv, p, v, tau, temp, err


def parse_reply_cubemars(b, motor_type):
    """Parse the official CubeMars periodic CAN upload message."""
    if len(b) != 8:
        return None

    p_raw = int.from_bytes(bytes(b[0:2]), byteorder="big", signed=True)
    v_raw = int.from_bytes(bytes(b[2:4]), byteorder="big", signed=True)
    current_raw = int.from_bytes(bytes(b[4:6]), byteorder="big", signed=True)
    temp = b[6]
    err = b[7]

    position_rad = math.radians(p_raw * 0.1)
    velocity_erpm = v_raw * 10.0
    current_a = current_raw * 0.01
    torque_nm = current_a * TORQUE_CONSTANTS[motor_type]
    return 0, position_rad, velocity_erpm, torque_nm, current_a, temp, err

def get_error_message(error_code):
    """
    Get human-readable error message for motor error code.

    Args:
        error_code (int): Error code (0-7)

    Returns:
        str: Error message or unknown error if code is invalid
    """
    return MOTOR_ERROR_CODES.get(error_code, f"Unknown error code: {error_code}")


def wrap_to_range(x, lo, hi):
    """
    Wrap a circular position command into the motor protocol's valid range.

    CubeMars MIT position commands are encoded over a fixed finite range
    (normally [-12.5, 12.5] rad).  Sending an absolute, unwrapped setpoint
    directly to the driver can clamp the command to the range endpoint and make
    the motor continue rotating instead of converging.  Wrapping preserves the
    requested joint angle modulo one encoder span while keeping the CAN command
    encodable.
    """
    span = hi - lo
    return ((x - lo) % span) + lo


# ===================== MOTOR NODE CLASS =====================


class MotorNode(Node):
    """
    ROS2 Node for controlling a Cubemars actuator via CAN bus.

    This node handles:
    - Communication with the motor via CAN
    - Publishing motor state (position, velocity, torque, temperature)
    - Receiving commands (MIT protocol and special commands)
    - Managing motor startup/shutdown sequences
    """

    def __init__(self):
        """Initialize the motor control node and its parameters"""
        super().__init__("motor_node")

        # ---- ROS Parameters ----
        self.declare_parameter("can_interface", "can0")  # CAN interface name
        self.declare_parameter("can_id", 1)  # CAN ID for this motor
        self.declare_parameter("motor_type", "AK70-10")  # Motor model
        self.declare_parameter("control_hz", 20.0)  # Control loop frequency
        self.declare_parameter("joint_name", "joint")  # Name for this joint/motor
        self.declare_parameter("auto_start", False)  # Whether to auto-start the motor
        self.declare_parameter("reverse_polarity", False)  # Reverse motor direction
        self.declare_parameter("protocol", "ak2")  # ak2/tmotor standard MIT or ak3/cubemars extended MIT
        self.declare_parameter("position_kp", 20.0)  # Kp for MotorCommand position mode
        self.declare_parameter("position_kd", 1.0)  # Kd for MotorCommand position mode
        self.declare_parameter("velocity_kd", 1.0)  # Kd for MotorCommand velocity mode

        # Get parameters
        self.iface = self.get_parameter("can_interface").value
        self.can_id = int(self.get_parameter("can_id").value)
        self.motor_type = self.get_parameter("motor_type").value
        if self.motor_type not in LIMITS:
            self.get_logger().error(
                f"Unsupported motor type: {self.motor_type}. Supported types: {list(LIMITS.keys())}"
            )
            raise ValueError(f"Unsupported motor type: {self.motor_type}")
        self.R = LIMITS[self.motor_type]  # Get motor limits for this type
        self.joint_name = self.get_parameter("joint_name").value
        self.control_dt = 1.0 / float(
            self.get_parameter("control_hz").value
        )  # Control period
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.control_hz = self.get_parameter("control_hz").value
        self.reverse_polarity = bool(self.get_parameter("reverse_polarity").value)
        self.protocol = str(self.get_parameter("protocol").value).lower()
        if self.protocol not in ("ak2", "tmotor", "ak3", "cubemars"):
            raise ValueError("protocol must be ak2/tmotor or ak3/cubemars")
        self.position_kp = float(self.get_parameter("position_kp").value)
        self.position_kd = float(self.get_parameter("position_kd").value)
        self.velocity_kd = float(self.get_parameter("velocity_kd").value)
        self.add_on_set_parameters_callback(self._on_parameter_update)

        # Log parameters for debugging
        self.get_logger().info(
            f"""
            Motor: {self.joint_name}
            Can_id: {self.can_id}
            Motor type: {self.motor_type}
            Control Hz: {self.control_hz}
            Auto Start: {self.auto_start}
            Reverse Polarity: {self.reverse_polarity}
            Protocol: {self.protocol}
            Position Kp: {self.position_kp}
            Position Kd: {self.position_kd}
            Velocity Kd: {self.velocity_kd}
            """
        )

        if self.protocol in ("ak3", "cubemars"):
            self.arb = (8 << 8) | (self.can_id & 0xFF)  # Official MIT control extended ID
            self.rx_arb = self.can_id & 0xFF
            self.is_extended_id = True
        else:
            self.arb = self.can_id & 0x7FF  # Legacy/TMotor MIT standard ID
            self.rx_arb = self.arb
            self.is_extended_id = False
        try:
            self.bus = can.interface.Bus(bustype="socketcan", channel=self.iface)
            try:
                # Filter to only receive messages with our CAN ID
                self.bus.set_filters([{"can_id": self.rx_arb, "can_mask": 0x1FFFFFFF if self.is_extended_id else 0x7FF}])
            except Exception:
                pass  # Some interfaces don't support filtering
        except Exception as e:
            self.get_logger().error(
                f"Failed to initialize CAN bus on interface '{self.iface}': {e}"
            )
            raise

        # ---- ROS Publishers and Subscribers ----
        # Publishers
        self.pub_err = self.create_publisher(
            String, f"/{self.joint_name}/error_code", 10
        )
        self.pub_state = self.create_publisher(
            MotorState, f"/{self.joint_name}/motor_state", 10
        )
        self.pub_estimate = self.create_publisher(
            MotorEstimate, f"/{self.joint_name}/estimate", 10
        )
        self.pub_info = self.create_publisher(
            MotorInfo, f"/{self.joint_name}/info", 10
        )

        # Subscribers
        # MIT command format: [position, velocity, Kp, Kd, torque]
        self.sub_mit_cmd = self.create_subscription(
            Float64MultiArray, f"/{self.joint_name}/mit_cmd", self.on_cmd, 10
        )
        # Generic actuation command used by the leg kinematics stack.
        self.sub_motor_command = self.create_subscription(
            MotorCommand, f"/{self.joint_name}/command", self.on_motor_command, 10
        )
        # Special commands: start, exit, zero, clear
        self.sub_special = self.create_subscription(
            String, f"/{self.joint_name}/special_cmd", self.on_special, 10
        )

        # ---- Internal State ----
        self._lock = threading.Lock()  # Thread safety for command access
        self.cmd = [0.0, 0.0, 0.0, 0.0, 0.0]  # Current command [p, v, kp, kd, t]
        self._started = False  # Whether the motor is started
        self._neutral_hold = (
            False  # When True, sends zero commands regardless of cmd cache
        )
        self._position_control = False  # Whether cached command uses position mode
        self._warned_no_torque_constant = False

        # Absolute position tracking (unwrapping)
        self._last_p = None  # Last raw position reading
        self._p_abs = 0.0  # Unwrapped absolute position
        self._span = self.R["P_MAX"] - self.R["P_MIN"]  # Position range

        # ---- Timers ----
        # Control timer sends commands at the specified frequency
        self.create_timer(self.control_dt, self._tick_control)

        # ---- CAN Receiver Thread ----
        self._stop = False  # Flag to stop the RX thread
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx.start()

        # Optional auto-start on initialization
        if self.auto_start and not self._started:
            self._send_special(0xFC)  # Send START command (0xFC)
            self._started = True
            self.get_logger().info(f"Auto-starting motor {self.joint_name}")

    # ---- Command Callbacks ----
    def on_cmd(self, msg):
        """
        Handle MIT command message: [position, velocity, Kp, Kd, torque]

        Args:
            msg: Float64MultiArray containing the command values
        """
        if len(msg.data) != 5:
            self.get_logger().warn(
                "mit_cmd expects [position, velocity, Kp, Kd, torque]"
            )
            return

        with self._lock:
            self.cmd = list(map(float, msg.data))
            self._neutral_hold = False  # New command cancels any previous "clear" hold
            self._position_control = self.cmd[2] > 0.0

        # Auto-start the motor on first command if not already started
        # if not self._started:
        #     self._send_special(0xFC)  # START command
        #     self._started = True
        #     self.get_logger().info(f"Auto-starting motor {self.joint_name} on first command")


    def _on_parameter_update(self, params):
        """Apply runtime gain updates from ros2 param set /motorN_node ..."""
        next_position_kp = self.position_kp
        next_position_kd = self.position_kd
        next_velocity_kd = self.velocity_kd

        for param in params:
            if param.name == "position_kp":
                next_position_kp = float(param.value)
            elif param.name == "position_kd":
                next_position_kd = float(param.value)
            elif param.name == "velocity_kd":
                next_velocity_kd = float(param.value)

        if not (self.R["KP_MIN"] <= next_position_kp <= self.R["KP_MAX"]):
            return SetParametersResult(
                successful=False,
                reason=f"position_kp must be in [{self.R['KP_MIN']}, {self.R['KP_MAX']}]",
            )
        if not (self.R["KD_MIN"] <= next_position_kd <= self.R["KD_MAX"]):
            return SetParametersResult(
                successful=False,
                reason=f"position_kd must be in [{self.R['KD_MIN']}, {self.R['KD_MAX']}]",
            )
        if not (self.R["KD_MIN"] <= next_velocity_kd <= self.R["KD_MAX"]):
            return SetParametersResult(
                successful=False,
                reason=f"velocity_kd must be in [{self.R['KD_MIN']}, {self.R['KD_MAX']}]",
            )

        self.position_kp = next_position_kp
        self.position_kd = next_position_kd
        self.velocity_kd = next_velocity_kd
        self.get_logger().info(
            f"Updated gains: position_kp={self.position_kp}, "
            f"position_kd={self.position_kd}, velocity_kd={self.velocity_kd}"
        )
        return SetParametersResult(successful=True)

    def on_motor_command(self, msg):
        """
        Bridge the generic MotorCommand topic into the cached MIT command.

        The rest of the SELQIE leg-control stack publishes MotorCommand messages
        on /motorN/command. Cubemars motors are driven with MIT commands in the
        form [position, velocity, Kp, Kd, torque], so this callback supplies the
        gains that are not present in MotorCommand and lets _tick_control send
        the packed CAN frame at the configured control rate.

        Args:
            msg: actuation_msgs/MotorCommand message
        """
        p = float(msg.pos_setpoint)
        v = float(msg.vel_setpoint)
        t = float(msg.torq_setpoint)

        if msg.control_mode == MotorCommand.CONTROL_MODE_POSITION:
            kp = self.position_kp
            kd = self.position_kd
            position_control = True
        elif msg.control_mode == MotorCommand.CONTROL_MODE_VELOCITY:
            p = 0.0
            kp = 0.0
            kd = self.velocity_kd
            position_control = False
        elif msg.control_mode == MotorCommand.CONTROL_MODE_TORQUE:
            p = 0.0
            v = 0.0
            kp = 0.0
            kd = 0.0
            position_control = False
        else:
            self.get_logger().warn(
                f"Unsupported MotorCommand control_mode {msg.control_mode}; command ignored"
            )
            return

        with self._lock:
            self.cmd = [p, v, kp, kd, t]
            self._neutral_hold = False  # New command cancels any previous "clear" hold
            self._position_control = position_control

    def on_special(self, msg):
        """
        Handle special command message for motor control

        Args:
            msg: String message with the command name
        """
        m = msg.data.strip().lower()

        if m == "start":
            # Start the motor (MIT control mode)
            if not self._started:
                if self.protocol in ("ak2", "tmotor"):
                    self._send_special(0xFC)  # START command (0xFC)
                self._started = True
                self.get_logger().info(f"Starting motor {self.joint_name}")

        elif m == "exit":
            # Exit MIT control mode
            if self._started:
                if self.protocol in ("ak2", "tmotor"):
                    self._send_special(0xFD)  # EXIT command (0xFD)
                self._started = False
                self.get_logger().info(f"Exiting MIT mode for motor {self.joint_name}")

        elif m == "zero":
            # Zero/home the encoder
            if self.protocol in ("ak2", "tmotor"):
                self._send_special(0xFE)  # ZERO command (0xFE)
                self.get_logger().info(f"Zeroing encoder for motor {self.joint_name}")
            else:
                self.get_logger().warn("Zero over CAN is not implemented for the AK 3.0 extended protocol")

        elif m == "clear":
            # Clear all commands (send zeros and hold)
            with self._lock:
                self.cmd = [0.0, 0.0, 0.0, 0.0, 0.0]  # Clear command cache
                self._neutral_hold = True  # Hold zeros until new command
                self._position_control = False

            # Send zero command immediately
            self._send_mit_once(0, 0, 0, 0, 0)
            self.get_logger().info(f"Clearing commands for motor {self.joint_name}")

        else:
            self.get_logger().warn(
                f"Unknown special command: '{m}'. Valid options: start|exit|zero|clear"
            )

    # ---- Control Loop ----
    def _tick_control(self):
        """Periodic control loop that sends commands to the motor"""
        with self._lock:
            # If in neutral hold mode, send zeros; otherwise send cached command
            p, v, kp, kd, t = ([0.0] * 5) if self._neutral_hold else self.cmd
            position_control = self._position_control

        if position_control:
            p = wrap_to_range(p, self.R["P_MIN"], self.R["P_MAX"])

        # Apply reverse polarity if configured
        if self.reverse_polarity:
            p = -p  # Invert position
            v = -v  # Invert velocity
            t = -t  # Invert torque

        # Pack the command into CAN message format
        data = (pack_mit_cubemars(p, v, kp, kd, t, self.R) if self.protocol in ("ak3", "cubemars") else pack_mit(p, v, kp, kd, t, self.R))

        try:
            # Send the command over CAN
            self.bus.send(
                can.Message(arbitration_id=self.arb, data=data, is_extended_id=self.is_extended_id)
            )
        except can.CanError:
            self.get_logger().error(
                f"Failed to send CAN message to motor {self.joint_name}"
            )

    # ---- Helper Methods ----
    def _send_special(self, code):
        """
        Send special command code to the motor

        Args:
            code: Command code byte (e.g. 0xFC for START)
        """
        # Special commands use all 0xFF bytes except the last one
        d = b"\xff" * 7 + bytes([code & 0xFF])

        try:
            self.bus.send(
                can.Message(arbitration_id=self.arb, data=d, is_extended_id=self.is_extended_id)
            )
        except can.CanError:
            self.get_logger().error(
                f"Failed to send special command {hex(code)} to motor {self.joint_name}"
            )

    def _send_mit_once(self, p, v, kp, kd, t):
        """
        Send one MIT protocol command to the motor

        Args:
            p: Position setpoint (rad)
            v: Velocity setpoint (rad/s)
            kp: Position gain
            kd: Velocity gain
            t: Torque feedforward (Nm)
        """
        # Apply reverse polarity if configured
        if self.reverse_polarity:
            p = -p  # Invert position
            v = -v  # Invert velocity
            t = -t  # Invert torque

        d = (pack_mit_cubemars(p, v, kp, kd, t, self.R) if self.protocol in ("ak3", "cubemars") else pack_mit(p, v, kp, kd, t, self.R))

        try:
            self.bus.send(
                can.Message(arbitration_id=self.arb, data=d, is_extended_id=self.is_extended_id)
            )
        except can.CanError:
            self.get_logger().error(
                f"Failed to send MIT command to motor {self.joint_name}"
            )

    def _rx_loop(self):
        """
        Background thread that receives and processes CAN messages from the motor
        """
        while not self._stop:
            # Wait for a CAN message (with timeout)
            rx = self.bus.recv(timeout=0.1)

            # Skip if no message or wrong ID/length
            if not rx or rx.arbitration_id != self.rx_arb or len(rx.data) != 8:
                continue

            # Parse the motor reply
            if self.protocol in ("ak3", "cubemars"):
                s = parse_reply_cubemars(rx.data, self.motor_type)
                if not s:
                    continue
                drv, p, v, tau, current, temp, err = s
            else:
                s = parse_reply(rx.data, self.R)
                if not s:
                    continue
                drv, p, v, tau, temp, err = s
                torque_constant = TORQUE_CONSTANTS.get(self.motor_type)
                current = tau / torque_constant if torque_constant else 0.0
                if torque_constant is None and not self._warned_no_torque_constant:
                    self.get_logger().warn(
                        f"No torque constant configured for {self.motor_type}; publishing current as 0.0 A"
                    )
                    self._warned_no_torque_constant = True

                # Verify the driver ID matches our expected ID (low byte of arbitration ID)
                if drv != (self.arb & 0xFF):
                    continue

            # Apply reverse polarity to received values if configured
            if self.reverse_polarity:
                p = -p  # Invert position
                v = -v  # Invert velocity
                tau = -tau  # Invert torque

            # ---- Process position data for unwrapping ----
            # Handle position unwrapping to track continuous rotation beyond ±12.5 rad
            if self._last_p is None:
                # First reading - initialize absolute position
                self._p_abs = p
            else:
                # Calculate position change, handling wraparound
                dp = p - self._last_p

                # Detect and correct for wraparound (e.g. going from +12.4 to -12.4 rad)
                if dp > 0.5 * self._span:  # Wraparound in negative direction
                    dp -= self._span
                if dp < -0.5 * self._span:  # Wraparound in positive direction
                    dp += self._span

                # Update absolute position
                self._p_abs += dp

            # Store current position for next iteration
            self._last_p = p

            # ---- Publish motor data ----
            # Publish temperature separately

            # Publish error code with human-readable message
            error_code = String()
            error_code.data = f"Error Code {err}: {get_error_message(err)}"
            self.pub_err.publish(error_code)

            # Publish complete motor state
            ms = MotorState()
            ms.name = self.joint_name  # Motor/joint name
            ms.position = p  # Position in rad (raw)
            ms.abs_position = self._p_abs  # Absolute position in rad (unwrapped)
            ms.velocity = v  # Velocity in rad/s for tmotor, ERPM for official CubeMars feedback
            ms.torque = tau  #  Torque in Nms
            ms.current = current  # current in A
            ms.temperature = temp  # Temperature in °C
            self.pub_state.publish(ms)

            estimate = MotorEstimate()
            estimate.pos_estimate = float(self._p_abs)
            estimate.vel_estimate = float(v)
            estimate.torq_estimate = float(tau)
            self.pub_estimate.publish(estimate)

            info = MotorInfo()
            info.axis_state = 0
            info.error = int(err)
            info.error_name = get_error_message(err)
            self.pub_info.publish(info)

    def destroy_node(self):
        """Clean up resources when the node is shutting down"""
        if self.protocol in ("ak2", "tmotor"):
            self._send_special(0xFD)  # EXIT command (0xFD)
        self._stop = True  # Signal RX thread to stop

        try:
            self._rx.join(timeout=0.3)  # Wait for RX thread to terminate
        except:
            pass

        try:
            if hasattr(self.bus, "shutdown"):
                self.bus.shutdown()  # Close CAN bus connection
        except:
            pass

        super().destroy_node()  # Call parent class cleanup


def main(args=None):
    """Main entry point for the motor node"""
    rclpy.init(args=args)
    node = MotorNode()

    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        # Handle graceful shutdown on Ctrl+C
        pass

    # Clean up
    node.destroy_node()
    rclpy.shutdown()
