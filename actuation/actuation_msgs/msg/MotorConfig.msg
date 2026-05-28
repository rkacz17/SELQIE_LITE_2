# Motor configuration values. Zero-valued fields are treated as "leave unchanged" by existing nodes.

uint32 AXIS_STATE_IDLE=1
uint32 AXIS_STATE_STARTUP_SEQUENCE=2
uint32 AXIS_STATE_FULL_CALIBRATION_SEQUENCE=3
uint32 AXIS_STATE_MOTOR_CALIBRATION=4
uint32 AXIS_STATE_ENCODER_INDEX_SEARCH=6
uint32 AXIS_STATE_ENCODER_OFFSET_CALIBRATION=7
uint32 AXIS_STATE_CLOSED_LOOP_CONTROL=8

uint32 axis_state
float32 gear_ratio
float32 pos_gain
float32 vel_gain
float32 vel_int_gain
