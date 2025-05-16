# esc_input.msg

# ESC input message definition for PX4 Autopilot

# Timestamp of the message
uint64 timestamp

# ESC input values (e.g., PWM signals)
float32 manual_control_1
float32 manual_control_2
float32 manual_control_3
float32 manual_control_4

float32 thrust_control_1
float32 thrust_control_2
float32 thrust_control_3
float32 thrust_control_4