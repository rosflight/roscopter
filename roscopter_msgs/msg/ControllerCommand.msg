# roscopter commands
# can be used for any level of command:
# high level: n_pos, e_pos, d_pos, yaw
# mid level: n_pos, e_pos, d_vel, yaw
#        or: n_vel, e_vel, d_pos, yaw_rate
#        or: n_vel, e_vel, d_vel, yaw_rate
# low level: n_acc, e_acc, d_acc, yaw_acc

# Input control types - Uses ROSflight controller
uint8 MODE_NPOS_EPOS_DPOS_YAW = 0
uint8 MODE_NVEL_EVEL_DPOS_YAWRATE = 1
uint8 MODE_NACC_EACC_DACC_YAWRATE = 2
uint8 MODE_NVEL_EVEL_DVEL_YAWRATE = 3
uint8 MODE_NPOS_EPOS_DVEL_YAW = 4
uint8 MODE_ROLL_PITCH_YAW_THROTTLE = 5
uint8 MODE_ROLL_PITCH_YAWRATE_THROTTLE = 6
uint8 MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE = 7

# Additional control types - Does not use ROSflight controller
# i.e., these modes pass thrust/torque commands directly to ROSflight mixer
uint8 MODE_PASS_THROUGH = 8
uint8 MODE_ROLL_PITCH_YAW_THRUST_TO_MOTOR = 9
uint8 MODE_ROLLRATE_PITCHRATE_YAWRATE_THRUST_TO_MOTOR = 10


std_msgs/Header header

uint8 mode

float32 cmd1
float32 cmd2
float32 cmd3
float32 cmd4

bool cmd_valid

# @warning The following data fields are set by the controller by default.
# See the override parameters to use these commands
float32 phi_c
float32 theta_c