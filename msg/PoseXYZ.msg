Header header

float64 x # +x: East
float64 y # +y: South
float64 z # +z: Upwards
float64 theta # XY-plane headings, in radians; 0: East, pi: North
float64 linear_velocity
float64 angular_velocity # in radians / s
float64 linear_velocity_z

# the following are for DRIVE_CTRL_MODE only
float64 linear_acceleration
float64 throttle # [0, 1] percentage of throttle to apply
float64 brake    # [0, 1] percentage of brake to apply
