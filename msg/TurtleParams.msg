float64 spatial_scale # if <=0, use default scale==1

int8 ctrl_mode # if false then controlled via VelocityXYZ
int8 VELOCITY_CTRL_MODE = 0
int8 DRIVE_CTRL_MODE = 1

# Model for throttle/brake -> state/velocity/acceleration:
# http://www.asawicki.info/Mirror/Car%20Physics%20for%20Games/Car%20Physics%20for%20Games.html

float64 accel_throttle   # >= 0, in m/s^2
float64 decel_brake      # >= 0, in m/s^2 
float64 coeff_drag       # >= 0, produces negative force proportional to squared speed
float64 coeff_resistance # >= 0, produces negative force proportional to speed
float64 mass             # >= 0, used to convert drag and resistance forces into deceleration (thus recommend setting to 1 and adjusting coeff_ accordingly)