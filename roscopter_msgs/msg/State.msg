# State message for ROScopter.

std_msgs/Header header

float32[3] position	# north, east, down (m)
float32 u       # Body x velocity (m/s)
float32 v       # Body y velocity (m/s)
float32 w       # Body z velocity (m/s)
float32 vg		# Groundspeed (m/s)

float32 p		# Body frame rollrate (rad/s)
float32 q		# Body frame pitchrate (rad/s)
float32 r		# Body frame yawrate (rad/s)
float32 phi		# Roll angle (rad)
float32 theta   # Pitch angle (rad)
float32 psi		# Yaw angle (rad)

float32 bx		# Gyro bias x
float32 by		# Gyro bias y
float32 bz		# Gyro bias z

float32[4] quat		# Quaternion (wxyz, NED)
bool quat_valid		# Quaternion valid

float32 initial_lat 	# Initial/origin latitude (lat. deg)
float32 initial_lon 	# Initial/origin longitude (lon. deg) 
float32 initial_alt 	# Initial/origin altitude (m) 
