uint64 timestamp		# time since system start (microseconds)
bool vehicle_is_openloop       # if vehicle`s attitude is up to target,set to true
float32 sin_roll		# roll angle in the openloop.
bool pre_openloop
uint32 change_flags             #
bool change_att_sp
float32 takeoff_att_sp
uint8 CHANGE_ONEROTOR = 0
uint8 CHANGE_TWOROTOR = 1
uint8 REACH_45 = 2
uint8 REACH_40 = 3
