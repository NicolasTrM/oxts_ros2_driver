# OxTS NCom packet

std_msgs/Header header      # Standard ROS message header 

uint8   sync_byte  # sync_byte					:	u8()				;                 0  to 0
uint16  time # time						:	u16()	:	le()	;                 1  to 2
float32 acc_x  # acc_x			// 1e-4 m/s^2	:	i24() 	:	le()	; --3             3  to 5
float32 acc_y  # acc_y			// 1e-4 m/s^2	:	i24() 	:	le()	; --6             6  to 8
float32 acc_z  # acc_z			// 1e-4 m/s^2	:	i24()	:	le()	; --9             9  to 11
float32 ang_rate_x   # ang_rate_x			// 1e-5 rad/s	:	i24() 	:	le()	; --12        12 to 14
float32 ang_rate_y   # ang_rate_y			// 1e-5 rad/s	:	i24() 	:	le()	; --15        15 to 17
float32 ang_rate_z   # ang_rate_z			// 1e-5 rad/s	:	i24() 	:	le()	; --18        18 to 20
uint8   status_packet    # status_packet				:	u8()				; --21            21 to 21
# check_sum_1					:	u8()				; --22        22 to 22
float64 lat      # lat 					    	:	f64() 	:	le()	; -- 23           23 to 30 
float64 lon      # lon 					    	:	f64() 	:	le()	; -- 31           31 to 38
float32  height   # height   					:	u32()   :   le()  	; -- 39           39 to 42
float32   north_vel  # north_vel			// 1e-4 m/s	:	i24() 	:	le()	; -- 43           43 to 45
float32   east_vel   # east_vel			// 1e-4 m/s	:	i24() 	:	le()	; -- 46           46 to 48
float32   vert_vel   # vert_vel			// 1e-4 m/s	:	i24() 	:	le()	; -- 49           49 to 51
float32   north_yaw  # north_yaw			// 1e-6 rad	:	i24() 	:	le()	; -- 52           52 to 54
float32   pitch      # pitch		                // 1e-6 rad    :	i24() 	:	le()	; -- 55           55 to 57
float32   roll       # roll				// 1e-6 rad   	:	i24() 	:	le()	; -- 58           58 to 60
# check_sum_2					:	u8()				; -- 61       61 to 61
# status_channel				:	u8()				; -- 62       62 to 62
# minutes_from_gps_start			:	u32()	:	le()	; -- 63       63 to 66
# undulation     		// 5e-3 m	:	i16()			; -- 67       67 to 68
# HDOP		         	// 1e-1	:	u8()			; -- 69       69
# PDOP		         	// 1e-1	:	u8()			; -- 70       70
# check_sum_3					:	u8()				; -- 71       71 to 71
