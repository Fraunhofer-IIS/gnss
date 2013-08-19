# SBAS long term satellite error correction type

time t0			 	# correction time

int32 iode          # IODE (issue of date ephemeris)

float64[3] dpos     # delta position (m) (ecef)
float64[3] dvel    	# delta velocity (m/s) (ecef)
float64 daf0		# delta clock-offset(s,s/s)
float64 daf1s 		# delta drift (s,s/s)