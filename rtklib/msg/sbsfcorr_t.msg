# SBAS fast correction type

time t0         	# time of applicability (TOF)

float64 prc        	# pseudorange correction (PRC) (m)
float64 rrc         # range-rate correction (RRC) (m/s)
float64 dt  		# range-rate correction delta-time (s)

int32 iodf          # IODF (issue of date fast corr)

int16 udre         	# UDRE+1
int16 ai   			# degradation factor indicator