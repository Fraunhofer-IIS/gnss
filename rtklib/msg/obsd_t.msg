 # observation data records

time time		# receiver sampling time (GPST)
uint8 sat 		# satellite
uint8 rcv       # receiver number
uint8[] SNR 	# signal strength (0.25 dBHz)
uint8[] LLI		# loss of lock indicator
uint8[] code 	# code indicator (CODE_???)
float64[] L    	# observation data carrier-phase (cycle)
float64[] P    	# observation data pseudorange (m)
float32[] D    	# observation data doppler frequency (Hz)
