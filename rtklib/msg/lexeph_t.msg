 # QZSS LEX ephemeris type 

time toe            # epoch time (GPST)  
time tof            # message frame time (GPST) 

int32 sat           # satellite  

uint8 health        # signal health (L1,L2,L1C,L5,LEX)
uint8 ura           # URA index 

float64[3] pos      # satellite position (m) 
float64[3] vel      # satellite velocity (m/s)
float64[3] acc      # satellite acceleration (m/s2)
float64[3] jerk     # satellite jerk (m/s3) 
float64 af0         # satellite clock bias 
float64 af1         # satellite clock drift (s,s/s) 
float64 tgd         # TGD 
float64[8] isc      # ISC 