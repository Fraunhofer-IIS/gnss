 # SSR correction type

time t0             # epoch time (GPST)
float64[] udi       # SSR update interval (s)
int32[] iod
int32 iode          # issue of data
int32 ura           # URA indicator
int32 refd          # sat ref datum (0:ITRF,1:regional)
float64[] deph     # delta orbit {radial,along,cross} (m)
float64[] ddeph    # dot delta orbit {radial,along,cross} (m/s)
float64[] dclk     # delta clock {c0,c1,c2} (m,m/s,m/s^2)
float64 hrclk       # high-rate clock corection (m)
float32[] cbias     # code biases (m)
uint8 update        # update flag (0:no update,1:update)