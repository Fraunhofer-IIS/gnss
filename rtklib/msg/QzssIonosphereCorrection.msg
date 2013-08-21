# QZSS LEX ionosphere correction type

time t0                      # epoch time (GPST)
float64 tspan                # valid time span (s)
float64[2] pos0              # reference position {lat,lon} (rad)
float64[6] coefs             # coefficients lat x lon (3 x 2)