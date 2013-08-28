# antenna parameter type

int32 sat              # satellite number (0:receiver)
int8[] type            # antenna type
int8[] code            # serial number or satellite code

time ts                # valid time start
time te                # valid time end

float64[3] off         # phase center offset e/n/u or x/y/z (m)
float64[19] var        # phase center variation (m)
                       # el=90,85,...,0 or nadir=0,1,2,3,... (deg)