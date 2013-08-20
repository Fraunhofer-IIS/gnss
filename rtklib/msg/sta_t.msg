# station parameter type

int8[] name     # marker name 
int8[] marker   # marker number 
int8[] antdes   # antenna descriptor
int8[] antsno   # antenna serial number
int8[] rectype  # receiver type descriptor
int8[] recver   # receiver firmware version 
int8[] recsno   # receiver serial number 

int32 antsetup       # antenna setup id 
int32 itrf           # ITRF realization year
int32 deltype        # antenna delta type (0:enu,1:xyz)

int64[3] pos       # station position (ecef) (m) 
int64[3] del       # antenna position delta (e/n/u or x/y/z) (m)
int64 hgt          #/* antenna height (m) 