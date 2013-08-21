# station parameter type

std_msgs/String name           # marker name 
std_msgs/String marker         # marker number 
std_msgs/String antdes         # antenna descriptor
std_msgs/String antsno         # antenna serial number
std_msgs/String rectype        # receiver type descriptor
std_msgs/String recver         # receiver firmware version 
std_msgs/String recsno         # receiver serial number 

int32 antsetup                 # antenna setup id 
int32 itrf                     # ITRF realization year
int32 deltype                  # antenna delta type (0:enu,1:xyz)

geometry_msgs/Vector3 pos      # station position (ecef) (m) 
geometry_msgs/Vector3 del      # antenna position delta (e/n/u or x/y/z) (m)
int64 hgt                      # antenna height (m) 