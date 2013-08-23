# The almanac consists of coarse orbit and status information for each satellite in the constellation, an ionospheric model, and information to relate GNSS derived time to Coordinated Universal Time (UTC)

int32 sat                     # satellite number
int32 svh                     # sv health (0:ok)
int32 svconf                  # as and sv config
int32 week                    # GPS/QZS: gps week, GAL: galileo week
time toa                      # Toa

# SV orbit parameters
float64 A
float64 e
float64 i0
float64 OMG0
float64 omg
float64 M0
float64 OMGd

float64 toas                  # Toa (s) in week

# SV clock parameters (af0,af1)
float64 f0
float64 f1