# navigation data type

Ephemeris[] eph                         # GPS/QZS/GAL ephemeris
GlonassEphemeris[] geph                       # GLONASS ephemeris
SbasEphemeris[] seph                       # SBAS ephemeris
PreciseEphemeris[] peph                       # precise ephemeris
PreciseClock[] pclk                       # precise clock
Almanac[] alm                         # almanac data
TotalElectronContent[] tec                         # TEC grid data
gtec_t[] gtec                       # G_TEC grid data

float64[4] utc_gps                  # GPS delta-UTC parameters {A0,A1,T,W}
float64[4] utc_glo                  # GLONASS UTC GPS time parameters
float64[4] utc_gal                  # Galileo UTC GPS time parameters
float64[4] utc_qzs                  # QZS UTC GPS time parameters
float64[8] ion_gps                  # GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
float64[4] ion_gal                  # Galileo iono model parameters {ai0,ai1,ai2,0}
float64[8] ion_qzs                  # QZSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
int32 leaps                         # leap seconds (s)

AntennaParameter[] pcvs             # satellite antenna pcv
sbssat_t sbssat                     # SBAS satellite corrections
sbsion_t[] sbsion                   # SBAS ionosphere corrections
DifferentialGnss[] dgps_t           # DGPS corrections
ssr_t[] ssr                         # SSR corrections
QzssEphemeris[] lexeph              # LEX ephemeris
QzssIonosphereCorrection lexion     # LEX ionosphere correction

lam[] lamcwl                        # carrier wave lengths (m)
cbias[] codebias                    # code bias (0:p1-p2,1:p1-c1,2:p2-c2) (m)