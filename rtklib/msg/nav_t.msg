# navigation data type

int32 n                     # number of broadcast ephemeris
int32 nmax
int32 ng                    # number of glonass ephemeris
int32 ngmax
int32 ns                    # number of sbas ephemeris
int32 nsmax
int32 ne                    # number of precise ephemeris
int32 nemax
int32 nc                    # number of precise clock
int32 ncmax
int32 na                    # number of almanac data
int32 namax
int32 nt                    # number of tec data
int32 ntmax
int32 nn                    # number of g_tec data
int32 nnmax

eph_t[] eph                 # GPS/QZS/GAL ephemeris
geph_t[] geph               # GLONASS ephemeris
seph_t[] seph               # SBAS ephemeris
peph_t[] peph               # precise ephemeris
pclk_t[] pclk               # precise clock
alm_t[] alm                 # almanac data
tec_t[] tec                 # TEC grid data
gtec_t[] gtec               # G_TEC grid data

float64[4] utc_gps          # GPS delta-UTC parameters {A0,A1,T,W}
float64[4] utc_glo          # GLONASS UTC GPS time parameters
float64[4] utc_gal          # Galileo UTC GPS time parameters
float64[4] utc_qzs          # QZS UTC GPS time parameters
float64[4] ion_gps          # GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
float64[4] ion_gal          # Galileo iono model parameters {ai0,ai1,ai2,0}
float64[4] ion_qzs          # QZSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
int32 leaps                 # leap seconds (s)

pcv_t[] pcvs                # satellite antenna pcv
sbssat_t sbssat             # SBAS satellite corrections
sbsion_t[] sbsion           # SBAS ionosphere corrections
dgps_t[] dgps_t             # DGPS corrections
ssr_t[] ssr                 # SSR corrections
lexeph_t[] lexeph           # LEX ephemeris
lexion_t lexion             # LEX ionosphere correction

#float64[][] lam            # carrier wave lengths (m) 
#float64[][] cbias          # code bias (0:p1-p2,1:p1-c1,2:p2-c2) (m)