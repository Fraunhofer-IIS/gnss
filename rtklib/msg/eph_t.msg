# GPS/QZS/GAL broadcast ephemeris type

int32 sat           	# satellite number */
int32 iode				# IODE,IODC */
int32 iodc
int32 sva            	# SV accuracy (URA index) */
int32 svh            	# SV health (0:ok) */
int32 week           	# GPS/QZS: gps week, GAL: galileo week */
int32 code           	# GPS/QZS: code on L2, GAL: data sources */
int32 flag           	# GPS/QZS: L2 P data flag */

# Toe,Toc,T_trans
time toe
time toc
time ttr

# SV orbit parameters
float64 A
float64 e
float64 i0
float64 OMG0
float64 omg
float64 M0
float64 deln
float64 OMGd
float64 idot
float64 crc
float64 crs
float64 cuc
float64 cus
float64 cic 
float64 cis

float64 toes 			# Toe (s) in week */
float64 fit        		# fit interval (h) */

# SV clock parameters (af0,af1,af2)
float64 f0
float64 f1
float64 f2

float64[4] tgd			# group delay parameters || GPS/QZS:tgd[0]=TGD
				 		# GAL:tgd[0]=BGD ||	5a/E1,tgd[1]=BGD E5b/E1

