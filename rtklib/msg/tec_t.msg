# TEC type

time time			# epoch time (GPST)

int32[3] ndata      # TEC grid data size {nlat,nlon,nhgt}

float64 rb          # earth radius (km)
float64[3] lats     # latitude start/interval (deg)
float64[3] lons     # longitude start/interval (deg)
float64[3] hgts     # heights start/interval (km)
float64[] data      # TEC grid data (tecu)
float32[] rms       # RMS values (tecu)