#!/usr/bin/env python

import pyproj
import math
import sys
from optparse import OptionParser
from math import atan2, cos, sin, sqrt


def pythagoras(a, b):
    return sqrt(a**2 + b**2)


class WGS84():
    A = 6378137.0
    recf = 298.257223563
    B = A * (recf - 1.0) / recf
    eSquared = ((A * A) - (B * B)) / (A * A)
    e2Squared = ((A * A) - (B * B)) / (B * B)
    tn = (A - B) / (A + B)
    ap = A * (1.0 - tn + 5.0 * ((tn * tn) - (tn * tn * tn)) / 4.0 + 81.0 * ((tn * tn * tn * tn) - (tn * tn * tn * tn * tn)) / 64.0)
    bp = 3.0 * A * (tn - (tn * tn) + 7.0 * ((tn * tn * tn) - (tn * tn * tn * tn)) / 8.0 + 55.0 * (tn * tn * tn * tn * tn) / 64.0) / 2.0
    cp = 15.0 * A * ((tn * tn) - (tn * tn * tn) + 3.0 * ((tn * tn * tn * tn) - (tn * tn * tn * tn * tn)) / 4.0) / 16.0
    dp = 35.0 * A * ((tn * tn * tn) - (tn * tn * tn * tn) + 11.0 * (tn * tn * tn * tn * tn) / 16.0) / 48.0
    ep = 315.0 * A * ((tn * tn * tn * tn) - (tn * tn * tn * tn * tn)) / 512.0


def ecef_to_utm(x, y, z):
    xyz = (x, y, z)
    a2 = WGS84.A * WGS84.A
    b2 = WGS84.B * WGS84.B
    p = pythagoras(xyz[0], xyz[1])
    theta = atan2((xyz[2] * WGS84.A) , (p * WGS84.B))
    esq = 1.0 - (b2 / a2)
    epsq = (a2 / b2 - 1.0)
    sth = sin(theta)
    sth3 = sth**3
    cth = cos(theta)
    cth3 = cth**3
    lat = atan2((xyz[2] + epsq * WGS84.B * sth3) , (p - esq * WGS84.A * cth3))
    if (lat > (math.pi/2)):
        lat = math.pi - lat
    lon = atan2(xyz[1],xyz[0])
    hgt = (p / cos(lat)) - (a2 / sqrt(a2 * cos(lat) * cos(lat) +  b2 * sin(lat) * sin(lat)))
    lat = lat * (180/math.pi)
    lon = lon * (180/math.pi)
    return latlon_to_utm(lat, lon)


def latlon_to_utm(latitude, longitude, height=0):
    """calculate UTM projection"""
    lon_6 = math.floor(longitude / 6.0)
    utmXZone = (longitude <= 0.0) and (30 + lon_6) or (31 + lon_6)
    p = pyproj.Proj(proj='utm', zone=utmXZone, ellps='WGS84')
    easting, northing = p(longitude, latitude)
    return (easting, northing)

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option('--latitude', help='Latitude value in float degrees [-90.0:90]')
    parser.add_option('--longitude', help='Longitude value in float degrees [-180.0:180]')
    parser.add_option('-x', help='ECEF-X')
    parser.add_option('-y', help='ECEF-Y')
    parser.add_option('-z', help='ECEF-Z')
    (options, args) = parser.parse_args()
    if options.latitude and options.longitude:
        en = latlon_to_utm(float(options.latitude), float(options.longitude))
    elif options.x and options.y and options.z:
        en = ecef_to_utm(float(options.x), float(options.y), float(options.z))
    else:
        print("Select either latitude/longitude or x, y, z")
        sys.exit(1)
    print("easting northing: %f %f" % (en[0], en[1]))
