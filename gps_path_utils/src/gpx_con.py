#!/usr/bin/env python3
import numpy as np
import argparse
import matplotlib.pyplot as plt
import os
import bagpy
from bagpy import bagreader
import pandas as pd
import math
import sys
import json
from gpx_converter import Converter
from pathlib import Path


def distance_in_km(lat1,lon1,lat2,lon2):
    """
    Calculate the Haversine distance.
    Parameters
    ----------
    origin : tuple of float
        (lat, long)
    destination : tuple of float
        (lat, long)
    Returns
    -------
    distance_in_km : float
    Examples
    --------
    >>> origin = (48.1372, 11.5756)  # Munich
    >>> destination = (52.5186, 13.4083)  # Berlin
    >>> round(distance(origin, destination), 1)
    504.2
    """
    #lat1, lon1 = origin
    #lat2, lon2 = destination
    lat1= lat1/10**7
    lat2 = lat2/10**7
    lon1= lon1/10**7
    lon2=lon2/10**7
    radius = 6371000  # m
    #radius = 3959 # miles

    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) * math.sin(dlat / 2) +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) * math.sin(dlon / 2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = radius * c

    return d

parser = argparse.ArgumentParser()

# parser.add_argument('-b', '--rosbag', required=True,
#     help="the rosbag file")
parser.add_argument('-m', '--mission_file', required=True,
    help="the mission json  file")

args = parser.parse_args()

f = open(args.mission_file)
data = json.load(f)
f.close()

mission_lat=[]
mission_lon=[]
idx=0
for idx, cord in enumerate(data['coordinates']):
	mission_lat.append(cord[1])
	mission_lon.append(cord[0])
	if (len(mission_lat)== 2000):
		Converter.dataframe_to_gpx(input_df=pd.DataFrame({'lat':mission_lat, 'lon':mission_lon}),
                           lats_colname='lat',
                           longs_colname='lon',
                           output_file="%s_%d.gpx" %(Path(args.mission_file).stem,idx))	
		mission_lat=[]
		mission_lon=[]

if (len(mission_lat)>0):
	Converter.dataframe_to_gpx(input_df=pd.DataFrame({'lat':mission_lat, 'lon':mission_lon}),
                           lats_colname='lat',
                           longs_colname='lon',
                           output_file="%s_%d.gpx" %(Path(args.mission_file).stem,idx))	