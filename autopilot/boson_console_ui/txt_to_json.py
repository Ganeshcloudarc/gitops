import json
from geojson import Polygon,dump,MultiLineString,LineString
from geojson import Point, Feature, FeatureCollection, dump
import sys

def get_waypoints(file):
    '''
    get way points as a list 
    '''
    file = open(file, 'r')
    gps_way_point_list = []
    
    lines = file.readlines()
    for line in lines:
        line = line.strip()
        lat,lon = line.split()
        gps_way_point_list.append([float(lon),float(lat)])

    return gps_way_point_list


if __name__=="__main__":
    if len(sys.argv) < 2:
        print('Please specify the path')
        sys.exit()
    input_path = sys.argv[1]
    tar_waypoints_list = get_waypoints(input_path)

    points=LineString(tar_waypoints_list)
    output_name = 'us.json'

    with open(output_name, 'w') as f:
        dump(points, f)

