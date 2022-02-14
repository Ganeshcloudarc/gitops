import numpy as np

def get_bearing(lat1, lon1, lat2, lon2):
    
		lat_start = np.radians(lat1)
		lon_start = np.radians(lon1)
		lat_end = np.radians(lat2)
		lon_end = np.radians(lon2)
		dLat = lat_end - lat_start
		dLon = lon_end - lon_start

		y = np.sin(dLon)*np.cos(lat_end)
		x = np.cos(lat_start)*np.sin(lat_end) - np.sin(lat_start)*np.cos(lat_end)*np.cos(dLon)
		bearing = np.degrees(np.arctan2(y,x))
		return bearing

def get_distance(lat1, lon1, lat2, lon2):
    
		R = 6371.0*1000.0
		lat_start = np.radians(lat1)
		lon_start = np.radians(lon1)
		lat_end = np.radians(lat2)
		lon_end = np.radians(lon2)
		dLat = lat_end - lat_start
		dLon = lon_end - lon_start

		a = np.sin(dLat/2.0)*np.sin(dLat/2.0) + np.cos(lat_start)*np.cos(lat_end)*np.sin(dLon/2.0)*np.sin(dLon/2.0)
		c = 2.0*np.arctan2(np.sqrt(a),np.sqrt(1-a))

		d = c*R

		return d

def ConvertTo180Range( deg):
	deg = ConvertTo360Range(deg)
	if deg > 180.0:
		deg = -(180.0 - (deg%180.0))

	return deg

def ConvertTo360Range(deg):

	# if deg < 0.0:
	deg = deg%360.0

	return deg


def find_smallest_diff_ang(goal, cur):

	## goal is in 180ranges, we need to convert to 360ranges first

	diff_ang1 = abs(ConvertTo360Range(goal) - cur)

	if diff_ang1 > 180.0:

		diff_ang = 180.0 - (diff_ang1%180.0)
	else:
		diff_ang = diff_ang1

	## check closet direction
	compare1 = ConvertTo360Range(ConvertTo360Range(goal) - ConvertTo360Range(cur + diff_ang))
	compare2 = ConvertTo180Range(goal - ConvertTo180Range(cur + diff_ang))
	# print(compare1, compare2)
	if (abs(compare1) < 0.5) or (compare1 == 360.0) or (abs(compare2) < 0.5) or (compare2 == 360.0):
		sign = 1.0 # clockwise count from current hdg to target
	else:
		sign = -1.0 # counter-clockwise count from current hdg to target

	return diff_ang, sign


def map_with_limit(self, val, in_min, in_max, out_min, out_max):

	# out = ((val - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min
	## in_min must be the minimum input 
	## in_max must be the maximum input

	## out_min is supposed to map with in_min value
	## out_max is sipposed to map with in_max value
	## out_min can be less/more than out_max, doesn't matter


	m = (out_max - out_min)/(in_max - in_min)
	out = m*(val - in_min) + out_min

	if out_min > out_max:
		if out > out_min:
			out = out_min
		elif out < out_max:
			out = out_max
		else:
			pass
	elif out_max > out_min:
		if out > out_max:
			out = out_max
		elif out < out_min:
			out = out_min
		else:
			pass
	else:
		pass
    # print(m, val, in_min, in_max, out_min, out_max)
	return out


# 35.8414803
# Lon: 139.5241089
# 1
if __name__ =="__main__":
	Lat1=13.593735
	Lon1= 80.000384 
	# # 2
	Lat2=13.593703745599896
	Lon2= 80.00040273815955

	dis = get_distance(Lat1,Lon1,Lat2,Lon2)
	print(dis)
	# bearing = get_bearing(Lat1,Lon1,Lat2,Lon2)
	# print(bearing)
	################################ performance of find_smallest_diff_ang ####################
	# print(find_smallest_diff_ang(60,90))
	# print(find_smallest_diff_ang(60,30))
	# print(find_smallest_diff_ang(60,239))
	# print(find_smallest_diff_ang(60,300))

	# (30.0, -1.0)
	# (30.0, 1.0)
	# (179.0, -1.0)
	# (120.0, 1.0)
	##############################################################


