
from math import radians, atan2, sin, cos, sqrt

def dist_btw_coord(lat1, lon1, lat2, lon2):
    # Calculate the Haversine distance between two coordinates
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    radius = 6371.0  # Radius of the Earth in kilometers 
    distance = radius * c * 1000 #converting in meters (1km = 1000 meters)
    return distance

def calculate_initial_bearing(lat1, lon1, lat2, lon2):
    # Calculate the initial bearing between two coordinates
    dlon = lon2 - lon1
    x = sin(dlon) * cos(lat2)
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
    initial_bearing = atan2(x, y)
    initial_bearing = (initial_bearing + 2 * 3.14159) % (2 * 3.14159)
    initial_bearing = initial_bearing * (180.0 / 3.14159)
    if initial_bearing == 0:
       return 0 #collinear
    elif initial_bearing < 180:
       return 1 #same direction 
    else: 
       return -1 #opposite direction


def check_direction_cross_product(point1, point2):
  """Checks if two points are in the same or opposite direction using the cross product.

  Args:
    point1: A tuple representing the first point (x1, y1).
    point2: A tuple representing the second point (x2, y2).

  Returns:
    "Same direction" if the points are in the same direction,
    "Opposite direction" if the points are in opposite directions,
    "Cannot determine" if the points are equal.
  """

  if point1 == point2:
    return "Cannot determine"

  # Create vectors from the origin to each point
  vector1 = (point1[0], point1[1], 0)  # Assuming 2D coordinates
  vector2 = (point2[0], point2[1], 0)

  # Calculate the cross product
  cross_product = (
      vector1[1] * vector2[2] - vector1[2] * vector2[1],
      vector1[2] * vector2[0] - vector1[0] * vector2[2],
      vector1[0] * vector2[1] - vector1[1] * vector2[0]
  )

  # Check the sign of the z-component of the cross product
  if cross_product[2] == 0:
    return 0  # Points are collinear
  elif cross_product[2] > 0:
    return 1
  else:
    return -1

def check_direction_dot_product(point1, point2):
  """Checks if two points are in the same or opposite direction using the dot product.

  Args:
    point1: A tuple representing the first point (x1, y1).
    point2: A tuple representing the second point (x2, y2).

  Returns:
    "Same direction" if the points are in the same general direction,
    "Opposite direction" if the points are in generally opposite directions,
    "Cannot determine" if the points are equal or one of them is the origin.
  """

  if point1 == point2 or point1 == (0, 0) or point2 == (0, 0):
    return "Cannot determine"

  # Create vectors from the origin to each point
  vector1 = (point1[0], point1[1])
  vector2 = (point2[0], point2[1])

  # Calculate the dot product
  dot_product = vector1[0] * vector2[0] + vector1[1] * vector2[1]

  # Check the sign of the dot product
  if dot_product == 0: 
     return 0 
  elif dot_product > 0:
    return 1
  else:
    return -1
