import numpy as np
pose_unit = [np.cos(np.pi/2), np.sin(np.pi/2)]

bbox_vect = [-1-0, 0-0 ]

area =np.cross(pose_unit, bbox_vect)
print(area)
if area > 0:
        print("Left")
else:
    print("right")

    