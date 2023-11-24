import numpy as np
import matplotlib.pyplot as plt
robot_position = [0, 0, np.radians(0)]
wheel_base = L = 2.5
speed = 1  # m/s

# steering_angle = a = np.arange(-np.radians(30), np.radians(30), 2*0.01744444444444444444)
for i in range(30,-30,-2):
    if i == 0:
        i = -0.1
    steering_angle = a = np.radians(i)

    print("steering_angle",steering_angle)
    # print(steering_angle)
    tyre_dis = d = 10


    turning_angle = b = d/L * np.tan(a)
    print("turning_angle", turning_angle)
    turning_radius = R = d/b
    print("turning_radius", turning_radius)

    cx = robot_position[0] - np.sin(robot_position[2]) * R
    cy = robot_position[1] + np.cos(robot_position[2]) * R

    print(f"cx {cx} cy {cy}")
    # c = np.arange(0, np.radians(turning_angle), 0.01744444444444444444)

    arg_lenght = abs(b) * R
    print("arg_lenght", arg_lenght)
    number_of_points = int(arg_lenght / 1)
    try:
        inc_angle = b / number_of_points
    except Exception as error:
        print("error", error)
        inc_angle = 0.1
    print("inc_angle", inc_angle)
    # inc_angle = 0.1

    c = np.arange(0, b+np.sign(b)*inc_angle, np.sign(b)*inc_angle)
    # c = b
    # c = np.arange(0, b, inc_angle)

    print("c", c)
    x = cx + np.sin(robot_position[2] + c) * R
    y = cy - np.cos(robot_position[2] + c) * R
    theta = robot_position[2] + c

    print(f"x : {x} y : {y} theta : {theta}")

    # for small steering angle keep b = 0 and use above formulas

    plt.scatter(x,y)
    plt.plot(x, y)
    print("i", i)
plt.xlim(0,15)
plt.ylim(-10,10)
plt.show()