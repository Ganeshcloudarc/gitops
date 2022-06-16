#!/usr/bin/env python
import time

import rospy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import numpy as np
from itertools import product
from .tf_helper import current_robot_pose, get_yaw, convert_point
import math

"""
Class to deal with OccupancyGrid in Python
as in local / global costmaps.

Author: Ramana ramanab@bosonmotors.com
"""


class OccupancyGridManager(object):
    def __init__(self, topic, subscribe_to_updates=False, base_frame='base_link', world_frame="map"):
        # OccupancyGrid starts on lower left corner
        self._grid_data = None
        self._occ_grid_metadata = None
        self._reference_frame = None
        self._base_frame = base_frame
        self._world_frame = world_frame
        self._sub = rospy.Subscriber(topic, OccupancyGrid,
                                     self._occ_grid_cb,
                                     queue_size=1)
        if subscribe_to_updates:
            rospy.loginfo("Subscribing to updates!")
            self._updates_sub = rospy.Subscriber(topic + '_updates',
                                                 OccupancyGridUpdate,
                                                 self._occ_grid_update_cb,
                                                 queue_size=1)
        rospy.loginfo("Waiting for '" +
                      str(self._sub.resolved_name) + "'...")
        while self._occ_grid_metadata is None and \
                self._grid_data is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("OccupancyGridManager for '" +
                      str(self._sub.resolved_name) +
                      "' initialized!")
        rospy.loginfo("Height (y / rows): " + str(self.height) +
                      ", Width (x / columns): " + str(self.width) +
                      ", starting from bottom left corner of the grid. " +
                      " Reference_frame: " + str(self.reference_frame) +
                      " origin: " + str(self.origin))

    @property
    def resolution(self):
        return self._occ_grid_metadata.resolution

    @property
    def width(self):
        return self._occ_grid_metadata.width

    @property
    def height(self):
        return self._occ_grid_metadata.height

    @property
    def origin(self):
        return self._occ_grid_metadata.origin

    @property
    def reference_frame(self):
        return self._reference_frame

    def _occ_grid_cb(self, data):
        # rospy.loginfo("Got a full OccupancyGrid update")
        self._occ_grid_metadata = data.info
        # Contains resolution, width & height
        # np.set_printoptions(threshold=99999999999, linewidth=200)
        # data comes in row-major order http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
        # first index is the row, second index the column
        self._grid_data = np.array(data.data,
                                   dtype=np.int8).reshape(data.info.height,
                                                          data.info.width)
        self._reference_frame = data.header.frame_id
        # print(self._grid_data)

    def _occ_grid_update_cb(self, data):
        # rospy.loginfo("Got a partial OccupancyGrid update")
        # x, y origin point of the update
        # width and height of the update
        # data, the update
        # data comes in row-major order http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
        # first index is the row, second index the column
        data_np = np.array(data.data,
                           dtype=np.int8).reshape(data.height, data.width)
        self._grid_data[data.y:data.y +
                               data.height, data.x:data.x + data.width] = data_np
        # print(self._grid_data)

    # def get_world_x_y(self, costmap_x, costmap_y):
    #     world_x = costmap_x * self.resolution + self.origin.position.x
    #     world_y = costmap_y * self.resolution + self.origin.position.y
    #     return world_x, world_y

    def get_local_x_y(self, costmap_x, costmap_y):
        local_x = costmap_x * self.resolution + self.origin.position.x
        local_y = costmap_y * self.resolution + self.origin.position.y
        return local_x, local_y

    def get_world_x_y(self, costmap_x, costmap_y):
        if self._reference_frame == self._world_frame:
            local_x, local_y = self.get_local_x_y(costmap_x, costmap_y)
            return local_x, local_y
        else:
            pose = current_robot_pose(self._world_frame, self._base_frame)
            yaw = get_yaw(pose.orientation)
            cos_theta = math.cos(yaw)
            sin_theta = math.sin(yaw)
            local_x, local_y = self.get_local_x_y(costmap_x, costmap_y)
            world_x = pose.position.x + (local_x * cos_theta - local_y * sin_theta)
            world_y = pose.position.y + (local_x * sin_theta + local_y * cos_theta)
            return world_x, world_y

    def get_costmap_x_y(self, local_x, local_y):
        costmap_x = int(
            round((local_x - self.origin.position.x) / self.resolution))
        costmap_y = int(
            round((local_y - self.origin.position.y) / self.resolution))
        return costmap_x, costmap_y

    def get_cost_from_world_x_y(self, x, y):
        if self._reference_frame == self._world_frame:
            cx, cy = self.get_costmap_x_y(x, y)
        else:
            point = [x, y, 0]
            trans_point = convert_point(point, self._world_frame, self._base_frame)
            cx, cy = self.get_costmap_x_y(trans_point[0], trans_point[1])
        try:
            return self.get_cost_from_costmap_x_y(cx, cy)
        except IndexError as e:
            raise IndexError(
                "Coordinates out of grid (in frame: {}) x: {}, y: {} must be in between: [{}, {}], [{}, {}]. Internal error: {}".format(
                    self.reference_frame, x, y,
                    self.origin.position.x,
                    self.origin.position.x + self.height * self.resolution,
                    self.origin.position.y,
                    self.origin.position.y + self.width * self.resolution,
                    e))

    def get_cost_from_costmap_x_y(self, x, y):
        if self.is_in_gridmap(x, y):
            # data comes in row-major order http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
            # first index is the row, second index the column
            return self._grid_data[y][x]
        else:
            raise IndexError(
                "Coordinates out of gridmap, x: {}, y: {} must be in between: [0, {}], [0, {}]".format(
                    x, y, self.height, self.width))

    def is_in_gridmap(self, x, y):
        if -1 < x < self.width and -1 < y < self.height:
            return True
        else:
            return False

    def get_closest_cell_under_cost(self, x, y, cost_threshold, max_radius):
        """
        Looks from closest to furthest in a circular way for the first cell
        with a cost under cost_threshold up until a distance of max_radius,
        useful to find closest free cell.
        returns -1, -1 , -1 if it was not found.

        :param x int: x coordinate to look from
        :param y int: y coordinate to look from
        :param cost_threshold int: maximum threshold to look for
        :param max_radius int: maximum number of cells around to check
        """
        return self._get_closest_cell_arbitrary_cost(
            x, y, cost_threshold, max_radius, bigger_than=False)

    def get_closest_cell_over_cost(self, x, y, cost_threshold, max_radius):
        """
        Looks from closest to furthest in a circular way for the first cell
        with a cost over cost_threshold up until a distance of max_radius,
        useful to find closest obstacle.
        returns -1, -1, -1 if it was not found.

        :param x int: x coordinate to look from
        :param y int: y coordinate to look from
        :param cost_threshold int: minimum threshold to look for
        :param max_radius int: maximum number of cells around to check
        """
        return self._get_closest_cell_arbitrary_cost(
            x, y, cost_threshold, max_radius, bigger_than=True)

    def _get_closest_cell_arbitrary_cost(self, x, y,
                                         cost_threshold, max_radius,
                                         bigger_than=False):

        # Check the actual goal cell
        try:
            cost = self.get_cost_from_costmap_x_y(x, y)
        except IndexError:
            return None

        if bigger_than:
            if cost > cost_threshold:
                return x, y, cost
        else:
            if cost < cost_threshold:
                return x, y, cost

        def create_radial_offsets_coords(radius):
            """
            Creates an ordered by radius (without repetition)
            generator of coordinates to explore around an initial point 0, 0

            For example, radius 2 looks like:
            [(-1, -1), (-1, 0), (-1, 1), (0, -1),  # from radius 1
            (0, 1), (1, -1), (1, 0), (1, 1),  # from radius 1
            (-2, -2), (-2, -1), (-2, 0), (-2, 1),
            (-2, 2), (-1, -2), (-1, 2), (0, -2),
            (0, 2), (1, -2), (1, 2), (2, -2),
            (2, -1), (2, 0), (2, 1), (2, 2)]
            """
            # We store the previously given coordinates to not repeat them
            # we use a Dict as to take advantage of its hash table to make it more efficient
            coords = {}
            # iterate increasing over every radius value...
            for r in range(1, radius + 1):
                # for this radius value... (both product and range are generators too)
                tmp_coords = product(range(-r, r + 1), repeat=2)
                # only yield new coordinates
                for i, j in tmp_coords:
                    if (i, j) != (0, 0) and not coords.get((i, j), False):
                        coords[(i, j)] = True
                        yield (i, j)

        coords_to_explore = create_radial_offsets_coords(max_radius)

        for idx, radius_coords in enumerate(coords_to_explore):
            # for coords in radius_coords:
            tmp_x, tmp_y = radius_coords
            # print("Checking coords: " +
            #       str((x + tmp_x, y + tmp_y)) +
            #       " (" + str(idx) + " / " + str(len(coords_to_explore)) + ")")
            try:
                cost = self.get_cost_from_costmap_x_y(x + tmp_x, y + tmp_y)
            # If accessing out of grid, just ignore
            except IndexError:
                pass
            if bigger_than:
                if cost > cost_threshold:
                    return x + tmp_x, y + tmp_y, cost

            else:
                if cost < cost_threshold:
                    return x + tmp_x, y + tmp_y, cost

        return -1, -1, -1


if __name__ == '__main__':
    rospy.init_node('test_occ_grid')
    ogm = OccupancyGridManager('/semantics/costmap_generator/occupancy_grid',
                               subscribe_to_updates=False, base_frame='ego_vehicle')
    print(ogm.width)
    print(ogm.height)
    from geometry_msgs.msg import PoseArray, Pose

    pose_arr_pub = rospy.Publisher("occ_check", PoseArray, queue_size=10)
    while True:
        pose_arr_msg = PoseArray()
        pose_arr_msg.header.frame_id = "map"
        cost_arr = []
        # for i in range(ogm.width):
        for j in range(ogm.height):
            i = 10
            pose = Pose()
            x, y = ogm.get_world_x_y(j, i)
            pose.position.x = x
            pose.position.y = y
            cost = ogm.get_cost_from_world_x_y(x, y)

            if not ogm.get_cost_from_costmap_x_y(i, j) == cost:
                print("not same")

            cost_arr.append(cost)
            # if cost == 100:
            pose_arr_msg.poses.append(pose)
        pose_arr_pub.publish(pose_arr_msg)
        print(cost_arr)
        print("published")

        # # print(ogm.is_in_gridmap(100,500))
        # print(ogm.origin)
        # x,y = ogm.get_world_x_y(0,50)
        # print(f"cost of x:{x},Y:{y}")

        # print(f"cost of x:{x},Y:{y}: == {ogm.get_cost_from_world_x_y(x,y)}")
        # time.sleep(0.2)

    '''
    wx1, wy1 = ogm.get_world_x_y(0, 0)
    print("world from costmap coords  0 0: ")
    print((wx1, wy1))
    cx1, cy1 = ogm.get_costmap_x_y(wx1, wy1)
    print("back to costmap: ")
    print((cx1, cy1))

    cx2, cy2 = ogm.get_costmap_x_y(0.0, 0.0)
    print("costmap from world coords  0 0: ")

    print((cx2, cy2))
    wx2, wy2 = ogm.get_world_x_y(cx2, cy2)
    print("back to world: ")
    print((wx2, wy2))

    cost = ogm.get_cost_from_costmap_x_y(cx1, cy1)
    print("cost cx1, cy1: " + str(cost))
    cost = ogm.get_cost_from_world_x_y(wx1, wy1)
    print("cost wx1, wy1: " + str(cost))
    cost = ogm.get_cost_from_costmap_x_y(cx2, cy2)
    print("cost cx2, cy2: " + str(cost))
    cost = ogm.get_cost_from_world_x_y(wx2, wy2)
    print("cost wx2, wy2: " + str(cost))

    # known place right now
    cost = ogm.get_cost_from_world_x_y(0.307, -0.283)
    print("cost of know nplace is: " + str(cost))

    # cost = ogm.get_cost_from_world_x_y(6.485, -1.462)
    # print("cost of known place is: " + str(cost))
    # cx, cy = ogm.get_costmap_x_y(6.485, -1.462)
    # print("from costmap coords: " + str((cx, cy)))

    print("trying to access out of bounds")
    try:
        cost = ogm.get_cost_from_costmap_x_y(9999, 0)
        print(cost)
    except IndexError as e:
        print("We got, correctly, indexerror: " + str(e))
    try:
        cost = ogm.get_cost_from_costmap_x_y(0, 9999)
        print(cost)
    except IndexError as e:
        print("We got, correctly, indexerror: " + str(e))

    il = range(0, ogm.height)
    # reverse the list as the origin coordinate is bottom left
    il.reverse()
    for i in il:
        accum = ''
        l = range(0, ogm.width)
        # l.reverse()
        for j in l:
            accum += str(ogm.get_cost_from_costmap_x_y(i, j)) + ' '
            # print(ogm.get_cost_from_costmap_x_y(i, 270))
        print (accum)
'''
