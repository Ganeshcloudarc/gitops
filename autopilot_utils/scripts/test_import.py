if __name__ == '__main__':
    rospy.init_node('test_occ_grid')
    ogm = OccupancyGridManager('/semantics/costmap_generator/occupancy_grid',
                               subscribe_to_updates=False)
    print(ogm.width)
    print(ogm.height)
    rospy.Publisher("occ_check", PoseArray,)
    while True:

        # print(ogm.is_in_gridmap(100,500))
        print(ogm.origin)
        x,y = ogm.get_world_x_y(100,50)
        print(f"cost of x:{x},Y:{y}")

        # print(f"cost of x:{