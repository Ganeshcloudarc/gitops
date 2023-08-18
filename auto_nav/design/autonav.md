# We will write design doc of auto nav here.

## Filtering bounding boxes
 1> By gps reference path.

 2> By rectangle, 2 * vehicle_length, 3 *  vehicle_width
 
 
 

## Approach

    """
     1> find the closest point on the gps path.
     2> filter bounding boxes with gps path by tree spacing distance(width).
     3> find the closest left and right bboxes, then detect tree pairs
     4> if tree pair detected:
            center line
        elif only one side:
            offset line.
        else:
            follow the gps path.
     5> menuaver detections -
            -> left
            -> right
     6> Detection of pair of trees
    """


# Publising the center line
    # Reduce the speed based on heading diff and distance to center line from robot pose
    # robot_line , m = atan(yaw), c = substitutr
    # find the perp distance from robot_pose to line and check for heading using m.
    # TODO LIST
    # PO -> Send the avg values of center line as local trajectory
    # P1 -> Reduce the speed based of heading diff and distance to center line and robot line
    # P2 -> Check for M thereshould and Fix on best value, based on center line inliers.
    #      when both heading error and lateral offset are under some threshould and no inliers on center line
    # the condition to lock the row heading.
    # P3 -> Look into collisions on center line
    # P4 -> Find the turn point and make the turns based left or right and with minimum turning radius.