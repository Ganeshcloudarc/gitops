import numpy as np
import math

"""
rosmsg show BoundingBoxArray
[jsk_recognition_msgs/BoundingBoxArray]:
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
jsk_recognition_msgs/BoundingBox[] boxes
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  geometry_msgs/Vector3 dimensions
    float64 x
    float64 y
    float64 z
  float32 value
  uint32 label
"""


def filter_bboxes_by_path(path, bboxes, radius):
    """
    filters the bboxes within a radius distance
    Parameters:
        path : numpy array of x,y coordinates
        bboxes: list of bbox
        radius : (float) radius of filtering (mostly tree spacing width)
    Returns:
        filtered_bboxes : numpy array of x,y coordinates of bboxes which are close to path by radius
    """
    filtered_index_list = []
    left_side = []
    right_side = []
    close_pt_x, close_pt_y = path[0][0], path[0][1]
    for i in range(len(path) - 6):
        path_x, path_y = path[i][0], path[i][1]
        path_vect = [path[i + 5][0] - path_x, path[i + 5][1] - path_y]
        for j in range(len(bboxes)):
            bx, by = bboxes[j].pose.position.x, bboxes[j].pose.position.y
            distance = math.hypot(path_x - bx, path_y - by)
            # print("distance", distance)
            if distance <= radius:
                # decide left or right
                bbox_vect = [bx - path_x, by - path_y]
                area = np.cross(path_vect, bbox_vect)
                if j not in filtered_index_list:
                    filtered_index_list.append(j)
                    if area > 0:
                        left_side.append(j)
                    else:
                        right_side.append(j)

    filtered_bboxes = [bboxes[i] for i in filtered_index_list]
    left_bboxes = [bboxes[i] for i in left_side]
    right_bboxes = [bboxes[i] for i in right_side]

    filtered_bboxes.sort(
        key=lambda bbox: math.hypot(close_pt_x - bbox.pose.position.x, close_pt_y - bbox.pose.position.y))
    left_bboxes.sort(key=lambda bbox: math.hypot(close_pt_x - bbox.pose.position.x, close_pt_y - bbox.pose.position.y))
    right_bboxes.sort(key=lambda bbox: math.hypot(close_pt_x - bbox.pose.position.x, close_pt_y - bbox.pose.position.y))

    return filtered_bboxes, left_bboxes, right_bboxes


def tree_pair_detection(left, right):
    pass


def devide_into_left_and_right_based_on_path(path, bboxes):
    pass


def devide_into_left_and_right_base_frame(pose, bboxes):
    pass


def filter_bboxes_by_pose_width_length_limits(pose, bboxes, width, length):
    """
    filters the bboxes with a rectangle of given width and length.
     Parameters:
         pose : (pose)
         bboxes: bounding boxes
         width: width of rectangle
         length: length of rectangle
     Returns:
         filtered_bboxes : numpy array of x,y coordinates of bboxes within a rectangle of given width and length.
    """
    pass


def filter_by_area(bboxes, min_area=0, max_area=100):
    """
    Filter the bboxes with area of bbox
    Parameters:
        bboxes: bboxes
        min_area: minimum area of bbox
        max_area: maximum area of bbox
    Returns:
        filtered_bboxes : bboxes whose areas greater than min_area and less than max_area
    """
    filtered_bboxes = []
    for box in bboxes.boxes:
        area = box.dimensions.x * box.dimensions.y
        if min_area <= area <= max_area:
            filtered_bboxes.append(box)
    return filtered_bboxes


def inside_radius(bboxes, radius):
    filtered_bboxes = []
    # filtered_bboxes.boxes =
    for i in range(len(bboxes.boxes)):
        dis = math.hypot(bboxes.boxes[i].pose.position.x, bboxes.boxes[i].pose.position.y)
        # print("dis", dis)
        if dis <= radius:
            filtered_bboxes.append(bboxes.boxes[i])
    return filtered_bboxes


def enlarge_bboxes(bboxes, scaling_factor):
    pass




if __name__ == "__main__":
    from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
    bboxes_arr = BoundingBoxArray()
    for j in range(1, 10):
        bbox = BoundingBox()
        bbox.pose.position.x = j
        bbox.pose.position.y = 0
        bboxes_arr.boxes.append(bbox)
    print("original bboxes ", len(bboxes_arr.boxes))

    new_bboxes = inside_radius(bboxes_arr, 8)
    print("new bboxes len", len(new_bboxes))








