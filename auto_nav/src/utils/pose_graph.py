import math
import numpy

class PoseGraph:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.goal_cost = math.inf
        self.parent = None
        self.child = None

    def update_parent(self, parent):
        self.parent = parent

    def update_child(self, child):
        self.child = child

    def update_cost(self,cost):
        self.cost = cost
    def __str__(self):
        return f"x:{self.x} y : {self.y}, parnet : {self.parent}"

if __name__ == "__main__":
    p1 = PoseGraph(1, 1)
    print(p1)
    p2 = PoseGraph(2, 4)
    p2.update_parent(p1)
    p3 = PoseGraph(10, 10)
    p3.update_parent(p2)
    print(p3)



