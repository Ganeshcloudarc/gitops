def slope(dx, dy):
    return (dy / dx) if dx else None


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    # def __int__(self, L):
    #     self.x = L[0]
    #     self.y = L[1]

    def __str__(self):
        return '({}, {})'.format(self.x, self.y)

    def __repr__(self):
        return 'Point({}, {})'.format(self.x, self.y)

    def halfway(self, target):
        midx = (self.x + target.x) / 2
        midy = (self.y + target.y) / 2
        return Point(midx, midy)

    def distance(self, target):
        dx = target.x - self.x
        dy = target.y - self.y
        return (dx * dx + dy * dy) ** 0.5

    def reflect_x(self):
        return Point(-self.x, self.y)

    def reflect_y(self):
        return Point(self.x, -self.y)

    def reflect_x_y(self):
        return Point(-self.x, -self.y)

    def slope_from_origin(self):
        return slope(self.x, self.y)

    def slope(self, target):
        return slope(target.x - self.x, target.y - self.y)

    def y_int(self, target):  # <= here's the magic
        return self.y - self.slope(target) * self.x

    def line_equation(self, target):
        slope = self.slope(target)

        y_int = self.y_int(target)
        if y_int < 0:
            y_int = -y_int
            sign = '-'
        else:
            sign = '+'

        return 'y = {}x {} {}'.format(slope, sign, y_int)

    def line_function(self, target):
        slope = self.slope(target)
        y_int = self.y_int(target)

        def fn(x):
            return slope * x + y_int

        return fn


if __name__ == "__main__":
    a = Point(2., 2.)
    b = Point(4., 3.)

    print(a)  # => (2.0, 2.0)
    print(repr(b))  # => Point(4.0, 3.0)
    print(a.halfway(b))  # => (3.0, 2.5)

    print(a.slope(b))  # => 0.5
    print(a.y_int(b))  # => 1.0
    print(a.line_equation(b))  # => y = 0.5x + 1.0

    line = a.line_function(b)
    print(line(x=6.))  # => 4.0
