# import matplotlib.pyplot as plt
# plt.title("Bresenham Algorithm")
# plt.xlabel("X Axis")
# plt.ylabel("Y Axis")
#
# def bres(x1,y1,x2,y2):
#     x,y = x1,y1
#     dx = abs(x2 - x1)
#     dy = abs(y2 -y1)
#     gradient = dy/float(dx)
#
#     if gradient > 1:
#         dx, dy = dy, dx
#         x, y = y, x
#         x1, y1 = y1, x1
#         x2, y2 = y2, x2
#
#     p = 2*dy - dx
#     print(f"x = {x}, y = {y}")
#     # Initialize the plotting points
#     xcoordinates = [x]
#     ycoordinates = [y]
#
#     for k in range(2, dx + 2):
#         if p > 0:
#             y = y + 1 if y < y2 else y - 1
#             p = p + 2 * (dy - dx)
#         else:
#             p = p + 2 * dy
#
#         x = x + 1 if x < x2 else x - 1
#
#         print(f"x = {x}, y = {y}")
#         xcoordinates.append(x)
#         ycoordinates.append(y)
#
#     plt.plot(xcoordinates, ycoordinates)
#     plt.show()
#
#
# def main():
#     x1 = int(input("Enter the Starting point of x: "))
#     y1 = int(input("Enter the Starting point of y: "))
#     x2 = int(input("Enter the end point of x: "))
#     y2 = int(input("Enter the end point of y: "))
#
#     bres(x1, y1, x2, y2)
#
# if __name__ == "__main__":
#     main()
import matplotlib.pyplot as plt


def draw_line(x1, y1, x2, y2):
    # Calculate differences and absolute differences between coordinates
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)

    # Calculate the direction of the line
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1

    # Initialize error
    error = dx - dy

    # Initialize coordinates
    x, y = x1, y1

    # Create a list to store the points on the line
    points = []

    # Add the first point (x1, y1)
    points.append((x, y))

    # Main loop to iterate through the line
    while x != x2 or y != y2:
        # Add the current point to the list
        # Calculate the next coordinates
        double_error = 2 * error
        if double_error > -dy:
            error -= dy
            x += sx
        if double_error < dx:
            error += dx
            y += sy
        points.append((x, y))

    return points


def bresenham_line(x1, y1, x2, y2):
    dx = abs(x1-x2)
    dy = abs(y1-y2)
    dy_2 = 2*dy
    temp = 2*dy - 2 * dx




# Example usage:
x1, y1 = -1, 1
x2, y2 = 10,10

points_on_line = draw_line(x1, y1, x2, y2)

# Print the points on the line
for point in points_on_line:
    print(point)

# Plot the line using Matplotlib
x_values, y_values = zip(*points_on_line)
plt.plot(x_values, y_values)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Bresenham Line Algorithm')
plt.show()
