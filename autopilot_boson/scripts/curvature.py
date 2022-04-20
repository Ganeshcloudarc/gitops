'''
# Python3 implementation of the approach
from math import sqrt

# Function to find the circle on
# which the given three points lie
def findCircle(x1, y1, x2, y2, x3, y3) :
    x12 = x1 - x2;
    x13 = x1 - x3;

    y12 = y1 - y2;
    y13 = y1 - y3;

    y31 = y3 - y1;
    y21 = y2 - y1;

    x31 = x3 - x1;
    x21 = x2 - x1;

    # x1^2 - x3^2
    sx13 = pow(x1, 2) - pow(x3, 2);

    # y1^2 - y3^2
    sy13 = pow(y1, 2) - pow(y3, 2);

    sx21 = pow(x2, 2) - pow(x1, 2);
    sy21 = pow(y2, 2) - pow(y1, 2);

    f = (((sx13) * (x12) + (sy13) *
        (x12) + (sx21) * (x13) +
        (sy21) * (x13)) // (2 *
        ((y31) * (x12) - (y21) * (x13))));
            
    g = (((sx13) * (y12) + (sy13) * (y12) +
        (sx21) * (y13) + (sy21) * (y13)) //
        (2 * ((x31) * (y12) - (x21) * (y13))));

    c = (-pow(x1, 2) - pow(y1, 2) -
        2 * g * x1 - 2 * f * y1);

    # eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0
    # where centre is (h = -g, k = -f) and
    # radius r as r^2 = h^2 + k^2 - c
    h = -g;
    k = -f;
    sqr_of_r = h * h + k * k - c;

    # r is the radius
    r = round(sqrt(sqr_of_r), 5);

    print("Centre = (", h, ", ", k, ")");
    print("Radius = ", r);
    print("curvature=",1/r);

# Driver code
if __name__ == "__main__" :
	
	x1 = 1 ; y1 = 1;
	x2 = 2 ; y2 = 4;
	x3 = 5 ; y3 = 3;
	findCircle(x1, y1, x2, y2, x3, y3);

# This code is contributed by Ryuga
'''


'''
Curvature, circumradius, and circumcenter functions
written by Hunter Ratliff on 2019-02-03
'''

def curvature(x_data,y_data):
    '''
    Calculates curvature for all interior points
    on a curve whose coordinates are provided
    Input:
        - x_data: list of n x-coordinates
        - y_data: list of n y-coordinates
    Output:
        - curvature: list of n-2 curvature values
    '''
    curvature = []
    for i in range(1,len(x_data)-1):
        R = circumradius(x_data[i-1:i+2],y_data[i-1:i+2])
        if ( R == 0 ):
            print('Failed: points are either collinear or not distinct')
            return 0
        curvature.append(1/R)
    return curvature

def circumradius(xvals,yvals):
    '''
    Calculates the circumradius for three 2D points
    '''
    x1, x2, x3, y1, y2, y3 = xvals[0], xvals[1], xvals[2], yvals[0], yvals[1], yvals[2]
    den = 2*((x2-x1)*(y3-y2)-(y2-y1)*(x3-x2))
    num = ( (((x2-x1)**2) + ((y2-y1)**2)) * (((x3-x2)**2)+((y3-y2)**2)) * (((x1-x3)**2)+((y1-y3)**2)) )**(0.5)
    if ( den == 0 ):
        print('Failed: points are either collinear or not distinct')
        return 0
    R = abs(num/den)
    return R

def circumcenter(xvals,yvals):
    '''
    Calculates the circumcenter for three 2D points
    '''
    x1, x2, x3, y1, y2, y3 = xvals[0], xvals[1], xvals[2], yvals[0], yvals[1], yvals[2]
    A = 0.5*((x2-x1)*(y3-y2)-(y2-y1)*(x3-x2))
    if ( A == 0 ):
        print('Failed: points are either collinear or not distinct')
        return 0
    xnum = ((y3 - y1)*(y2 - y1)*(y3 - y2)) - ((x2**2 - x1**2)*(y3 - y2)) + ((x3**2 - x2**2)*(y2 - y1))
    x = xnum/(-4*A)
    y =  (-1*(x2 - x1)/(y2 - y1))*(x-0.5*(x1 + x2)) + 0.5*(y1 + y2)
    return x, y

# test values
x = [0,0.8,2.8]
y = [0,1.6,-0.4]

print(curvature(x,y))
print(circumradius(x[0:3],y[0:3]))
print(circumcenter(x[0:3],y[0:3]))

