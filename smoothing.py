#!/usr/bin/env python3
from itertools import count
from matplotlib.animation import FuncAnimation
from scipy.interpolate import Rbf
from scipy.signal import savgol_filter
import math
import csv
import matplotlib.pyplot as plt

"""
This script can smooth and make comparison between original pointcloud and smoothed pointcloud.
"""

# This program is not included in ROS
cloudXYZ = None
cloudXYZ_sort = None
cloudXYZ_smooth = None
x,y,z = [],[],[]
x_sort,y_sort,z_sort = [],[],[]
xs,ys,zs = [],[],[]

# Plotting
ax = None

# Limit
x_min,x_max = -0.42,-0.05
y_min,y_max = 0,0.5
z_min,z_max = 0,0.35


# Sorting
refvec = [0,1]
origin = [0,0]
index = 0
index_sort = 0
index_smooth = 0

# Downsampling (only for smoothed cloud)
def downsampling(cloud,intervals):
    output = []
    for i in range(0,len(cloud)):
        if (i%intervals == 0):
            a = cloudXYZ_sort[i][0]
            b = cloudXYZ_sort[i][1]
            c = cloudXYZ_sort[i][2]
            output.append([a,b,c])
    return output

# Sorting operations
def clockwiseangle_and_distance(point):
    global origin,refvec
    # Vector between point and the origin: v = p - o
    vector = [point[0]-origin[0], point[1]-origin[1]]
    # Length of vector: ||v||
    lenvector = math.hypot(vector[0], vector[1])
    # If length is zero there is no angle
    if lenvector == 0:
        return -math.pi, 0
    # Normalize vector: v/||v||
    normalized = [vector[0]/lenvector, vector[1]/lenvector]
    dotprod  = normalized[0]*refvec[0] + normalized[1]*refvec[1]     # x1*x2 + y1*y2
    diffprod = refvec[1]*normalized[0] - refvec[0]*normalized[1]     # x1*y2 - y1*x2
    angle = math.atan2(diffprod, dotprod)
    # Negative angles represent counter-clockwise angles so we need to subtract them 
    # from 2*pi (360 degrees)
    if angle < 0:
        return 2*math.pi+angle, lenvector
    # I return first the angle because that's the primary sorting criterium
    # but if two vectors have the same angle then the shorter distance should come first.
    return angle, lenvector

# Read pointcloud saved in csv
def read_csv_cloud(file_name):
    cloud = []
    with open(file_name) as csvfile:
        rows = csv.reader(csvfile)
        for row in rows:
            x = float(row[0])
            y = float(row[1])
            z = float(row[2])
            if((x>x_min)and(x<x_max))and((y>y_min)and(y<y_max))and((z>z_min)and(z<z_max)):
                cloud.append([x,y,z]) 
    return cloud

# Seperate the cloud to X/Y/Z channel
def seperate_cloud_channel(cloud):
    ch_x,ch_y,ch_z = [],[],[]
    for i in range(0,len(cloud)):
        ch_x.append(cloud[i][0])
        ch_y.append(cloud[i][1])
        ch_z.append(cloud[i][2])
    return ch_x,ch_y,ch_z

# Drawing animation
def animate(k):
    global cloudXYZ_sort,cloudXYZ_smooth,cloudXYZ,ax
    global x,y,z
    global x_sort,y_sort,z_sort
    global xs,ys,zs
    global index,index_sort,index_smooth

    # Remove extreme points
    a = cloudXYZ[k][0]
    b = cloudXYZ[k][1]
    c = cloudXYZ[k][2]
    d = cloudXYZ_sort[k][0]
    e = cloudXYZ_sort[k][1]
    f = cloudXYZ_sort[k][2]
    g = cloudXYZ_smooth[0][k]
    h = cloudXYZ_smooth[1][k]
    i = cloudXYZ_smooth[2][k]

    # Original pointcloud
    x.append(a)
    y.append(b)
    z.append(c)
    print("[RAW] index:{} x:{} y:{} z:{}".format(index,x[index],y[index],z[index]))
    index += 1

    # Sorted pointcloud
    x_sort.append(d)
    y_sort.append(e)
    z_sort.append(f)
    print("[Sorted] index:{} x:{} y:{} z:{}".format(index_sort,x_sort[index_sort],y_sort[index_sort],z_sort[index_sort]))
    index_sort += 1

    # Smoothed pointcloud
    xs.append(g)
    ys.append(h)
    zs.append(i)
    print("[Smooth] index:{} x:{} y:{} z:{}\r\n".format(index_smooth,xs[index_smooth],ys[index_smooth],zs[index_smooth]))
    index_smooth += 1

    plt.cla()
    # Plotting curve
    #ax.plot3D(x, y, z, color='blue',label='planned_path')

    # Plotting scatter points
    ax.scatter(x, y, z, color='red', cmap='jet', label='original pointcloud')
    ax.scatter(xs, ys, zs, color='green', cmap='jet', label='planned path')
    ax.scatter(x_sort, y_sort, z_sort, color='blue', cmap='jet', label='sorted pointcloud')

    ax.set_title('Path Planner (m)')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Height')
    plt.legend(loc='upper left')
    plt.tight_layout()

def main():
    global cloudXYZ_sort,cloudXYZ_smooth,cloudXYZ,ax
    global origin,refvec
    # Load the pointcloud
    cloudXYZ = read_csv_cloud('save_cloud_backup.csv')

    # Sort the pointcloud with clockwise
    origin[0] = cloudXYZ[0][0]
    origin[1] = cloudXYZ[0][1]
    cloudXYZ_sort = sorted(cloudXYZ,key=clockwiseangle_and_distance)

    # Smooth the pointcloud
    i,j,k = seperate_cloud_channel(cloudXYZ_sort)
    window_size = 201
    polynormial_order = 2
    cloudXYZ_smooth = savgol_filter((i,j,k),window_size,polynormial_order)
    #cloudXYZ_smooth = Rbf(cloudXYZ[0], y, z, smooth=0.05)


    # Plotter
    ax = plt.axes(projection='3d')
    ani = FuncAnimation(plt.gcf(), animate, interval=100)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()




