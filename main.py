import pyqtgraph.opengl as gl
import numpy as np
from transformation_fix_y import rotation_matrix
from pyqtgraph.Qt import QtCore, QtGui
import math

from rm_utils import *

# Format array printing for easier debugging
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

# Create window
w = gl.GLViewWidget()
w.show()
w.setWindowTitle('UR5')

# Create grid
g = gl.GLGridItem()
g.translate(0, 0, -0.1)  # Grid a bit lower for visablity
w.addItem(g)

# color specification in (normalised) RGBA values
joint_color = (1., 1., .4, 1)  # yellow
arm_color = (0.4, 0.4, 1, 1)  # light blue

# D-H parameters
theta1 = 0
a1 = 0
d1 = 0.089159 * 10
alpha1 = np.pi / 2

theta2 = 0
a2 = -0.425 * 10
d2 = 0
alpha2 = 0

theta3 = 0
a3 = -0.3922 * 10
d3 = 0
alpha3 = 0

# Make joints and arms vertices
radius = 0.2
width = .6 * 1 * radius * 2
depth = width
depth_cylinder = 1.2 * width
vertices_joint1, faces_joint1 = cylinder(radius, depth_cylinder, N=40)
vertices_arm1, faces_arm1 = box((width, -d1, depth))

vertices_joint2, faces_joint2 = cylinder(radius, depth_cylinder, N=40)
vertices_arm2, faces_arm2 = box((-a2, width, depth))

vertices_joint3, faces_joint3 = cylinder(radius, depth_cylinder, N=40)
vertices_arm3, faces_arm3 = box((-a3, width, depth))

# create coordinate axes, also called frames.
# frame0: the fixed world-frame
frame0 = gl.GLAxisItem(antialias=True, glOptions='opaque')
# we make the lines a bit thicker to stress this is the world-frame
frame0.setGLOptions({'glLineWidth': (4,)})

# Create frames for each link
frame1 = gl.GLAxisItem(antialias=True, glOptions='opaque')
frame1.setParentItem(frame0)
frame2 = gl.GLAxisItem(antialias=True, glOptions='opaque')
frame2.setParentItem(frame1)
frame3 = gl.GLAxisItem(antialias=True, glOptions='opaque')
frame3.setParentItem(frame2)

# Make the joints and arms itself
joint1 = gl.GLMeshItem(vertexes=vertices_joint1, faces=faces_joint1,
                       drawEdges=False, drawFaces=True, color=joint_color)
arm1 = gl.GLMeshItem(vertexes=vertices_arm1, faces=faces_arm1,
                     drawEdges=False, drawFaces=True, color=arm_color)
joint2 = gl.GLMeshItem(vertexes=vertices_joint2, faces=faces_joint2,
                       drawEdges=False, drawFaces=True, color=joint_color)
arm2 = gl.GLMeshItem(vertexes=vertices_arm2, faces=faces_arm2,
                     drawEdges=False, drawFaces=True, color=arm_color)
joint3 = gl.GLMeshItem(vertexes=vertices_joint3, faces=faces_joint3,
                       drawEdges=False, drawFaces=True, color=joint_color)
arm3 = gl.GLMeshItem(vertexes=vertices_arm3, faces=faces_arm3,
                     drawEdges=False, drawFaces=True, color=arm_color)

# Add joints and arms to window. Translate them a bit so they are centered
joint1.setParentItem(frame1)
arm1.setParentItem(frame1)
arm1.translate(-width / 2, -width / 2, -width / 2)
joint1.translate(0, 0, -width / 2)
joint2.setParentItem(frame2)
arm2.setParentItem(frame2)
arm2.translate(-width / 2, -width / 2, -width / 2)
joint2.translate(0, 0, -width / 2)
joint3.setParentItem(frame3)
arm3.setParentItem(frame3)
arm3.translate(-width / 2, -width / 2, -width / 2)
joint3.translate(0, 0, -width / 2)

# now all objects are added to the window so we really can see them
w.addItem(frame0)
w.addItem(frame1)
w.addItem(joint1)
w.addItem(arm1)
w.addItem(frame2)
w.addItem(joint2)
w.addItem(arm2)
w.addItem(frame3)
w.addItem(joint3)
w.addItem(arm3)


# update robot so its end effector is at vector
def update_vector(vector):
    global w, frame1
    angles = kin_planar_inverse([d1, a2, a3], vector, False)
    print(np.degrees(angles))

    update_angles(angles)

    # now update the OpenGL graphics in window w.
    w.updateGL()

    return True


# Update the angle of each link [theta1, theta2, theta3]
def update_angles(angles):
    global w, frame1

    print(np.degrees(angles))

    # Unpack angles
    theta1, theta2, theta3 = -angles[0], -angles[1], -angles[2]

    # Make rotation matrixes for D-H notation
    R_0_1_alpha = rotation_matrix(alpha1, [1, 0, 0])
    R_0_1_theta = rotation_matrix(theta1, [0, 0, 1])
    R_0_1_a = np.identity(4)
    R_0_1_a[0, 3] = a1
    R_0_1_d = np.identity(4)
    R_0_1_d[2, 3] = d1

    # Calculate Homogeneus Transfer Matrix in correct order
    R_0_1 = np.dot(np.dot(np.dot(R_0_1_theta, R_0_1_d), R_0_1_a), R_0_1_alpha)

    # Do this for al D-H parameters
    R_1_2_alpha = rotation_matrix(alpha2, [1, 0, 0])
    R_1_2_theta = rotation_matrix(theta2, [0, 0, 1])
    R_1_2_a = np.identity(4)
    R_1_2_a[0, 3] = a2
    R_1_2_d = np.identity(4)
    R_1_2_d[2, 3] = d2
    R_1_2 = np.dot(np.dot(np.dot(R_1_2_theta, R_1_2_d), R_1_2_a), R_1_2_alpha)

    R_2_3_alpha = rotation_matrix(alpha3, [1, 0, 0])
    R_2_3_theta = rotation_matrix(theta3, [0, 0, 1])
    R_2_3_a = np.identity(4)
    R_2_3_a[0, 3] = a3
    R_2_3_d = np.identity(4)
    R_2_3_d[2, 3] = d3
    R_2_3 = np.dot(np.dot(np.dot(R_2_3_theta, R_2_3_d), R_2_3_a), R_2_3_alpha)

    # Apply trasformations
    frame1.setTransform(R_0_1.flatten())
    frame2.setTransform(R_1_2.flatten())
    frame3.setTransform(R_2_3.flatten())

    w.updateGL()

    return True


# Function used for antimation
def update_window():
    global w, i, tr, N
    update_vector(tr[i])
    i = i + 1
    if i == N: i = 0


# Returns array with points of a circumforence of a circle
def PointsInCircum(r, n=100):
    return [[math.cos(2 * np.pi / n * x) * r, math.sin(2 * np.pi / n * x) * r, 0] for x in range(0, n + 1)]


# Make animation points
N = 200
i = 0
tr = PointsInCircum(3, N)
h1 = np.linspace(0, 3, int(N / 2))
h2 = np.linspace(3, 0, int(N / 2))
h = np.append(h1, h2)

for ii in range(0, N):
    tr[ii][2] = h[ii]

# Start Animation Timer
timer = QtCore.QTimer()
timer.timeout.connect(update_window)
timer.start(100)
