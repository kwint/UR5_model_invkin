import pyqtgraph.opengl as gl
import numpy as np
from transformation_ian import rotation_matrix
from pyqtgraph.Qt import QtCore, QtGui
import math

from rm_utils import *


np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})


w = gl.GLViewWidget()
w.show()
w.setWindowTitle('UR5')

g = gl.GLGridItem()

g.translate(0, 0, -0.1) # Grid a bit lower for visablity
w.addItem(g)

# color specification in (normalised) RGBA values
joint_color = (1., 1., .4, 1)  # yellow
arm_color = (0.4, 0.4, 1, 1)  # light blue

# D-H parameters
theta1 = 0
a1 = 0
d1 = 0.089159 * 10
alpha1 = np.pi/2

theta2 = 0
a2 = -0.425 * 10
d2 = 0
alpha2 = 0

theta3 = 0
a3 = -0.3922 * 10
d3 = 0
alpha3 = 0

# other dimensions:  radius of the joints
# width of the links, depth (in z-dimension) of the links
radius = 0.2
width = .6 * 1 * radius *2
depth = width
# make cylinder slightly larger in depth
# (distance along z-axis) so be better see it
depth_cylinder = 1.2 * width

# create the vertices and faces for joint1 and link1
# that will used below to create the cylinder and box in pyqtgraph
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
# frame1 is connected at joint1 and rotates relative to frame0
frame1 = gl.GLAxisItem(antialias=True, glOptions='opaque')
frame1.setParentItem(frame0)
# frame1_end is the coordinate frame fixed at the end of link1
# it is specified relative to frame1
# frame1_end = gl.GLAxisItem(antialias=True, glOptions='opaque')
# frame1_end.setParentItem(frame1)
# frame1_end.translate(a1, 0, 0)

# Do the same thing for frame 2, which is the second arm
frame2 = gl.GLAxisItem(antialias=True, glOptions='opaque')
frame2.setParentItem(frame1)
# frame2_end is the coordinate frame fixed at the end of link1
# it is specified relative to frame1
# frame2_end = gl.GLAxisItem(antialias=True, glOptions='opaque')
# frame2_end.setParentItem(frame2)
# frame2_end.translate(a2, 0, 0)


# Do the same thing for frame 2, which is the second arm
frame3 = gl.GLAxisItem(antialias=True, glOptions='opaque')
frame3.setParentItem(frame2)
# frame2_end is the coordinate frame fixed at the end of link1
# it is specified relative to frame1
# frame3_end = gl.GLAxisItem(antialias=True, glOptions='opaque')
# frame3_end.setParentItem(frame3)
# frame3_end.translate(a3, 0, 0)

# now create joint1 using the previously calculated vertices and faces
joint1 = gl.GLMeshItem(vertexes=vertices_joint1, faces=faces_joint1,
                       drawEdges=False, drawFaces=True, color=joint_color)

# now create arm1 using the previously calculated vertices and faces
arm1 = gl.GLMeshItem(vertexes=vertices_arm1, faces=faces_arm1,
                     drawEdges=False, drawFaces=True, color=arm_color)

# now create joint1 using the previously calculated vertices and faces
joint2 = gl.GLMeshItem(vertexes=vertices_joint2, faces=faces_joint2,
                       drawEdges=False, drawFaces=True, color=joint_color)

# now create arm1 using the previously calculated vertices and faces
arm2 = gl.GLMeshItem(vertexes=vertices_arm2, faces=faces_arm2,
                     drawEdges=False, drawFaces=True, color=arm_color)

# now create joint1 using the previously calculated vertices and faces
joint3 = gl.GLMeshItem(vertexes=vertices_joint3, faces=faces_joint3,
                       drawEdges=False, drawFaces=True, color=joint_color)

# now create arm1 using the previously calculated vertices and faces
arm3 = gl.GLMeshItem(vertexes=vertices_arm3, faces=faces_arm3,
                     drawEdges=False, drawFaces=True, color=arm_color)


# we specify joint1 relative to frame1, so that it will rotate with frame1
joint1.setParentItem(frame1)
# we specify arm1 relative to frame1, so that it will rotate with frame1
arm1.setParentItem(frame1)
# lower the arm a bit, so that the y-axis is just at the middle of the arm
arm1.translate(-width / 2, -width / 2, -width / 2)
joint1.translate(0, 0, -width/2)

# we specify joint1 relative to frame1, so that it will rotate with frame1
joint2.setParentItem(frame2)
# joint2.rotate(90, 1,0,0)
# we specify arm1 relative to frame1, so that it will rotate with frame1
arm2.setParentItem(frame2)
# lower the arm a bit, so that the y-axis is just at the middle of the arm
arm2.translate(-width / 2, -width / 2, -width / 2)
joint2.translate(0, 0, -width/2)

# we specify joint1 relative to frame1, so that it will rotate with frame1
joint3.setParentItem(frame3)
# joint3.rotate(90, 1,0,0)
# we specify arm1 relative to frame1, so that it will rotate with frame1
arm3.setParentItem(frame3)
# lower the arm a bit, so that the y-axis is just at the middle of the arm
arm3.translate(-width / 2, -width / 2, -width / 2)
joint3.translate(0, 0, -width/2)

# now all objects are added to the window so we really can see them
w.addItem(frame0)

w.addItem(frame1)
# w.addItem(frame1_end)
w.addItem(joint1)
w.addItem(arm1)

w.addItem(frame2)
# w.addItem(frame2_end)
w.addItem(joint2)
w.addItem(arm2)

w.addItem(frame3)
# w.addItem(frame3_end)
w.addItem(joint3)
w.addItem(arm3)

trajectory = np.array([
    [3, 3, 3],
])
arms = np.array([d1, a2, a3])

# define the window updating function
def update_point(trajectory):
    # we need access to variables in the main-program, so make
    # them accessable by making them global within this function
    global w, frame1, arms
    # print(i)
    # 'inverse kinematics' of a 1-link planar robot:
    # angles1 = np.arctan2(trajectory[i, 1], trajectory[i, 0])

    angles = kin_planar_inverse([d1, a2, a3], trajectory, False)
    print(np.degrees(angles))

    update_angles(angles)

    # now update the OpenGL graphics in window w.
    w.updateGL()

    # next line not really necessary, but its neat to return something
    return True

def update_angles(angles):
    # we need access to variables in the main-program, so make
    # them accessable by making them global within this function
    global w, frame1
    # print(i)
    # 'inverse kinematics' of a 1-link planar robot:
    # angles1 = np.arctan2(trajectory[i, 1], trajectory[i, 0])

    # angles = kin_planar_inverse([a1, a2, a3], trajectory, False)
    print(np.degrees(angles))

    theta1, theta2, theta3 = -angles[0], -angles[1], -angles[2]
    R_0_1x = rotation_matrix(alpha1, [1, 0, 0])
    R_0_1z = rotation_matrix(theta1, [0, 0, 1])
    R_0_1_a = np.identity(4)
    R_0_1_a[0, 3] = a1
    R_0_1_d = np.identity(4)
    R_0_1_d[2, 3] = d1
    R_0_1 = np.dot(np.dot(np.dot(R_0_1z, R_0_1_d), R_0_1_a), R_0_1x)

    R_1_2x = rotation_matrix(alpha2, [1, 0, 0])
    R_1_2z = rotation_matrix(theta2, [0, 0, 1])
    R_1_2_a = np.identity(4)
    R_1_2_a[0, 3] = a2
    R_1_2_d = np.identity(4)
    R_1_2_d[2, 3] = d2
    R_1_2 = np.dot(np.dot(np.dot(R_1_2z, R_1_2_d), R_1_2_a), R_1_2x)

    R_2_3x = rotation_matrix(alpha3, [1, 0, 0])
    R_2_3z = rotation_matrix(theta3, [0, 0, 1])
    R_2_3_a = np.identity(4)
    R_2_3_a[0, 3] = a3
    R_2_3_d = np.identity(4)
    R_2_3_d[2, 3] = d3
    R_2_3 = np.dot(np.dot(np.dot(R_2_3z, R_2_3_d), R_2_3_a), R_2_3x)

    frame1.setTransform(R_0_1.flatten())
    frame2.setTransform(R_1_2.flatten())
    frame3.setTransform(R_2_3.flatten())

    # print(R_0_1)
    # print(R_1_2)
    # print(R_2_3)
    # print(np.dot(np.dot(R_0_1, R_1_2), R_2_3))
    # now update the OpenGL graphics in window w.
    w.updateGL()

    # next line not really necessary, but its neat to return something
    return True

def update_window():
    global w, i, tr, N
    update_point(tr[i])
    i = i + 1
    if i == N: i = 0


def PointsInCircum(r,n=100):
    return [[math.cos(2*np.pi/n*x)*r,math.sin(2*np.pi/n*x)*r, 0] for x in range(0,n+1)]


N = 200
tr = PointsInCircum(3, N)
h1 = np.linspace(0, 3, int(N/2))
h2 = np.linspace(3, 0, int(N/2))
h = np.append(h1, h2)
for ii in range(0, N):
    tr[ii][2] = h[ii]


i = 0
timer = QtCore.QTimer()
timer.timeout.connect(update_window)
timer.start(100)


