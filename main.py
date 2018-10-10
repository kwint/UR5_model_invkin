import pyqtgraph.opengl as gl
import numpy as np
from transformations import rotation_matrix
from pyqtgraph.Qt import QtCore, QtGui

from rm_utils import *

w = gl.GLViewWidget()
w.show()
w.setWindowTitle('UR5')

g = gl.GLGridItem()

g.translate(0, 0, -0.1) # Grid a bit lower for visablity
w.addItem(g)

# color specification in (normalised) RGBA values
joint_color = (1., 1., .4, 1)  # yellow
arm_color = (0.4, 0.4, 1, 1)  # light blue

# arm lengths:
a1 = 2
a2 = 1.5
a3 = 2

# other dimensions:  radius of the joints
# width of the links, depth (in z-dimension) of the links
radius = 0.1
width = .6 * 1 * radius
depth = width
# make cylinder slightly larger in depth
# (distance along z-axis) so be better see it
depth_cylinder = 1.2 * width

# create the vertices and faces for joint1 and link1
# that will used below to create the cylinder and box in pyqtgraph
vertices_joint1, faces_joint1 = cylinder(radius, depth_cylinder, N=40)
vertices_arm1, faces_arm1 = box((depth, width, a1))

vertices_joint2, faces_joint2 = cylinder(radius, depth_cylinder, N=40)
vertices_arm2, faces_arm2 = box((a2, width, depth))

vertices_joint3, faces_joint3 = cylinder(radius, depth_cylinder, N=40)
vertices_arm3, faces_arm3 = box((a3, width, depth))


# create coordinate axes, also called frames.
# frame0: the fixed world-frame
frame0 = gl.GLAxisItem(antialias=True, glOptions='opaque')
# we make the lines a bit thicker to stress this is the world-frame
frame0.setGLOptions({'glLineWidth': (2,)})
# frame1 is connected at joint1 and rotates relative to frame0
frame1 = gl.GLAxisItem(antialias=True, glOptions='opaque')
frame1.setParentItem(frame0)
# frame1_end is the coordinate frame fixed at the end of link1
# it is specified relative to frame1
frame1_end = gl.GLAxisItem(antialias=True, glOptions='opaque')
frame1_end.setParentItem(frame1)
frame1_end.translate(0, 0, a1)

# Do the same thing for frame 2, which is the second arm
frame2 = gl.GLAxisItem(antialias=True, glOptions='opaque')
frame2.setParentItem(frame1_end)
# frame2_end is the coordinate frame fixed at the end of link1
# it is specified relative to frame1
frame2_end = gl.GLAxisItem(antialias=True, glOptions='opaque')
frame2_end.setParentItem(frame2)
frame2_end.translate(a2, 0, 0)

# Do the same thing for frame 2, which is the second arm
frame3 = gl.GLAxisItem(antialias=True, glOptions='opaque')
frame3.setParentItem(frame2_end)
# frame2_end is the coordinate frame fixed at the end of link1
# it is specified relative to frame1
frame3_end = gl.GLAxisItem(antialias=True, glOptions='opaque')
frame3_end.setParentItem(frame3)
frame3_end.translate(a3, 0, 0)

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
joint2.rotate(90, 1,0,0)
# we specify arm1 relative to frame1, so that it will rotate with frame1
arm2.setParentItem(frame2)
# lower the arm a bit, so that the y-axis is just at the middle of the arm
arm2.translate(-width / 2, -width / 2, -width / 2)
joint2.translate(0, 0, -width/2)

# we specify joint1 relative to frame1, so that it will rotate with frame1
joint3.setParentItem(frame3)
joint3.rotate(90, 1,0,0)
# we specify arm1 relative to frame1, so that it will rotate with frame1
arm3.setParentItem(frame3)
# lower the arm a bit, so that the y-axis is just at the middle of the arm
arm3.translate(-width / 2, -width / 2, -width / 2)
joint3.translate(0, 0, -width/2)

# now all objects are added to the window so we really can see them
w.addItem(frame0)

w.addItem(frame1)
w.addItem(frame1_end)
w.addItem(joint1)
w.addItem(arm1)

w.addItem(frame2)
w.addItem(frame2_end)
w.addItem(joint2)
w.addItem(arm2)

w.addItem(frame3)
w.addItem(frame3_end)
w.addItem(joint3)
w.addItem(arm3)

trajectory = np.array([
    [3, 3, 3],
])
arms = np.array([a1, a2, a3])

# define the window updating function
def update_window(trajectory):
    # we need access to variables in the main-program, so make
    # them accessable by making them global within this function
    global w, frame1, arms
    # print(i)
    # 'inverse kinematics' of a 1-link planar robot:
    # angles1 = np.arctan2(trajectory[i, 1], trajectory[i, 0])

    angles = kin_planar_inverse([a1, a2, a3], trajectory, False)
    print(np.degrees(angles))

    R_0_1 = rotation_matrix(angles[0], [0, 0, 1])
    R_1_2 = rotation_matrix(angles[1], [0, 1, 0])
    R_2_3 = rotation_matrix(angles[2], [0, 1, 0])

    frame1.setTransform(R_0_1.flatten())
    frame2.setTransform(R_1_2.flatten())
    frame3.setTransform(R_2_3.flatten())

    # now update the OpenGL graphics in window w.
    w.updateGL()

    # next line not really necessary, but its neat to return something
    return True
