# utility functions for the course Robot Modelling
# Rufus Fraanje (p.r.fraanje@hhs.nl), sept. 2016

###############################################################################
def kin_planar_forward(arms, angles):
    """Forward kinematics of a 2-link planar robot.
    
    Inputs:
        arms: 2-element array/list with arm lengths
        angles: 2-element array/list with angles in radians(!)

    Output:
        point2: 2-element numpy array with (x,y) coordinate of end point

    """
    import numpy as np
    x1 = arms[0] * np.cos(angles[0])
    y1 = arms[0] * np.sin(angles[0])
    # adjust this
    x2 = x1 + 0
    y2 = y1 + 0
    # point1 = np.array([x1,y1])
    point2 = np.array([x2, y2])
    return point2


def kin_planar_inverse(arms, point, elbow_down=True):
    """Inverse kinematics of a 2-link planar robot.
    
    Inputs:
        arms: 2-element array/list with arm lengths
        point2: 2-element array with (x,y) coordinate of end point
        elbow_down (optional): True/False boolean to determine 
                           which solution needs to be returned

    Output:
        angles: 2-element array/list with angles in radians(!)

    """
    import numpy as np
    x, y, z = point[0], point[1], point[2]
    d1, d2, d3 = arms[0], arms[1], arms[2]

    theta = [0, 0, 0]
    theta[0] = np.arctan2(y, x)

    r = (x**2 + y**2)**0.5
    s = z - d1
    # c = (r**2 + s**2)*1*0.5
    # D = (r**2 + s**2 - d2**2 - d3**2) / (2*d2*d3)
    #
    # # theta[1] = np.arctan2(s, r) - np.arctan2(d3*s, d2+d3*s)
    # # theta[2] = np.arctan2(-(1-D**2)**0.5, D)
    #
    # theta[1] = np.arctan2(s, r) - np.arccos((-d3**2+d2**2+c**2)/(-2*d3*d2))
    # theta[2] = np.pi + np.arccos((-c**2+d3**2+d2**2)/(-2*d3*d2))

    D = (r ** 2 + s ** 2 - d2 ** 2 - d3 ** 2) / (2 * d2 * d3)
    theta[2] = -np.arccos(D)
    theta[1] = np.arctan2(s, r) - np.arctan((d3 * np.sin(theta[2]) / (d2+d3*np.cos(theta[2]))))
    return theta


# cylinder is a convenience function to create a cylinder shape in
# pyqtgraph/OpenGL, it gives you a number of vertices distributed over the
# surface of the cylinder and triangular shaped faces that cover the whole
# surface of the cylinder
# cylinders are being used to visualize joints
def cylinder(radius, height, N):
    """Calculates vertices and faces for a cylinder for visualisation in
    pyqtgraph/OpenGL.

    Inputs:
        radius: radius of the cylinder
        height: height of the cylinder
        N: number of segments to approximate the circular shape of the cylinder 

    Outputs:
        vertices: array with on each row the (x,y,z) coordinates of the vertices 
        faces: array with triangular faces of the cylinder

    Note:
        The cylinder is a circle in the x,y plane with center at (0,0) that is
        extruded along the z-axis.

    """
    import numpy as np
    import scipy.spatial
    t = np.linspace(0, 2 * np.pi, N, endpoint=False).reshape(N, 1)
    vertices = np.zeros((2 * N, 3))
    vertices[0:N, :] = np.hstack((radius * np.cos(t), radius * np.sin(t), np.zeros((N, 1))))
    vertices[N:2 * N, :] = vertices[0:N, :] + np.hstack((np.zeros((N, 2)), height * np.ones((N, 1))))
    faces = np.zeros((N - 2 + 2 * N + N - 2, 3), dtype=np.uint)
    # bottom, makes use of Delaunay triangulation contained in Scipy's
    # submodule spatial (which on its turn makes use of the Qhull library)
    faces[0:N - 2, :] = scipy.spatial.Delaunay(vertices[0:N, 0:2], furthest_site=True, qhull_options='QJ').simplices[:,
                        -1::-1]
    # sides
    for i in range(N - 1):
        faces[N - 2 + 2 * i, :] = np.array([i, i + 1, N + i + 1], dtype=np.uint)
        faces[N - 2 + 2 * i + 1, :] = np.array([i, N + i + 1, N + i], dtype=np.uint)
    # final one between the last and the first:
    faces[N - 2 + 2 * (N - 1), :] = np.array([N - 1, 0, N], dtype=np.uint)
    faces[N - 2 + 2 * (N - 1) + 1, :] = np.array([N - 1, N, 2 * N - 1], dtype=np.uint)
    # top
    faces[N - 2 + 2 * N:N - 2 + 2 * N + N - 2, :] = N + faces[0:N - 2, -1::-1]

    return vertices, faces


# simular to the cylinder, but not for creating a box-shaped object
# boxes are used to visualize links
def box(size=(1, 1, 1)):
    """Calculates vertices and faces for a box for visualisation in
    pyqtgraph/OpenGL.

    Inputs:
        size: 3 element array/list with the width,depth,height, i.e. 
              the dimensions along the x, y and z-axis.

    Outputs:
        vertices: array with on each row the (x,y,z) coordinates of the vertices 
        faces: array with triangular faces of the box 

    Note:
        The box is between (0,0,0) and (size[0],size[1],size[2]), note that
        negative sizes are not prevented but result in strange artifacts because
        it changes the orientation of the faces of the box (inside becomes
        outside).

    """
    import numpy as np
    vertices = np.zeros((8, 3))
    faces = np.zeros((12, 3), dtype=np.uint)
    xdim = size[0]
    ydim = size[1]
    zdim = size[2]
    vertices[0, :] = np.array([0, ydim, 0])
    vertices[1, :] = np.array([xdim, ydim, 0])
    vertices[2, :] = np.array([xdim, 0, 0])
    vertices[3, :] = np.array([0, 0, 0])
    vertices[4, :] = np.array([0, ydim, zdim])
    vertices[5, :] = np.array([xdim, ydim, zdim])
    vertices[6, :] = np.array([xdim, 0, zdim])
    vertices[7, :] = np.array([0, 0, zdim])

    faces = np.array([
        # bottom (clockwise, while looking from top)
        [2, 1, 0],
        [3, 2, 0],
        # sides (counter-clock-wise)
        [0, 1, 5],
        [0, 5, 4],
        [1, 2, 6],
        [1, 6, 5],
        [2, 3, 7],
        [2, 7, 6],
        [3, 0, 4],
        [3, 4, 7],
        # top (counter-clockwise)
        [4, 5, 6],
        [4, 6, 7]
    ], dtype=np.uint)

    return vertices, faces


def circle_fit(points):
    """Calculate the circle through three points.

       Input:
            points:  3x3 ndarray, with on each row the x/y/z coordinates of the
                     points.
       Output:
            radius:  radius of the circle, if all points are on a line radius is np.inf
            pcenter: center coordinate of the circle if it exists, else None
            vperp:   unit-vector perpendicular to the circle 
            angles:  array of angles between the point 1 and point 0 and 
                     between point 2 and point 0

    """
    import numpy as np
    plane = np.vstack((points[1:2, :] - points[0:1, :], points[2:3, :] - points[0:1, :])).T
    q, r = np.linalg.qr(plane, mode='complete')
    if np.allclose(r[1, 1], 0):  # all points are on a line
        radius = np.inf
        pcenter = None
        vperp = None
        angles = None
    else:
        # make diagonal of r to be positive
        signs_r = np.diag(np.diag(np.sign(r)))
        q[:, 0:2] = np.dot(q[:, 0:2], signs_r)
        if np.linalg.det(q) < 0: q[:, 2] *= -1  # = -q[:,2]
        r[0:2, :] = np.dot(signs_r, r[0:2, :])

        vperp = q[:, 2]
        alpha = .5 * r[0, 0]

        if np.allclose(r[0, 0], r[0, 1]):  # four points make a straight angle
            beta = .5 * r[1, 1]
        else:
            beta = (r[0, 1] * (r[0, 1] - r[0, 0]) + r[1, 1] ** 2) / (2 * r[1, 1])

        pcenter = points[0, :] + alpha * q[:, 0] + beta * q[:, 1]
        radius = np.sqrt(alpha ** 2 + beta ** 2)
        angles = np.array([2 * np.arctan2(beta, alpha),
                           2 * np.arcsin(.5 * np.sqrt((r[0, 1] - r[0, 0]) ** 2 + r[1, 1] ** 2) / radius)])
    return radius, pcenter, vperp, angles


def rotation_between_two_vectors(a, b):
    """Return the rotation matrix (3x3 ndarray) to rotate a in the direction of b.
        
        Inputs:
           a,b: two 3 element arrays

        Output:
            R:  3x3 rotation matrix, such that b/|b| = R a/|a|

    """
    import numpy as np

    # algorithm obtained from:
    # http://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
    an = b / np.linalg.norm(a)
    bn = a / np.linalg.norm(b)
    # rotation axis
    v = np.cross(bn, an)
    # sine of angle
    s = np.linalg.norm(v)
    # cosine of angle
    c = np.inner(bn, an)
    Vskew = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.eye(3) + Vskew + np.dot(Vskew, Vskew) * (1 - c) / s ** 2
    return R


def extrude(polygon, curve, twist=None, calc_faces=False):
    """Extrude a polygon along a curve with an optional twist.

        Inputs:
            polygon: num_p x 2 ndarray with a convex(*) 2D-polygon with num_p points
            curve:   N x 3 ndarray with the curve along which the polygon should be 
                     extruded, note that subsequent points should not bend "too much" 
                     to prevent the vertices moving "backward" such that a face 
                     shrinks to a "negative" surface
            twist (optional): scalar or ndarray of length N, 
                     scalar: specifies the amount of twist (in radians) between the 
                             first and the last polygon 
                     ndarray of length N: element i specifies the amount of
                     twist (in radians) between the first and the i-th polygon

        Outputs:
            vertices: (num_p*N) x 3 ndarray with the vertices of the extruded
                      polygon
            faces (optional): 2*(num_p-2) + 2*num_p*N  x 3  integer ndarray 
                              with triangular faces of the extruded polygon,
                              including bottom and top faces. Only determined
                              and returned when calc_faces = True (not default).
                              Note, that the face determination is very
                              simplistic and assumes that the extrusion only 
                              moves all vertices "further", so no shrinking!
                              So, no reordering of vertices.

        Notes:
        (*) Convex restriction because for triangulation the Delaunay algorithm
            from scipy.spatial is used, i.e. the Qhull Delaunay triangulation.
            If you would like to extend the algorithm for non-convex polygons,
            use for example Polygon3 or the triangle module, a wrapper to 
            Jonathan Richard Shewchuk’s mesh generation library.
    
    """
    import numpy as np
    import scipy.spatial
    from transformations import rotation_matrix

    # N is number of points in the (3D) curve
    N = curve.shape[0]
    if not (twist is None):
        # check if twist is a scalar or an array
        if not (hasattr(twist, "__len__")):
            twist = twist / (N - 1) * np.ones(N)
            twist[0] = 0.
        else:
            twist = np.hstack((0, np.diff(twist)))

    # num_p is the number of vertices in the (2D) polygon
    num_p = polygon.shape[0]
    # determine the direction of the first segment of the curve
    difference = curve[1, :] - curve[0, :]
    # the rotation matrix is used to rotate the polygon such that it 
    # will be perpedicular to the curve  (changed to [0,0,-1.], to get shading
    # right, has to check this!)
    R = rotation_between_two_vectors(np.array([0, 0, -1.]), difference)
    vertices = np.zeros((N * num_p, 3))
    vertices[0:num_p, 0:2] = polygon
    # rotate and shift the polygon
    vertices[0:num_p, :] = np.dot(vertices[0:num_p, :], R.T) + np.dot(np.ones((num_p, 1)), curve[0:1, :])
    # second step is just shifting in the direction of the curve (i.e. 'difference')
    vertices[num_p:2 * num_p, :] = vertices[0:num_p, :] + np.dot(np.ones((num_p, 1)), difference.reshape(1, 3))
    # apply twist rotation if needed:
    if not (twist is None):
        # take care rotation_matrix gives a homogeneous transformation
        Rtwist = rotation_matrix(twist[1], difference, curve[1, :])
        vertices[num_p:2 * num_p, :] = np.dot(np.hstack((vertices[num_p:2 * num_p, :], np.ones((num_p, 1)))), Rtwist.T)[
                                       :, 0:3]
    # determine the vertices by extruding along the curve
    for i in range(2, N):
        # determine curvature of the curve by fitting a circle
        radius, pcenter, vperp, angles = circle_fit(curve[i - 2:i + 1, :])
        # determine direction
        difference = curve[i, :] - curve[i - 1, :]
        if np.allclose(radius, np.inf):  # 3 points on 1 line
            vertices[i * num_p:(i + 1) * num_p, :] = vertices[(i - 1) * num_p:i * num_p, :] + np.dot(
                np.ones((num_p, 1)), difference.reshape(1, 3))
        else:
            # take care rotation_matrix gives a homogeneous transformation
            R = rotation_matrix(angles[1], vperp, pcenter)
            vertices[i * num_p:(i + 1) * num_p, :] = np.dot(
                np.hstack((vertices[(i - 1) * num_p:i * num_p, :], np.ones((num_p, 1)))), R.T)[:, 0:3]
        # determine and apply twist if necessary
        if not (twist is None):
            # take care rotation_matrix gives a homogeneous transformation
            Rtwist = rotation_matrix(twist[i], difference, curve[i, :])
            vertices[i * num_p:(i + 1) * num_p, :] = np.dot(
                np.hstack((vertices[i * num_p:(i + 1) * num_p, :], np.ones((num_p, 1)))), Rtwist.T)[:, 0:3]

    # if selected, determine the indices of the faces (looking at a conter clock-wise
    # oriented face means that you are looking at a front-side, so looking from
    # the outside all faces are specified in counter clock-wise orientation) 
    if calc_faces:
        # initialize faces (botom, sides and top):
        faces = np.zeros(((num_p - 2) + 2 * num_p * N + (num_p - 2), 3), dtype=np.uint)
        # bottom (walk clockwise through points, because bottom is outside)
        # faces[0:num_p-2,:] = scipy.spatial.Delaunay(polygon[-1::-1,:],furthest_site=True,
        faces[0:num_p - 2, :] = scipy.spatial.Delaunay(polygon, furthest_site=True,
                                                       qhull_options='QJ').simplices[:, -1::-1]

        # sides
        for i in range(N - 1):  # loop over all levels
            for j in range(num_p - 1):  # loop over all vertices at a level
                faces[num_p - 2 + i * 2 * num_p + 2 * j, :] = np.array(
                    [i * num_p + j, i * num_p + j + 1, (i + 1) * num_p + j + 1], dtype=np.uint)
                faces[num_p - 2 + i * 2 * num_p + 2 * j + 1, :] = np.array(
                    [i * num_p + j, (i + 1) * num_p + j + 1, (i + 1) * num_p + j], dtype=np.uint)
            # close the ring between the last and the first vertices at a level 
            faces[num_p - 2 + i * 2 * num_p + 2 * (num_p - 1), :] = np.array(
                [i * num_p + num_p - 1, i * num_p, (i + 1) * num_p], dtype=np.uint)
            faces[num_p - 2 + i * 2 * num_p + 2 * (num_p - 1) + 1, :] = np.array(
                [i * num_p + num_p - 1, (i + 1) * num_p, (i + 1) * num_p + num_p - 1], dtype=np.uint)

        # top
        # just reverse the ones from the bottom but at (N-1)*num_p higher vertex numbers
        faces[-num_p + 2::, :] = (N - 1) * num_p + faces[0:num_p - 2, -1::-1]

        return vertices, faces
    else:
        return vertices


# to do: write documentation
def sigmoid_interp(start, stop, alpha, N):
    import numpy as np
    return start + (stop - start) / (1 + np.exp(-alpha * np.linspace(-1, 1, N)))


def rounded_rectangle_polygon(xdim, ydim, N, percentage=0):
    import numpy as np
    polygon = np.zeros((N, 2))
    radius = .5 * min(xdim, ydim) * percentage
    # circumference = 2*(xdim + ydim) + (2*np.pi-8)*radius

    t = np.linspace(0, 2 * np.pi, N).reshape(N, 1)
    N1 = np.int(N / 4)
    N2 = np.int(N / 2)
    N3 = np.int(3 * N / 4)
    N4 = N
    part1 = np.hstack(((xdim / 2 - radius) + radius * np.cos(t[0:N1]), (ydim / 2 - radius) + radius * np.sin(t[0:N1])))
    part2 = np.hstack(
        ((-xdim / 2 + radius) + radius * np.cos(t[N1:N2]), (ydim / 2 - radius) + radius * np.sin(t[N1:N2])))
    part3 = np.hstack(
        ((-xdim / 2 + radius) + radius * np.cos(t[N2:N3]), (-ydim / 2 + radius) + radius * np.sin(t[N2:N3])))
    part4 = np.hstack(
        ((xdim / 2 - radius) + radius * np.cos(t[N3:N4]), (-ydim / 2 + radius) + radius * np.sin(t[N3:N4])))

    polygon = np.vstack((part1, part2, part3, part4))

    return polygon


def link(alpha=0, a=1, theta=0, d=0, joint_a_radius=.7, joint_a_height=.5, joint_b_radius=.7, joint_b_height=.5,
         link_width=.5, link_height=0.1, Na=40, Nb=40, Nlink=30, Nround=16, calc_faces=False):
    import numpy as np
    from transformations import rotation_matrix

    vertices_joint_a, faces_joint_a = cylinder(joint_a_radius, joint_a_height, Na)
    vertices_joint_b, faces_joint_b = cylinder(joint_b_radius, joint_b_height, Nb)
    vertices_joint_b[:, 2] -= joint_b_height
    # transform points of joint_b to end of link
    Htrans_z = np.eye(4)
    Htrans_z[2, 3] = d
    Htrans_x = np.eye(4)
    Htrans_x[0, 3] = a
    Hrot_z = rotation_matrix(theta, [0, 0, 1])
    Hrot_x = rotation_matrix(alpha, [1, 0, 0])
    H = np.dot(Hrot_z, np.dot(Htrans_z, np.dot(Htrans_x, Hrot_x)))
    vertices_joint_b = np.dot(np.hstack((vertices_joint_b, np.ones((vertices_joint_b.shape[0], 1)))), H.T)[:, 0:3]

    curve_link = np.vstack(
        (np.array([[0, 0, .5 * joint_a_height], [.5 * joint_a_radius, 0, 0.5 * joint_a_height]]),
         np.hstack((np.linspace(joint_a_radius, a, Nlink).reshape(Nlink, 1), np.zeros((Nlink, 1)),
                    sigmoid_interp(.5 * joint_a_height, d - .5 * joint_b_height, 7, Nlink).reshape(Nlink, 1)))))

    polygon_link = rounded_rectangle_polygon(link_height, link_width, Nround, percentage=0.3)

    if calc_faces:
        vertices_link, faces_link = extrude(polygon_link, curve_link, alpha, calc_faces=True)
        if not (theta is 0):
            vertices_link = np.dot(np.hstack((vertices_link, np.ones((vertices_link.shape[0], 1)))), Hrot_z.T)[:, 0:3]
        return vertices_link, faces_link, vertices_joint_a, faces_joint_a, vertices_joint_b, faces_joint_b, H
    else:
        if not (theta is 0):
            vertices_link = np.dot(np.hstack((vertices_link, np.ones((vertices_link.shape[0], 1)))), Hrot_z.T)[:, 0:3]
        vertices_link = extrude(polygon_link, curve_link, alpha, calc_faces=False)
        return vertices_link, vertices_joint_a, vertices_joint_b, H
