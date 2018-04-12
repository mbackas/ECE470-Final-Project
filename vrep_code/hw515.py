#HW 5.1.5 HH
#################################################################################
import numpy as np


# Initial positions of the 8 spheres
Z = []
Z.append([0, 0, 0, 1]) # First sphere always at zero position
Z.append([0, 2, 0, 1])
Z.append([0, 4, -2, 1])
Z.append([0, 2, -6, 1])
Z.append([0, 0, -6, 1])
Z.append([0, -2, -6, 1])
Z.append([0, -2, -2, 1])
Z.append([0, -2, 0, 1])

#################################################################################

from scipy import linalg
import math

def distance_between_points(p_1, p_2):
    return math.sqrt((p_1[0] - p_2[0])**2 + (p_1[1] - p_2[1])**2 + (p_1[2] - p_2[2])**2)

def are_spheres_colliding(p_1, p_2, r_1, r_2):
    return distance_between_points(p_1, p_2) <= (r_1 + r_2)

def bracket_screw(screw):
    matrix_w = np.array([
        [0, -screw[2], screw[1]],
        [screw[2], 0,-screw[0]],
        [-screw[1],screw[0], 0]
    ])
    v = (screw[3:6])[:, None]
    matrix_top = np.concatenate((matrix_w, v),axis = 1)
    matrix_bottom = np.array([[0, 0, 0, 0]])
    matrix_screw = np.concatenate((matrix_top, matrix_bottom), axis = 0)
    return matrix_screw

# Get position of sphere N which is initialially at position M
def get_sphere_position(theta, S, M, N):
    e = []

    if N == 0 or N == 1:
        return M

    for i in range(1, N):
        e.append(linalg.expm(bracket_screw(S[i - 1]) * theta[i - 1]))

    e = np.array(e)
    ret = np.eye(4)
    for i in range(1, N):
        ret = ret.dot(e[i - 1])
    return ret.dot(M)

# HW 5.1.3
# Returns an array sphere_centers where sphere_centers[i] is the center of sphere i
def place_spheres(S, theta, Z):

    S = S.transpose()

    # Spheres 0, 1, ..., 7
    sphere_centers = []
    for i in range(8):
        x = get_sphere_position(theta, S, Z[i], i)
        sphere_centers.append(list(x[:3]))

    sphere_centers = np.array(sphere_centers)
    return sphere_centers

# HW 5.1.4
def is_there_collision(S, theta, Z, r):
    spheres = place_spheres(S, theta, Z)
    #print(spheres)

    collision = False
    for i in range(8):
        for j in range(8):
            if i != j and are_spheres_colliding(spheres[i], spheres[j], r, r):
                collision = True
    return collision
#############################################################################
'''
theta = theta.transpose()

c = []
for i in range(20):
    if is_there_collision(S, theta[i], Z, r):
        c += [1]
    else:
        c += [0]

c = [c]
print(c)

with open('r.txt', 'w') as rf:
    rf.write(str(c))
'''
