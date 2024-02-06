import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math

def create_cube_vertices_and_faces(center, edge_length):
    # Calculate half the edge length for convenience
    half_edge = edge_length / 2

    # Calculate the coordinates of the cube vertices
    vertices = [
        (center[0] - half_edge, center[1] - half_edge, center[2] - half_edge),  # Vertex 0
        (center[0] + half_edge, center[1] - half_edge, center[2] - half_edge),  # Vertex 1
        (center[0] + half_edge, center[1] + half_edge, center[2] - half_edge),  # Vertex 2
        (center[0] - half_edge, center[1] + half_edge, center[2] - half_edge),  # Vertex 3
        (center[0] - half_edge, center[1] - half_edge, center[2] + half_edge),  # Vertex 4
        (center[0] + half_edge, center[1] - half_edge, center[2] + half_edge),  # Vertex 5
        (center[0] + half_edge, center[1] + half_edge, center[2] + half_edge),  # Vertex 6
        (center[0] - half_edge, center[1] + half_edge, center[2] + half_edge)   # Vertex 7
    ]

    # Define the cube faces using the vertices
    faces = [
        [0, 1, 5, 4],
        [1, 2, 6, 5],
        [2, 3, 7, 6],
        [3, 0, 4, 7],
        [0, 3, 2, 1],
        [4, 5, 6, 7]
    ]

    return vertices, faces
def d_eucl(x_temp, y_temp, z_temp, x2_temp, y2_temp, z2_temp):
    distance_temp1 = (x_temp - x2_temp) * (x_temp - x2_temp)
    distance_temp2 = (y_temp - y2_temp) * (y_temp - y2_temp)
    distance_temp = distance_temp1 + distance_temp2 + (z_temp - z2_temp) * (z_temp - z2_temp)
    distance_temp = math.sqrt(distance_temp)
    return (distance_temp)

def check_obstacle(obstacles_coordinates, x_temp, y_temp, z_temp):
    # print("długośc tablicy :", len(obstacles_coordinates))
    omit = 0
    for i in range(len(obstacles_coordinates)):
        # print(d_eucl(x_temp, y_temp, obstacles_coordinates[i][0], obstacles_coordinates[i][1]))
        if (d_eucl(x_temp, y_temp, z_temp, obstacles_coordinates[i][0], obstacles_coordinates[i][1], obstacles_coordinates[i][2]) < 30):
            omit = 1
    return (omit)


# Define the vertices and faces for the outer cube
outer_vertices = [
    [0, 0, 0],
    [150, 0, 0],
    [150, 150, 0],
    [0, 150, 0],
    [0, 0, 150],
    [150, 0, 150],
    [150, 150, 150],
    [0, 150, 150]
]

outer_faces = [
    [0, 1, 5, 4],
    [1, 2, 6, 5],
    [2, 3, 7, 6],
    [3, 0, 4, 7],
    [0, 3, 2, 1],
    [4, 5, 6, 7]
]

# point_coordinates = (119, 128, 111)
# point_coordinates = (70,70,70)
# point_coordinates2 = (30,30,50)
point_coordinates = (100,120,100)
point_coordinates2 = (30,20,20)
obstacles_coordinates_new = ([30, 30, 50], [80,80,50], [80,100,100], [50,60,60])

# Cube parameters
cube_edge_length = 10

# Calculate the coordinates of the cube vertices
inner_vertices = [
    (point_coordinates[0] - cube_edge_length / 2, point_coordinates[1] - cube_edge_length / 2, point_coordinates[2] - cube_edge_length / 2),  # Vertex 0
    (point_coordinates[0] + cube_edge_length / 2, point_coordinates[1] - cube_edge_length / 2, point_coordinates[2] - cube_edge_length / 2),  # Vertex 1
    (point_coordinates[0] + cube_edge_length / 2, point_coordinates[1] + cube_edge_length / 2, point_coordinates[2] - cube_edge_length / 2),  # Vertex 2
    (point_coordinates[0] - cube_edge_length / 2, point_coordinates[1] + cube_edge_length / 2, point_coordinates[2] - cube_edge_length / 2),  # Vertex 3
    (point_coordinates[0] - cube_edge_length / 2, point_coordinates[1] - cube_edge_length / 2, point_coordinates[2] + cube_edge_length / 2),  # Vertex 4
    (point_coordinates[0] + cube_edge_length / 2, point_coordinates[1] - cube_edge_length / 2, point_coordinates[2] + cube_edge_length / 2),  # Vertex 5
    (point_coordinates[0] + cube_edge_length / 2, point_coordinates[1] + cube_edge_length / 2, point_coordinates[2] + cube_edge_length / 2),  # Vertex 6
    (point_coordinates[0] - cube_edge_length / 2, point_coordinates[1] + cube_edge_length / 2, point_coordinates[2] + cube_edge_length / 2)   # Vertex 7
]
inner_faces = [
    [0, 1, 5, 4],
    [1, 2, 6, 5],
    [2, 3, 7, 6],
    [3, 0, 4, 7],
    [0, 3, 2, 1],
    [4, 5, 6, 7]
]


clockwise = True
anticlockwise = False
dist = 0  # current distance
f = []  # list of potential fields
q = []  # empty queue

previous_neighbours_list = []
# q.append((119, 128, 111))  # adding initial values to the queue (goal coordinates and 0 as a current distance)
q.append(point_coordinates)


while len(q) > 0:
    x_restriction = 140
    x_restriction_left = 10
    y_restriction = 140
    y_restriction_bottom = 10
    z_restriction_front = 10
    z_restriction_back = 140

    previous_neighbours_list.clear()
    intersection1 = [item1 for item1 in f for item2 in q if item1 == item2]

    intersection2 = [item1 for item1 in obstacles_coordinates_new for item2 in q if item1 == item2]

    if len(intersection1) == 1 or len(intersection2) == 1:
        q.pop(0)
        dist = dist + 0.5
        intersection1.clear()
        intersection2.clear()
        continue

    else:
        if clockwise == True:
            #print(clockwise)
            # define neighbours depending of a current direction for clockwise and anticlockwise
            p0_x = q[0][0] - dist
            p0_y = q[0][1]
            p0_z = q[0][2]

            p1_x = q[0][0] - dist
            p1_y = q[0][1] + dist
            p1_z = q[0][2]

            p2_x = q[0][0]
            p2_y = q[0][1] + dist
            p2_z = q[0][2]

            p3_x = q[0][0] + dist
            p3_y = q[0][1] + dist
            p3_z = q[0][2]

            p4_x = q[0][0] + dist
            p4_y = q[0][1]
            p4_z = q[0][2]

            p5_x = q[0][0] + dist
            p5_y = q[0][1] - dist
            p5_z = q[0][2]

            p6_x = q[0][0]
            p6_y = q[0][1] - dist
            p6_z = q[0][2]

            p7_x = q[0][0] - dist
            p7_y = q[0][1] - dist
            p7_z = q[0][2]
            #********************************************
            p8_x = q[0][0] - dist
            p8_y = q[0][1]
            p8_z = q[0][2] - dist

            p9_x = q[0][0] - dist
            p9_y = q[0][1] + dist
            p9_z = q[0][2] - dist

            p10_x = q[0][0]
            p10_y = q[0][1] + dist
            p10_z = q[0][2] - dist

            p11_x = q[0][0] + dist
            p11_y = q[0][1] + dist
            p11_z = q[0][2] - dist

            p12_x = q[0][0] + dist
            p12_y = q[0][1]
            p12_z = q[0][2] - dist

            p13_x = q[0][0] + dist
            p13_y = q[0][1] - dist
            p13_z = q[0][2] - dist

            p14_x = q[0][0]
            p14_y = q[0][1] - dist
            p14_z = q[0][2] - dist

            p15_x = q[0][0] - dist
            p15_y = q[0][1] - dist
            p15_z = q[0][2] - dist
            #****************************************
            p16_x = q[0][0] - dist
            p16_y = q[0][1]
            p16_z = q[0][2] + dist

            p17_x = q[0][0] - dist
            p17_y = q[0][1] + dist
            p17_z = q[0][2] + dist

            p18_x = q[0][0]
            p18_y = q[0][1] + dist
            p18_z = q[0][2] + dist

            p19_x = q[0][0] + dist
            p19_y = q[0][1] + dist
            p19_z = q[0][2] + dist

            p20_x = q[0][0] + dist
            p20_y = q[0][1]
            p20_z = q[0][2] + dist

            p21_x = q[0][0] + dist
            p21_y = q[0][1] - dist
            p21_z = q[0][2] + dist

            p22_x = q[0][0]
            p22_y = q[0][1] - dist
            p22_z = q[0][2] + dist

            p23_x = q[0][0] - dist
            p23_y = q[0][1] - dist
            p23_z = q[0][2] + dist

            if len(q) == 1:
                q.append((p0_x, p0_y, p0_z, dist))
                q.append((p1_x, p1_y, p1_z, dist))
                q.append((p2_x, p2_y, p2_z, dist))
                q.append((p3_x, p3_y, p3_z, dist))
                q.append((p4_x, p4_y, p4_z, dist))
                q.append((p5_x, p5_y, p5_z, dist))
                q.append((p6_x, p6_y, p6_z, dist))
                q.append((p7_x, p7_y, p7_z, dist))
                q.append((p8_x, p8_y, p8_z, dist))
                q.append((p9_x, p9_y, p9_z, dist))
                q.append((p10_x, p10_y, p10_z, dist))
                q.append((p11_x, p11_y, p11_z, dist))
                q.append((p12_x, p12_y, p12_z, dist))
                q.append((p13_x, p13_y, p13_z, dist))
                q.append((p14_x, p14_y, p14_z, dist))
                q.append((p15_x, p15_y, p15_z, dist))
                q.append((p16_x, p16_y, p16_z, dist))
                q.append((p17_x, p17_y, p17_z, dist))
                q.append((p18_x, p18_y, p18_z, dist))
                q.append((p19_x, p19_y, p19_z, dist))
                q.append((p20_x, p20_y, p20_z, dist))
                q.append((p21_x, p21_y, p21_z, dist))
                q.append((p22_x, p22_y, p22_z, dist))
                q.append((p23_x, p23_y, p23_z, dist))
                dist = 5
            else:
                for x in q[::-1]:
                    previous_neighbours_list.append(x)
                    if len(previous_neighbours_list) >= 24:
                        break

                euclidean_0, euclidean_1, euclidean_2, euclidean_3, euclidean_4, euclidean_5, euclidean_6, euclidean_7, euclidean_8, euclidean_9, euclidean_10, euclidean_11, euclidean_12, euclidean_13, euclidean_14, euclidean_15, euclidean_16, euclidean_17, euclidean_18, euclidean_19, euclidean_20, euclidean_21, euclidean_22, euclidean_23 = ([] for _ in range(24))

                for i in previous_neighbours_list:

                    euclidean_dist1 = d_eucl(p0_x, p0_y, p0_z, i[0], i[1], i[2])
                    euclidean_dist2 = d_eucl(p1_x, p1_y, p1_z, i[0], i[1], i[2])
                    euclidean_dist3 = d_eucl(p2_x, p2_y, p2_z, i[0], i[1], i[2])
                    euclidean_dist4 = d_eucl(p3_x, p3_y, p3_z, i[0], i[1], i[2])
                    euclidean_dist5 = d_eucl(p4_x, p4_y, p4_z, i[0], i[1], i[2])
                    euclidean_dist6 = d_eucl(p5_x, p5_y, p5_z, i[0], i[1], i[2])
                    euclidean_dist7 = d_eucl(p6_x, p6_y, p6_z, i[0], i[1], i[2])
                    euclidean_dist8 = d_eucl(p7_x, p7_y, p7_z, i[0], i[1], i[2])
                    euclidean_dist9 = d_eucl(p8_x, p8_y, p8_z, i[0], i[1], i[2])
                    euclidean_dist10 = d_eucl(p9_x, p9_y, p9_z, i[0], i[1], i[2])
                    euclidean_dist11 = d_eucl(p10_x, p10_y, p10_z, i[0], i[1], i[2])
                    euclidean_dist12 = d_eucl(p11_x, p11_y, p11_z, i[0], i[1], i[2])
                    euclidean_dist13 = d_eucl(p12_x, p12_y, p12_z, i[0], i[1], i[2])
                    euclidean_dist14 = d_eucl(p13_x, p13_y, p3_z, i[0], i[1], i[2])
                    euclidean_dist15 = d_eucl(p14_x, p14_y, p14_z, i[0], i[1], i[2])
                    euclidean_dist16 = d_eucl(p15_x, p15_y, p15_z, i[0], i[1], i[2])
                    euclidean_dist17 = d_eucl(p16_x, p16_y, p16_z, i[0], i[1], i[2])
                    euclidean_dist18 = d_eucl(p17_x, p17_y, p17_z, i[0], i[1], i[2])
                    euclidean_dist19 = d_eucl(p18_x, p18_y, p18_z, i[0], i[1], i[2])
                    euclidean_dist20 = d_eucl(p19_x, p19_y, p19_z, i[0], i[1], i[2])
                    euclidean_dist21 = d_eucl(p20_x, p20_y, p20_z, i[0], i[1], i[2])
                    euclidean_dist22 = d_eucl(p21_x, p21_y, p21_z, i[0], i[1], i[2])
                    euclidean_dist23 = d_eucl(p22_x, p22_y, p22_z, i[0], i[1], i[2])
                    euclidean_dist24 = d_eucl(p23_x, p23_y, p23_z, i[0], i[1], i[2])
                #print(euclidean_dist8,euclidean_dist6,euclidean_dist7,euclidean_dist4,euclidean_dist5,euclidean_dist1,euclidean_dist2)
                if (
                        p0_x < x_restriction and p0_x > x_restriction_left and p0_y < y_restriction and p0_y > y_restriction_bottom and p0_z > z_restriction_front and p0_z < z_restriction_back and euclidean_dist1 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p0_x, p0_y, p0_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p0_x-15, p0_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p0_x, p0_y), 15, 1)
                        if (p0_x, p0_y, p0_z, dist) not in q:
                            q.append((p0_x, p0_y, p0_z, dist))
                if (
                        p1_x < x_restriction and p1_x > x_restriction_left and p1_y < y_restriction and p1_y > y_restriction_bottom and p1_z > z_restriction_front and p1_z < z_restriction_back and euclidean_dist2 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p1_x, p1_y, p1_z)) == 0:
                        # pygame.draw.circle(surf, color, (p1_x, p1_y), 15, 1)
                        # pygame.draw.rect(surf, color, pygame.Rect(p1_x-15, p1_y-15, 30, 30), 1)
                        if (p1_x, p1_y, p1_z, dist) not in q:
                            q.append((p1_x, p1_y, p1_z, dist))
                if (
                        p2_x < x_restriction and p2_x > x_restriction_left and p2_y < y_restriction and p2_y > y_restriction_bottom and p2_z > z_restriction_front and p2_z < z_restriction_back  and euclidean_dist3 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p2_x, p2_y, p2_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p2_x-15, p2_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p2_x, p2_y), 15, 1)
                        if (p2_x, p2_y, p2_z, dist) not in q:
                            q.append((p2_x, p2_y, p2_z, dist))
                if (
                        p3_x < x_restriction and p3_x > x_restriction_left and p3_y < y_restriction and p3_y > y_restriction_bottom and p3_z > z_restriction_front and p3_z < z_restriction_back  and euclidean_dist4 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p3_x, p3_y, p3_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p3_x-15, p3_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p3_x, p3_y), 15, 1)
                        if (p3_x, p3_y, p3_z, dist) not in q:
                            q.append((p3_x, p3_y, p3_z, dist))
                if (
                        p4_x < x_restriction and p4_x > x_restriction_left and p4_y < y_restriction and p4_y > y_restriction_bottom and p4_z > z_restriction_front and p4_z < z_restriction_back and euclidean_dist5 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p4_x, p4_y, p4_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p4_x-15, p4_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p4_x, p4_y), 15, 1)
                        if (p4_x, p4_y, p4_z, dist) not in q:
                            q.append((p4_x, p4_y, p4_z, dist))
                if (
                        p5_x < x_restriction and p5_x > x_restriction_left and p5_y < y_restriction and p5_y > y_restriction_bottom and p5_z > z_restriction_front and p5_z < z_restriction_back and euclidean_dist6 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p5_x, p5_y, p5_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p5_x-15, p5_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p5_x, p5_y), 15, 1)
                        if (p5_x, p5_y, p5_z, dist) not in q:
                            q.append((p5_x, p5_y, p5_z, dist))
                if (
                        p6_x < x_restriction and p6_x > x_restriction_left and p6_y < y_restriction and p6_y > y_restriction_bottom and p6_z > z_restriction_front and p6_z < z_restriction_back and euclidean_dist7 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p6_x, p6_y, p6_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p6_x-15, p6_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p6_x, p6_y), 15, 1)
                        if (p6_x, p6_y, p6_z, dist) not in q:
                            q.append((p6_x, p6_y, p6_z, dist))
                if (
                        p7_x < x_restriction and p7_x > x_restriction_left and p7_y < y_restriction and p7_y > y_restriction_bottom and p7_z > z_restriction_front and p7_z < z_restriction_back and euclidean_dist8 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p7_x, p7_y, p7_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p8_x-15, p8_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p8_x, p8_y), 15, 1)
                        if (p7_x, p7_y, p7_z, dist) not in q:
                            q.append((p7_x, p7_y, p7_z, dist))

                if (
                        p8_x < x_restriction and p8_x > x_restriction_left and p8_y < y_restriction and p8_y > y_restriction_bottom and p8_z > z_restriction_front and p8_z < z_restriction_back and euclidean_dist9 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p8_x, p8_y, p8_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p8_x-15, p8_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p8_x, p8_y), 15, 1)
                        if (p8_x, p8_y, p8_z, dist) not in q:
                            q.append((p8_x, p8_y, p8_z, dist))

                if (
                        p9_x < x_restriction and p9_x > x_restriction_left and p9_y < y_restriction and p9_y > y_restriction_bottom and p9_z > z_restriction_front and p9_z < z_restriction_back and euclidean_dist10 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p9_x, p9_y, p9_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p0_x-15, p0_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p0_x, p0_y), 15, 1)
                        if (p9_x, p9_y, p9_z, dist) not in q:
                            q.append((p9_x, p9_y, p9_z, dist))
                if (
                        p10_x < x_restriction and p10_x > x_restriction_left and p10_y < y_restriction and p10_y > y_restriction_bottom and p10_z > z_restriction_front and p10_z < z_restriction_back and euclidean_dist11 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p10_x, p10_y, p10_z)) == 0:
                        # pygame.draw.circle(surf, color, (p1_x, p1_y), 15, 1)
                        # pygame.draw.rect(surf, color, pygame.Rect(p1_x-15, p1_y-15, 30, 30), 1)
                        if (p10_x, p10_y, p10_z, dist) not in q:
                            q.append((p10_x, p10_y, p10_z, dist))
                if (
                        p11_x < x_restriction and p11_x > x_restriction_left and p11_y < y_restriction and p11_y > y_restriction_bottom and p11_z > z_restriction_front and p11_z < z_restriction_back and euclidean_dist12 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p11_x, p11_y, p11_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p2_x-15, p2_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p2_x, p2_y), 15, 1)
                        if (p11_x, p11_y, p11_z, dist) not in q:
                            q.append((p11_x, p11_y, p11_z, dist))
                if (
                        p12_x < x_restriction and p12_x > x_restriction_left and p12_y < y_restriction and p12_y > y_restriction_bottom and p12_z > z_restriction_front and p12_z < z_restriction_back and euclidean_dist13 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p12_x, p12_y, p12_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p3_x-15, p3_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p3_x, p3_y), 15, 1)
                        if (p12_x, p12_y, p12_z, dist) not in q:
                            q.append((p12_x, p12_y, p12_z, dist))
                if (
                        p13_x < x_restriction and p13_x > x_restriction_left and p13_y < y_restriction and p13_y > y_restriction_bottom and p13_z > z_restriction_front and p13_z < z_restriction_back and euclidean_dist14 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p13_x, p13_y, p13_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p4_x-15, p4_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p4_x, p4_y), 15, 1)
                        if (p13_x, p13_y, p13_z, dist) not in q:
                            q.append((p13_x, p13_y, p13_z, dist))
                if (
                        p14_x < x_restriction and p14_x > x_restriction_left and p14_y < y_restriction and p14_y > y_restriction_bottom and p14_z > z_restriction_front and p14_z < z_restriction_back and euclidean_dist15 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p14_x, p14_y, p14_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p5_x-15, p5_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p5_x, p5_y), 15, 1)
                        if (p14_x, p14_y, p14_z, dist) not in q:
                            q.append((p14_x, p14_y, p14_z, dist))
                if (
                        p15_x < x_restriction and p15_x > x_restriction_left and p15_y < y_restriction and p15_y > y_restriction_bottom and p15_z > z_restriction_front and p15_z < z_restriction_back and euclidean_dist16 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p15_x, p15_y, p15_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p6_x-15, p6_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p6_x, p6_y), 15, 1)
                        if (p15_x, p15_y, p15_z, dist) not in q:
                            q.append((p15_x, p15_y, p15_z, dist))
                if (
                        p16_x < x_restriction and p16_x > x_restriction_left and p16_y < y_restriction and p16_y > y_restriction_bottom and p16_z > z_restriction_front and p16_z < z_restriction_back and euclidean_dist17 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p16_x, p16_y, p16_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p8_x-15, p8_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p8_x, p8_y), 15, 1)
                        if (p16_x, p16_y, p16_z, dist) not in q:
                            q.append((p16_x, p16_y, p16_z, dist))

                if (
                        p17_x < x_restriction and p17_x > x_restriction_left and p17_y < y_restriction and p17_y > y_restriction_bottom and p17_z > z_restriction_front and p17_z < z_restriction_back and euclidean_dist18 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p17_x, p17_y, p17_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p8_x-15, p8_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p8_x, p8_y), 15, 1)
                        if (p17_x, p17_y, p17_z, dist) not in q:
                            q.append((p17_x, p17_y, p17_z, dist))

                if (
                        p18_x < x_restriction and p18_x > x_restriction_left and p18_y < y_restriction and p18_y > y_restriction_bottom and p18_z > z_restriction_front and p18_z < z_restriction_back and euclidean_dist19 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p18_x, p18_y, p18_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p0_x-15, p0_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p0_x, p0_y), 15, 1)
                        if (p18_x, p18_y, p18_z, dist) not in q:
                            q.append((p18_x, p18_y, p18_z, dist))
                if (
                        p19_x < x_restriction and p19_x > x_restriction_left and p19_y < y_restriction and p19_y > y_restriction_bottom and p19_z > z_restriction_front and p19_z < z_restriction_back and euclidean_dist20 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p19_x, p19_y, p19_z)) == 0:
                        # pygame.draw.circle(surf, color, (p1_x, p1_y), 15, 1)
                        # pygame.draw.rect(surf, color, pygame.Rect(p1_x-15, p1_y-15, 30, 30), 1)
                        if (p19_x, p19_y, p19_z, dist) not in q:
                            q.append((p19_x, p19_y, p19_z, dist))
                if (
                        p20_x < x_restriction and p20_x > x_restriction_left and p20_y < y_restriction and p20_y > y_restriction_bottom and p20_z > z_restriction_front and p20_z < z_restriction_back and euclidean_dist21 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p20_x, p20_y, p20_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p2_x-15, p2_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p2_x, p2_y), 15, 1)
                        if (p20_x, p20_y, p20_z, dist) not in q:
                            q.append((p20_x, p20_y, p20_z, dist))
                if (
                        p21_x < x_restriction and p21_x > x_restriction_left and p21_y < y_restriction and p21_y > y_restriction_bottom and p21_z > z_restriction_front and p21_z < z_restriction_back and euclidean_dist22 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p21_x, p21_y, p21_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p3_x-15, p3_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p3_x, p3_y), 15, 1)
                        if (p21_x, p21_y, p21_z, dist) not in q:
                            q.append((p21_x, p21_y, p21_z, dist))
                if (
                        p22_x < x_restriction and p22_x > x_restriction_left and p22_y < y_restriction and p22_y > y_restriction_bottom and p22_z > z_restriction_front and p22_z < z_restriction_back and euclidean_dist23 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p22_x, p22_y, p22_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p4_x-15, p4_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p4_x, p4_y), 15, 1)
                        if (p22_x, p22_y, p22_z, dist) not in q:
                            q.append((p22_x, p22_y, p22_z, dist))
                if (
                        p23_x < x_restriction and p23_x > x_restriction_left and p23_y < y_restriction and p23_y > y_restriction_bottom and p23_z > z_restriction_front and p23_z < z_restriction_back and euclidean_dist23 > 15):
                    if (check_obstacle(obstacles_coordinates_new, p23_x, p23_y, p23_z)) == 0:
                        # pygame.draw.rect(surf, color, pygame.Rect(p5_x-15, p5_y-15, 30, 30), 1)
                        # pygame.draw.circle(surf, color, (p5_x, p5_y), 15, 1)
                        if (p23_x, p23_y, p23_z, dist) not in q:
                            q.append((p23_x, p23_y, p23_z, dist))

                #print("q", q)

                print(dist)
                clockwise = False
                anticlockwise = True  # changing the direction to opposite
                f.append(q.pop(0))  # drop current element from queue and adding it to potential fields list
                dist = dist + 1.2  # increase the distance

        else:
            #print("anticlcockwise")
            p0_x = q[0][0] - dist
            p0_y = q[0][1] - dist
            p0_z = q[0][2]

            p1_x = q[0][0]
            p1_y = q[0][1] - dist
            p1_z = q[0][2]

            p2_x = q[0][0] + dist
            p2_y = q[0][1] - dist
            p2_z = q[0][2]

            p3_x = q[0][0] + dist
            p3_y = q[0][1]
            p3_z = q[0][2]

            p4_x = q[0][0] + dist
            p4_y = q[0][1] + dist
            p4_z = q[0][2]

            p5_x = q[0][0]
            p5_y = q[0][1] + dist
            p5_z = q[0][2]

            p6_x = q[0][0] - dist
            p6_y = q[0][1] + dist
            p6_z = q[0][2]

            p7_x = q[0][0] - dist
            p7_y = q[0][1]
            p7_z = q[0][2]
            #************************************
            p8_x = q[0][0] - dist
            p8_y = q[0][1] - dist
            p8_z = q[0][2] - dist

            p9_x = q[0][0]
            p9_y = q[0][1] - dist
            p9_z = q[0][2] - dist

            p10_x = q[0][0] + dist
            p10_y = q[0][1] - dist
            p10_z = q[0][2] - dist

            p11_x = q[0][0] + dist
            p11_y = q[0][1]
            p11_z = q[0][2] - dist

            p12_x = q[0][0] + dist
            p12_y = q[0][1] + dist
            p12_z = q[0][2] - dist

            p13_x = q[0][0]
            p13_y = q[0][1] + dist
            p13_z = q[0][2] - dist

            p14_x = q[0][0] - dist
            p14_y = q[0][1] + dist
            p14_z = q[0][2] - dist

            p15_x = q[0][0] - dist
            p15_y = q[0][1]
            p15_z = q[0][2] - dist
            #*********************************
            p16_x = q[0][0] - dist
            p16_y = q[0][1] - dist
            p16_z = q[0][2] + dist

            p17_x = q[0][0]
            p17_y = q[0][1] - dist
            p17_z = q[0][2] + dist

            p18_x = q[0][0] + dist
            p18_y = q[0][1] - dist
            p18_z = q[0][2] + dist

            p19_x = q[0][0] + dist
            p19_y = q[0][1]
            p19_z = q[0][2] + dist

            p20_x = q[0][0] + dist
            p20_y = q[0][1] + dist
            p20_z = q[0][2] + dist

            p21_x = q[0][0]
            p21_y = q[0][1] + dist
            p21_z = q[0][2] + dist

            p22_x = q[0][0] - dist
            p22_y = q[0][1] + dist
            p22_z = q[0][2] + dist

            p23_x = q[0][0] - dist
            p23_y = q[0][1]
            p23_z = q[0][2] + dist

            for x in q[::-1]:
                previous_neighbours_list.append(x)
                if len(previous_neighbours_list) >= 24:
                    break

            euclidean_0, euclidean_1, euclidean_2, euclidean_3, euclidean_4, euclidean_5, euclidean_6, euclidean_7, euclidean_8, euclidean_9, euclidean_10, euclidean_11, euclidean_12, euclidean_13, euclidean_14, euclidean_15, euclidean_16, euclidean_17, euclidean_18, euclidean_19, euclidean_20, euclidean_21, euclidean_22, euclidean_23 = ([] for _ in range(24))

            for i in previous_neighbours_list:
                euclidean_dist1 = d_eucl(p0_x, p0_y, p0_z, i[0], i[1], i[2])
                euclidean_dist2 = d_eucl(p1_x, p1_y, p1_z, i[0], i[1], i[2])
                euclidean_dist3 = d_eucl(p2_x, p2_y, p2_z, i[0], i[1], i[2])
                euclidean_dist4 = d_eucl(p3_x, p3_y, p3_z, i[0], i[1], i[2])
                euclidean_dist5 = d_eucl(p4_x, p4_y, p4_z, i[0], i[1], i[2])
                euclidean_dist6 = d_eucl(p5_x, p5_y, p5_z, i[0], i[1], i[2])
                euclidean_dist7 = d_eucl(p6_x, p6_y, p6_z, i[0], i[1], i[2])
                euclidean_dist8 = d_eucl(p7_x, p7_y, p7_z, i[0], i[1], i[2])
                euclidean_dist9 = d_eucl(p8_x, p8_y, p8_z, i[0], i[1], i[2])
                euclidean_dist10 = d_eucl(p9_x, p9_y, p9_z, i[0], i[1], i[2])
                euclidean_dist11 = d_eucl(p10_x, p10_y, p10_z, i[0], i[1], i[2])
                euclidean_dist12 = d_eucl(p11_x, p11_y, p11_z, i[0], i[1], i[2])
                euclidean_dist13 = d_eucl(p12_x, p12_y, p12_z, i[0], i[1], i[2])
                euclidean_dist14 = d_eucl(p13_x, p13_y, p3_z, i[0], i[1], i[2])
                euclidean_dist15 = d_eucl(p14_x, p14_y, p14_z, i[0], i[1], i[2])
                euclidean_dist16 = d_eucl(p15_x, p15_y, p15_z, i[0], i[1], i[2])
                euclidean_dist17 = d_eucl(p16_x, p16_y, p16_z, i[0], i[1], i[2])
                euclidean_dist18 = d_eucl(p17_x, p17_y, p17_z, i[0], i[1], i[2])
                euclidean_dist19 = d_eucl(p18_x, p18_y, p18_z, i[0], i[1], i[2])
                euclidean_dist20 = d_eucl(p19_x, p19_y, p19_z, i[0], i[1], i[2])
                euclidean_dist21 = d_eucl(p20_x, p20_y, p20_z, i[0], i[1], i[2])
                euclidean_dist22 = d_eucl(p21_x, p21_y, p21_z, i[0], i[1], i[2])
                euclidean_dist23 = d_eucl(p22_x, p22_y, p22_z, i[0], i[1], i[2])
                euclidean_dist24 = d_eucl(p23_x, p23_y, p23_z, i[0], i[1], i[2])

            if (
                    p0_x < x_restriction and p0_x > x_restriction_left and p0_y < y_restriction and p0_y > y_restriction_bottom and p0_z > z_restriction_front and p0_z < z_restriction_back and euclidean_dist1 > 15):
                if (check_obstacle(obstacles_coordinates_new, p0_x, p0_y, p0_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p0_x-15, p0_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p0_x, p0_y), 15, 1)
                    if (p0_x, p0_y, p0_z, dist) not in q:
                        q.append((p0_x, p0_y, p0_z, dist))
            if (
                    p1_x < x_restriction and p1_x > x_restriction_left and p1_y < y_restriction and p1_y > y_restriction_bottom and p1_z > z_restriction_front and p1_z < z_restriction_back and euclidean_dist2 > 15):
                if (check_obstacle(obstacles_coordinates_new, p1_x, p1_y, p1_z)) == 0:
                    # pygame.draw.circle(surf, color, (p1_x, p1_y), 15, 1)
                    # pygame.draw.rect(surf, color, pygame.Rect(p1_x-15, p1_y-15, 30, 30), 1)
                    if (p1_x, p1_y, p1_z, dist) not in q:
                        q.append((p1_x, p1_y, p1_z, dist))
            if (
                    p2_x < x_restriction and p2_x > x_restriction_left and p2_y < y_restriction and p2_y > y_restriction_bottom and p2_z > z_restriction_front and p2_z < z_restriction_back and euclidean_dist3 > 15):
                if (check_obstacle(obstacles_coordinates_new, p2_x, p2_y, p2_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p2_x-15, p2_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p2_x, p2_y), 15, 1)
                    if (p2_x, p2_y, p2_z, dist) not in q:
                        q.append((p2_x, p2_y, p2_z, dist))
            if (
                    p3_x < x_restriction and p3_x > x_restriction_left and p3_y < y_restriction and p3_y > y_restriction_bottom and p3_z > z_restriction_front and p3_z < z_restriction_back and euclidean_dist4 > 15):
                if (check_obstacle(obstacles_coordinates_new, p3_x, p3_y, p3_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p3_x-15, p3_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p3_x, p3_y), 15, 1)
                    if (p3_x, p3_y, p3_z, dist) not in q:
                        q.append((p3_x, p3_y, p3_z, dist))
            if (
                    p4_x < x_restriction and p4_x > x_restriction_left and p4_y < y_restriction and p4_y > y_restriction_bottom and p4_z > z_restriction_front and p4_z < z_restriction_back and euclidean_dist5 > 15):
                if (check_obstacle(obstacles_coordinates_new, p4_x, p4_y, p4_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p4_x-15, p4_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p4_x, p4_y), 15, 1)
                    if (p4_x, p4_y, p4_z, dist) not in q:
                        q.append((p4_x, p4_y, p4_z, dist))
            if (
                    p5_x < x_restriction and p5_x > x_restriction_left and p5_y < y_restriction and p5_y > y_restriction_bottom and p5_z > z_restriction_front and p5_z < z_restriction_back and euclidean_dist6 > 15):
                if (check_obstacle(obstacles_coordinates_new, p5_x, p5_y, p5_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p5_x-15, p5_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p5_x, p5_y), 15, 1)
                    if (p5_x, p5_y, p5_z, dist) not in q:
                        q.append((p5_x, p5_y, p5_z, dist))
            if (
                    p6_x < x_restriction and p6_x > x_restriction_left and p6_y < y_restriction and p6_y > y_restriction_bottom and p6_z > z_restriction_front and p6_z < z_restriction_back and euclidean_dist7 > 15):
                if (check_obstacle(obstacles_coordinates_new, p6_x, p6_y, p6_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p6_x-15, p6_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p6_x, p6_y), 15, 1)
                    if (p6_x, p6_y, p6_z, dist) not in q:
                        q.append((p6_x, p6_y, p6_z, dist))
            if (
                    p7_x < x_restriction and p7_x > x_restriction_left and p7_y < y_restriction and p7_y > y_restriction_bottom and p7_z > z_restriction_front and p7_z < z_restriction_back and euclidean_dist8 > 15):
                if (check_obstacle(obstacles_coordinates_new, p7_x, p7_y, p7_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p8_x-15, p8_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p8_x, p8_y), 15, 1)
                    if (p7_x, p7_y, p7_z, dist) not in q:
                        q.append((p7_x, p7_y, p7_z, dist))

            if (
                    p8_x < x_restriction and p8_x > x_restriction_left and p8_y < y_restriction and p8_y > y_restriction_bottom and p8_z > z_restriction_front and p8_z < z_restriction_back and euclidean_dist9 > 15):
                if (check_obstacle(obstacles_coordinates_new, p8_x, p8_y, p8_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p8_x-15, p8_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p8_x, p8_y), 15, 1)
                    if (p8_x, p8_y, p8_z, dist) not in q:
                        q.append((p8_x, p8_y, p8_z, dist))

            if (
                    p9_x < x_restriction and p9_x > x_restriction_left and p9_y < y_restriction and p9_y > y_restriction_bottom and p9_z > z_restriction_front and p9_z < z_restriction_back and euclidean_dist10 > 15):
                if (check_obstacle(obstacles_coordinates_new, p9_x, p9_y, p9_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p0_x-15, p0_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p0_x, p0_y), 15, 1)
                    if (p9_x, p9_y, p9_z, dist) not in q:
                        q.append((p9_x, p9_y, p9_z, dist))
            if (
                    p10_x < x_restriction and p10_x > x_restriction_left and p10_y < y_restriction and p10_y > y_restriction_bottom and p10_z > z_restriction_front and p10_z < z_restriction_back and euclidean_dist11 > 15):
                if (check_obstacle(obstacles_coordinates_new, p10_x, p10_y, p10_z)) == 0:
                    # pygame.draw.circle(surf, color, (p1_x, p1_y), 15, 1)
                    # pygame.draw.rect(surf, color, pygame.Rect(p1_x-15, p1_y-15, 30, 30), 1)
                    if (p10_x, p10_y, p10_z, dist) not in q:
                        q.append((p10_x, p10_y, p10_z, dist))
            if (
                    p11_x < x_restriction and p11_x > x_restriction_left and p11_y < y_restriction and p11_y > y_restriction_bottom and p11_z > z_restriction_front and p11_z < z_restriction_back and euclidean_dist12 > 15):
                if (check_obstacle(obstacles_coordinates_new, p11_x, p11_y, p11_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p2_x-15, p2_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p2_x, p2_y), 15, 1)
                    if (p11_x, p11_y, p11_z, dist) not in q:
                        q.append((p11_x, p11_y, p11_z, dist))
            if (
                    p12_x < x_restriction and p12_x > x_restriction_left and p12_y < y_restriction and p12_y > y_restriction_bottom and p12_z > z_restriction_front and p12_z < z_restriction_back and euclidean_dist13 > 15):
                if (check_obstacle(obstacles_coordinates_new, p12_x, p12_y, p12_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p3_x-15, p3_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p3_x, p3_y), 15, 1)
                    if (p12_x, p12_y, p12_z, dist) not in q:
                        q.append((p12_x, p12_y, p12_z, dist))
            if (
                    p13_x < x_restriction and p13_x > x_restriction_left and p13_y < y_restriction and p13_y > y_restriction_bottom and p13_z > z_restriction_front and p13_z < z_restriction_back and euclidean_dist14 > 15):
                if (check_obstacle(obstacles_coordinates_new, p13_x, p13_y, p13_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p4_x-15, p4_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p4_x, p4_y), 15, 1)
                    if (p13_x, p13_y, p13_z, dist) not in q:
                        q.append((p13_x, p13_y, p13_z, dist))
            if (
                    p14_x < x_restriction and p14_x > x_restriction_left and p14_y < y_restriction and p14_y > y_restriction_bottom and p14_z > z_restriction_front and p14_z < z_restriction_back and euclidean_dist15 > 15):
                if (check_obstacle(obstacles_coordinates_new, p14_x, p14_y, p14_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p5_x-15, p5_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p5_x, p5_y), 15, 1)
                    if (p14_x, p14_y, p14_z, dist) not in q:
                        q.append((p14_x, p14_y, p14_z, dist))
            if (
                    p15_x < x_restriction and p15_x > x_restriction_left and p15_y < y_restriction and p15_y > y_restriction_bottom and p15_z > z_restriction_front and p15_z < z_restriction_back and euclidean_dist16 > 15):
                if (check_obstacle(obstacles_coordinates_new, p15_x, p15_y, p15_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p6_x-15, p6_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p6_x, p6_y), 15, 1)
                    if (p15_x, p15_y, p15_z, dist) not in q:
                        q.append((p15_x, p15_y, p15_z, dist))
            if (
                    p16_x < x_restriction and p16_x > x_restriction_left and p16_y < y_restriction and p16_y > y_restriction_bottom and p16_z > z_restriction_front and p16_z < z_restriction_back and euclidean_dist17 > 15):
                if (check_obstacle(obstacles_coordinates_new, p16_x, p16_y, p16_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p8_x-15, p8_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p8_x, p8_y), 15, 1)
                    if (p16_x, p16_y, p16_z, dist) not in q:
                        q.append((p16_x, p16_y, p16_z, dist))

            if (
                    p17_x < x_restriction and p17_x > x_restriction_left and p17_y < y_restriction and p17_y > y_restriction_bottom and p17_z > z_restriction_front and p17_z < z_restriction_back and euclidean_dist18 > 15):
                if (check_obstacle(obstacles_coordinates_new, p17_x, p17_y, p17_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p8_x-15, p8_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p8_x, p8_y), 15, 1)
                    if (p17_x, p17_y, p17_z, dist) not in q:
                        q.append((p17_x, p17_y, p17_z, dist))

            if (
                    p18_x < x_restriction and p18_x > x_restriction_left and p18_y < y_restriction and p18_y > y_restriction_bottom and p18_z > z_restriction_front and p18_z < z_restriction_back and euclidean_dist19 > 15):
                if (check_obstacle(obstacles_coordinates_new, p18_x, p18_y, p18_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p0_x-15, p0_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p0_x, p0_y), 15, 1)
                    if (p18_x, p18_y, p18_z, dist) not in q:
                        q.append((p18_x, p18_y, p18_z, dist))
            if (
                    p19_x < x_restriction and p19_x > x_restriction_left and p19_y < y_restriction and p19_y > y_restriction_bottom and p19_z > z_restriction_front and p19_z < z_restriction_back and euclidean_dist20 > 15):
                if (check_obstacle(obstacles_coordinates_new, p19_x, p19_y, p19_z)) == 0:
                    # pygame.draw.circle(surf, color, (p1_x, p1_y), 15, 1)
                    # pygame.draw.rect(surf, color, pygame.Rect(p1_x-15, p1_y-15, 30, 30), 1)
                    if (p19_x, p19_y, p19_z, dist) not in q:
                        q.append((p19_x, p19_y, p19_z, dist))
            if (
                    p20_x < x_restriction and p20_x > x_restriction_left and p20_y < y_restriction and p20_y > y_restriction_bottom and p20_z > z_restriction_front and p20_z < z_restriction_back and euclidean_dist21 > 15):
                if (check_obstacle(obstacles_coordinates_new, p20_x, p20_y, p20_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p2_x-15, p2_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p2_x, p2_y), 15, 1)
                    if (p20_x, p20_y, p20_z, dist) not in q:
                        q.append((p20_x, p20_y, p20_z, dist))
            if (
                    p21_x < x_restriction and p21_x > x_restriction_left and p21_y < y_restriction and p21_y > y_restriction_bottom and p21_z > z_restriction_front and p21_z < z_restriction_back and euclidean_dist22 > 15):
                if (check_obstacle(obstacles_coordinates_new, p21_x, p21_y, p21_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p3_x-15, p3_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p3_x, p3_y), 15, 1)
                    if (p21_x, p21_y, p21_z, dist) not in q:
                        q.append((p21_x, p21_y, p21_z, dist))
            if (
                    p22_x < x_restriction and p22_x > x_restriction_left and p22_y < y_restriction and p22_y > y_restriction_bottom and p22_z > z_restriction_front and p22_z < z_restriction_back and euclidean_dist23 > 15):
                if (check_obstacle(obstacles_coordinates_new, p22_x, p22_y, p22_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p4_x-15, p4_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p4_x, p4_y), 15, 1)
                    if (p22_x, p22_y, p22_z, dist) not in q:
                        q.append((p22_x, p22_y, p22_z, dist))
            if (
                    p23_x < x_restriction and p23_x > x_restriction_left and p23_y < y_restriction and p23_y > y_restriction_bottom and p23_z > z_restriction_front and p23_z < z_restriction_back and euclidean_dist23 > 15):
                if (check_obstacle(obstacles_coordinates_new, p23_x, p23_y, p23_z)) == 0:
                    # pygame.draw.rect(surf, color, pygame.Rect(p5_x-15, p5_y-15, 30, 30), 1)
                    # pygame.draw.circle(surf, color, (p5_x, p5_y), 15, 1)
                    if (p23_x, p23_y, p23_z, dist) not in q:
                        q.append((p23_x, p23_y, p23_z, dist))

            clockwise = True
            anticlockwise = False
            f.append(q.pop(0))
            dist = dist + 1.2
            # print("potential: ", f)

        if len(q) == 0 or len(q) ==1:
            break


inner_vertices2, inner_faces2 = create_cube_vertices_and_faces(point_coordinates2, cube_edge_length)

vertices_obst1, faces_obst1 = create_cube_vertices_and_faces(obstacles_coordinates_new[0], cube_edge_length)
vertices_obst2, faces_obst2 = create_cube_vertices_and_faces(obstacles_coordinates_new[1], cube_edge_length)

vertices_obst3, faces_obst3 = create_cube_vertices_and_faces(obstacles_coordinates_new[2], cube_edge_length)
vertices_obst4, faces_obst4 = create_cube_vertices_and_faces(obstacles_coordinates_new[3], cube_edge_length)


# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Create Poly3DCollection for the outer cube
outer_poly3d = Poly3DCollection([[(outer_vertices[point][0], outer_vertices[point][1], outer_vertices[point][2]) for point in face] for face in outer_faces],
                                alpha=0.25, facecolors='cyan', linewidths=1, edgecolors='r')
ax.add_collection3d(outer_poly3d)

# Create Poly3DCollection for the inner cube
inner_poly3d = Poly3DCollection([[(inner_vertices[point][0], inner_vertices[point][1], inner_vertices[point][2]) for point in face] for face in inner_faces],
                                alpha=0.7, facecolors='black', linewidths=1, edgecolors=None)
ax.add_collection3d(inner_poly3d)

inner_poly3d2 = Poly3DCollection([[(inner_vertices2[point][0], inner_vertices2[point][1], inner_vertices2[point][2]) for point in face] for face in inner_faces2],
                                alpha=0.7, facecolors='purple', linewidths=1, edgecolors=None)
ax.add_collection3d(inner_poly3d2)
inner_poly3d2 = Poly3DCollection([[(vertices_obst1[point][0], vertices_obst1[point][1], vertices_obst1[point][2]) for point in face] for face in faces_obst1],
                                alpha=0.6, facecolors='red', linewidths=1, edgecolors=None)
ax.add_collection3d(inner_poly3d2)
inner_poly3d2 = Poly3DCollection([[(vertices_obst2[point][0], vertices_obst2[point][1], vertices_obst2[point][2]) for point in face] for face in faces_obst2],
                                alpha=0.6, facecolors='red', linewidths=1, edgecolors=None)

ax.add_collection3d(inner_poly3d2)
inner_poly3d2 = Poly3DCollection([[(vertices_obst3[point][0], vertices_obst3[point][1], vertices_obst3[point][2]) for point in face] for face in faces_obst3],
                                alpha=0.6, facecolors='red', linewidths=3, edgecolors=None)
ax.add_collection3d(inner_poly3d2)
inner_poly3d2 = Poly3DCollection([[(vertices_obst4[point][0], vertices_obst4[point][1], vertices_obst4[point][2]) for point in face] for face in faces_obst4],
                                alpha=0.6, facecolors='red', linewidths=3, edgecolors=None)

ax.add_collection3d(inner_poly3d2)

points_size = []

counter=0
# for n in range(1, len(f)+1):
#     if n < len(f)/3:
#         points_size.append(50)
#     else:
#         counter = counter + 1
#         points_size.append(50+counter*1.5)
for n in range(1, len(f)+1):
    counter = counter + 1
    points_size.append(50 + counter)

new_f = f.copy()
print(new_f)
x, y, z = zip(*new_f)
ax.scatter(x, y, z, c='lightgreen', marker='o', s = points_size, alpha= 0.35, edgecolors='g')  # You can change color and marker style

# Show the plot
plt.show()
#  size of the plot is determined by the limits set for the X, Y, and Z axes
# Set plot limits
ax.set_xlim([0, 150])
ax.set_ylim([0, 150])
ax.set_zlim([0, 150])

# Set axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Show the plot
plt.show()
