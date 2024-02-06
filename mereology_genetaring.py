import matplotlib as plt
import matplotlib.pyplot as pypl
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import math
import random


def d_eucl(x_temp, y_temp, z_temp, x2_temp, y2_temp, z2_temp):
    distance_temp1 = (x_temp - x2_temp) * (x_temp - x2_temp)
    distance_temp2 = (y_temp - y2_temp) * (y_temp - y2_temp)
    distance_temp = distance_temp1 + distance_temp2 + (z_temp - z2_temp) * (z_temp - z2_temp)
    distance_temp = math.sqrt(distance_temp)
    return (distance_temp)


def d_eucl2(x_temp, y_temp, x2_temp, y2_temp, goal_x, goal_y):
    distance_temp = (x_temp - x2_temp) * (x_temp - x2_temp)
    distance_temp = distance_temp + ((y_temp - y2_temp) * (y_temp - y2_temp))
    distance_temp = math.sqrt(distance_temp)
    distance_temp2 = (x2_temp - goal_x) * (x2_temp - goal_x)
    distance_temp2 = distance_temp2 + ((y2_temp - goal_y) * (y2_temp - goal_y))
    distance_temp2 = math.sqrt(distance_temp2)
    distance_temp3 = distance_temp + 0.4 * (distance_temp2)
    return (distance_temp3)


def generate_rand_points(start, end, num):
    res = []

    for j in range(num):
        res.append(random.randint(start, end))
    return res


# Check collisions with obstacles, if omit==1 than collision detected,
def check_obstacle(obstacles_coordinates, x_temp, y_temp, z_temp):
    # print("długośc tablicy :", len(obstacles_coordinates))
    omit = 0
    for i in range(len(obstacles_coordinates)):
        # print(d_eucl(x_temp, y_temp, obstacles_coordinates[i][0], obstacles_coordinates[i][1]))
        if (d_eucl(x_temp, y_temp, z_temp, obstacles_coordinates[i][0], obstacles_coordinates[i][1], obstacles_coordinates[i][2]) < 60):
            omit = 1
    return (omit)


def inCircle(lst_obstacles, lst_x, radius):
    points_inside_circle = []

    for point in lst_x:
        x, y = point
        is_inside = 0
        for i in range(len(lst_obstacles)):
            obstacle_x, obstacle_y = lst_obstacles[i]
            dx = abs(x - obstacle_x)
            dy = abs(y - obstacle_y)
            # Distance between point and obstacles
            distance = math.sqrt(dx ** 2 + dy ** 2)
            if distance <= radius:
                is_inside = 1
            else:
                is_inside = 0
            points_inside_circle.append(is_inside)

    return points_inside_circle


def linear_function_between2points(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    if x1 == x2:
        print("Dividing by 0")
        return None
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1

    return m, b


def coorinates_on_line(m, b, range_x):
    points = []
    for x in range_x:
        y = m * x + b
        points.append([x, y])
    return points


def search_path(closest_points, list_of_fields, iteration, goal_x, goal_y, obstacles_coordinates_check):
    path = []
    for j in range(iteration):
        # if (goal_x - 2) >= closest_points[0][0] <= (goal_x + 2) and (goal_y - 2) >= closest_points[0][1] <= (goal_y + 2):
        if (x_draw_goal - 2) <= closest_points[0][0] <= (x_draw_goal + 2) and (y_draw_goal - 2) <= closest_points[0][
            1] <= (y_draw_goal + 2):
            print("You get a goal!", closest_points)
            break
        else:
            min_x_values = []
            temp_min_points = []
            minimum = 0
            valid_field_found = False  # Flag to track whether a valid field is found
            for i in range(len(list_of_fields)):
                euclidean_points_dist = d_eucl2(closest_points[0][0], closest_points[0][1], list_of_fields[i][0],
                                                list_of_fields[i][1], goal_x, goal_y)

                if i == 0:
                    minimum = euclidean_points_dist
                    temp_min_points.clear()
                    temp_min_points.append((list_of_fields[i][0], list_of_fields[i][1]))
                else:
                    if euclidean_points_dist < minimum and euclidean_points_dist != 0:
                        min_x_values.clear()
                        x_closest = closest_points[0][0]
                        x_field = list_of_fields[i][0]
                        min_x_values.append(x_closest)
                        min_x_values.append(x_field)
                        min_current_x = min(min_x_values)
                        max_current_x = max(min_x_values)
                        _range = range(min_current_x, max_current_x, 10)
                        linear_function = linear_function_between2points(closest_points[0], list_of_fields[i][0:2])

                        if linear_function is not None:
                            m, b = linear_function
                            points_on_line = coorinates_on_line(m, b, _range)
                            table_of_results = inCircle(obstacles_coordinates_check, points_on_line, radius=60)

                            if all(v == 0 for v in table_of_results):
                                minimum = euclidean_points_dist
                                temp_min_points.clear()
                                temp_min_points.append((list_of_fields[i][0], list_of_fields[i][1]))
                                valid_field_found = True  # Set the flag to True
                            else:
                                points_on_line.clear()
                                table_of_results.clear()
                if i == (len(list_of_fields) - 1):
                    if valid_field_found:
                        counter = 0
                        for n in range(len(list_of_fields)):
                            if (n - counter) == (len(list_of_fields) - counter - 1):
                                break
                            if temp_min_points[0] == list_of_fields[n - 1][0:2]:
                                list_of_fields.pop(n - 1)
                                counter = counter + 1

                        print('Closet:', closest_points, 'tempminpoint', temp_min_points)
                        path.append((temp_min_points[0][0], temp_min_points[0][1]))
                        closest_points.clear()
                        closest_points = temp_min_points.copy()
                    else:
                        print("No valid field found in the last iteration.")
                        print("Path:", path)
                        return path


def search_min_distances(path, goal_coord):
    path_distances = []
    optimal_path = []
    for i in range(len(path)):
        euclid_dist = d_eucl(path[i][0], path[i][1], goal_coord[0][0], goal_coord[0][1])
        if i == 0:
            path_distances.append(euclid_dist)
            optimal_path.append(path[i])
        else:
            if euclid_dist < path_distances[-1]:
                optimal_path.append(path[i])
                path_distances.append((euclid_dist))
            if euclid_dist == path_distances[-1] and i + 1 < len(path) - 2:
                euclid_superset = d_eucl(path[i + 1][0], path[i + 1][1], goal_coord[0][0], goal_coord[0][1])
                if euclid_superset < euclid_dist:
                    optimal_path.pop(-1)
                    optimal_path.append(path[i])
                    path_distances.append(euclid_superset)
                if euclid_superset == euclid_dist and i + 2 < len(path) - 2:
                    euclid_superset2 = d_eucl(path[i + 2][0], path[i + 2][1], goal_coord[0][0], goal_coord[0][1])

                    if euclid_superset2 < euclid_dist:
                        optimal_path.pop(-1)
                        optimal_path.append(path[i])
                        path_distances.append(euclid_superset2)

                else:
                    continue

    return (optimal_path)


def path_smoothing(path, iter_number):
    # split coordinates into two lists
    x_list_x = [x for x, y in path]
    y_list_y = [y for x, y in path]

    x_list = x_list_x.copy()
    y_list = y_list_y.copy()
    x0 = x_list_x.copy()
    y0 = y_list_y.copy()
    alpha = .3
    beta = .3
    new_path = []
    for i0 in range(0, iter_number):
        for i in range(1, len(path) - 1):
            # part1
            x_list[i] = x_list[i] + alpha * (x_list[i - 1] + x_list[i + 1] - 2 * x_list[i])
            y_list[i] = y_list[i] + alpha * (y_list[i - 1] + y_list[i + 1] - 2 * y_list[i])
            # part2
            x_list[i] = x_list[i] + beta * (x0[i] - x_list[i])
            y_list[i] = y_list[i] + beta * (y0[i] - y_list[i])

        if i0 == iter_number - 1:
            new_path = list(zip(x_list, y_list))

    return (new_path)


def draw_path(start_coord, path, color):
    for point in range(len(path)):
        if point == 0:
            pygame.draw.line(surf, color, start_coord[0], path[point + 1], 2)
        else:
            pygame.draw.line(surf, color, path[point], path[point + 1], 2)
        if point + 1 == len(path) - 1:
            break


def draw_circle(lst_of_obst, color, radius):
    for obst in range(len(lst_of_obst)):
        pygame.draw.circle(surf, color, (lst_of_obst[obst][0], lst_of_obst[obst][1]), radius, 1)


def getImage(path, zoom=0.05):
    return OffsetImage(pypl.imread(path), zoom=zoom)


def to_pygame(coords, height):
    # Convert coordinates into pygame coordinates (lower-left => top left).
    return (coords[0][0], height - coords[0][1])


def cout_map_perc(obstacle, size_map_x, size_map_y, radius_):
    numer_of_obstacles = len(obstacle)
    map_field = size_map_x * size_map_y
    one_obstacle_field = 3.14 * (radius_ ** 2)
    occupied_field = (one_obstacle_field * numer_of_obstacles) / map_field
    return occupied_field


def to_pygame2(coords, height):
    # Convert coordinates into pygame coordinates (lower-left => top left).
    pygame_change = []
    for i in range(len(coords)):
        change = (coords[i][0], height - coords[i][1])
        pygame_change.append(change)
    return pygame_change


plt.rcParams.update({
    'figure.subplot.left': 0,
    'figure.subplot.bottom': 0,
    'figure.subplot.right': 1,
    'figure.subplot.top': 1,
    "lines.marker": "o",  # available ('o', 'v', '^', '<', '>', '8', 's', 'p', '*', 'h', 'H', 'D', 'd', 'P', 'X')
    "lines.linewidth": "0.4",
    "axes.prop_cycle": plt.cycler('color', ['white']),  # line color
    "text.color": "black",  # no text in this example
    "axes.facecolor": "white",  # background of the figure
    "axes.edgecolor": "gray",
    "axes.labelcolor": "black",  # no labels in this example
    "axes.grid": "True",
    "grid.linestyle": ":",
    "grid.color": "lightgray",
    "figure.edgecolor": "white",
})

paths = [
    'marker_images/marker_1184.png',
    'marker_images/marker_1751.png',
    'marker_images/marker_4076.png',
    'marker_images/marker_1281.png'
]
paths2 = [
    'marker_images/marker_2165.png',
    'marker_images/marker_733.png']

paths3 = [
    'marker_images/marker_497.png']

paths4 = [
    'start_images/start_point.jpg']

black_sqr = [
    'black/black_square1.png'
]
center_list = [(0, 0), (0, 500), (500, 500), (500, 0)]
goal_coordianets = [(490, 490, 490)]

obstacles_coordinates = [(1, 1, 1)]

start_coordinates = [(119, 128, 111)]

# dividing lists into 2 for getting separated x and y coordinates
x1, y1 = zip(*center_list)
x2, y2 = zip(*obstacles_coordinates)
x3, y3 = zip(*goal_coordianets)
x4, y4 = zip(*start_coordinates)
x_borders, x_obstacles, x_goal, y_borders, y_obstacles, y_goal, x_start, y_start = ([] for _ in range(8))

# finding minimum values in x,y coordinates from border
x_min = min(x1)
y_min = min(y1)

x_max = max(x1)
y_max = max(y1)
norm_coord_min = [(x_min, y_min)]

with open('txt_files/normalizing_coord.txt', 'w') as file:
    for x, y in norm_coord_min:
        file.write('{} {}\n'.format(x, y))

# normaliziing the elements, getting new (0,0) point on map
for i in range(len(x1)):
    a = x1[i] - x_min
    x_borders.append(a)
for i in range(len(x2)):
    b = x2[i] - x_min
    x_obstacles.append(b)
for i in range(len(x3)):
    c = x3[i] - x_min
    x_goal.append(c)
for i in range(len(x4)):
    d = x4[i] - x_min
    x_start.append(d)

for i in range(len(y1)):
    a = y1[i] - y_min
    y_borders.append(a)
for i in range(len(y2)):
    b = y2[i] - y_min
    y_obstacles.append(b)
for i in range(len(y3)):
    c = y3[i] - y_min
    y_goal.append(c)
for i in range(len(y4)):
    d = y4[i] - y_min
    y_start.append(d)
import numpy as np

# insert images insted of normal points
sizex = (x_borders[2] + 1) / 100
sizey = (y_borders[2] + 1) / 100
fig, ax = pypl.subplots()

fig.set_figheight(sizey)
fig.set_figwidth(sizex)

pypl.xticks(np.arange(0, x_borders[2], 1))
pypl.yticks(np.arange(0, y_borders[2], 1))

ax.scatter(x_borders, y_borders)
ax.scatter(x_obstacles, y_obstacles)
ax.scatter(x_goal, y_goal)
ax.scatter(x_start, y_start)

# get ride of the x and y axis values
pypl.tick_params(left=False, right=False, labelleft=False,
                 labelbottom=False, bottom=False)

print("borders: ", x_borders, "y: ", y_borders)
print("start: ", x_start, y_start)
print("goal: ", x_goal, y_goal)
print("obstacles: ", x_obstacles, "y : ", y_obstacles)

obstacles_images = []
for i in range(len(obstacles_coordinates)):
    for j in black_sqr:
        obstacles_images.append(j)

# display images as points in plot
for x0, y0, path in zip(x_borders, y_borders, paths):
    ab = AnnotationBbox(getImage(path), (x0, y0), frameon=False)
    ax.add_artist(ab)
for x0, y0, path in zip(x_obstacles, y_obstacles, obstacles_images):
    bc = AnnotationBbox(getImage(path), (x0, y0), frameon=False)
    ax.add_artist(bc)
for x0, y0, path in zip(x_goal, y_goal, paths3):
    cd = AnnotationBbox(getImage(path), (x0, y0), frameon=False)
    ax.add_artist(cd)
for x0, y0, path in zip(x_start, y_start, paths4):
    de = AnnotationBbox(getImage(path), (x0, y0), frameon=False)
    ax.add_artist(de)

ax = fig.gca()
pypl.savefig('plots/my_plot.png')

import pygame
from pygame.locals import *

pygame.init()

window = pygame.display.set_mode((sizex * 100, sizey * 100), DOUBLEBUF)
screen = pygame.display.get_surface()

surf = pygame.image.load('plots/my_plot.png')
# surf = pygame.transform.scale(surf_load, (650, 450))
size = surf.get_size()
print(size, "SIZEE")
################## SQUARE FILL AL ######################
goal_coordinates_pygame, start_coordinates_pygame, obstacles_coordinates_pygame1 = ([] for _ in range(3))
for i in x_goal:
    for j in y_goal:
        goal_coordinates_pygame.append((i, j))

for i in range(len(x_obstacles)):
    obstacles_coordinates_pygame1.append((x_obstacles[i], y_obstacles[i]))

for i in x_start:
    for j in y_start:
        start_coordinates_pygame.append((i, j))

goals_c = to_pygame(goal_coordinates_pygame, size[1])
x_draw_goal = goals_c[0]
y_draw_goal = goals_c[1]

# initial values
color = (50, 50, 50)
clockwise = True
anticlockwise = False
dist = 0  # current distance
f = []  # list of potential fields
q = []  # empty queue

obstacles_coordinates_new = to_pygame2(obstacles_coordinates_pygame1, size[1])

previous_neighbours_list = []
q.append((x_draw_goal, y_draw_goal, dist))  # adding initial values to the queue (goal coordinates and 0 as a current distance)



while len(q) > 0:
    x_restriction = 300
    x_restriction_left = 10
    y_restriction = 300
    y_restriction_bottom = 10
    z_restriction_front = 10
    z_restriction_back = 300

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
            dist = dist + 1  # increase the distance

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
            dist = dist + 1

        if len(q) == 0 or len(q) ==1:
            break




closest_points = []
start_coordinates1 = []
new_f = f.copy()
print("generated potential fields: ", new_f)

start_c = to_pygame(start_coordinates_pygame, size[1])
new_startX = start_c[0]
new_startY = start_c[1]
start_coordinates1.append((new_startX, new_startY))
closest_points.append((new_startX, new_startY))
minimum = 600

# generated_path = search_path(closest_points, new_f, 1000, x_draw_goal, y_draw_goal)
generated_path = search_path(closest_points, new_f, len(new_f), x_draw_goal, y_draw_goal,obstacles_coordinates_new)
# generated_path = search_path(closest_points, path2_artur, len(path2_artur), x_draw_goal, y_draw_goal,obstacles_coordinates_new)
optimal = search_min_distances(generated_path, [[x_draw_goal, y_draw_goal]])
path_smooth = path_smoothing(optimal, 10)

print(obstacles_coordinates, "first")
print(obstacles_coordinates_new, "new")
color1 = (100, 255, 0)
draw_path(start_coordinates1, generated_path, color=(100, 0, 255))
# draw_path(start_coordinates1, path_smooth,color1)
draw_circle(obstacles_coordinates_new, color=(255, 0, 0), radius=60)
field = 50
item = 50
screen.blit(surf, (0, 0))
# pygame.display.flip()
import time

crashed = False
while not crashed:

    for field in range(len(f) - 1):
        for item in range(len(f) - 1):
            pygame.draw.rect(screen, color, pygame.Rect(f[item][0], f[item][1], 2, 2), 2)
            pygame.display.update()
            time.sleep(0.01)
    # for point in range(len(generated_path)-1):
    #     if point == 0:
    #         pygame.draw.line(screen, color, start_coordinates1[0], generated_path[point + 1], 2)
    #         pygame.display.update()
    #         time.sleep(1)
    #     if point + 1 == len(generated_path):
    #         break
    #     if point >= 1:
    #         pygame.draw.line(screen, color, generated_path[point], generated_path[point + 1], 2)
    #         pygame.display.update()
    #         time.sleep(0.3)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            crashed = True

pygame.quit()
