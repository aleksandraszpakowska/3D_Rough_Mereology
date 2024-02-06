import cv2
import numpy as np

def read_coordinates(file_name):
    with open(file_name) as file:
        coordinates = []
        for line in file:
            row = line.split()
            x = row[0]
            y = row[1]
            coordinates.append([float(x), float(y)])
    return coordinates

def reverse_y(coords, height):
    #Convert coordinates into pygame coordinates (lower-left => top left).
    _change =[]
    for i in range(len(coords)):
        change = (coords[i][0], height - coords[i][1])
        _change.append(change)
    return _change

def reverse_y_point(coords, height):
    #Convert coordinates into pygame coordinates (lower-left => top left).
    return (coords[0], height - coords[1])


def find_minimum_of_the_sum(list_of_coords):
    minimum = 1000
    point = []
    for item in range(len(list_of_coords)):
        sum_of_xy = list_of_coords[item][0] + list_of_coords[item][1]
        if sum_of_xy < minimum:
            minimum = sum_of_xy
            point.clear()
            point.append([list_of_coords[item][0], list_of_coords[item][1]])
            if len(list_of_coords) == 5:
                break
    return point


def find_max_of_the_sum(list_of_coords):
    maximum = 0
    point = []
    for item in range(len(list_of_coords)):
        sum_of_xy = list_of_coords[item][0] + list_of_coords[item][1]
        if sum_of_xy > maximum:
            maximum = sum_of_xy
            point.clear()
            point.append([list_of_coords[item][0], list_of_coords[item][1]])
            if len(list_of_coords) == 5:
                break

    return point

def convert_px(point,scale,bound_lst, height):

    bound_lst = reverse_y(bound_lst, height)
    point = reverse_y_point(point, height)
    new_00 = find_minimum_of_the_sum(bound_lst)
    new_x = point[0]-new_00[0][0]
    new_y = point[1]-new_00[0][1]

    x = new_x * scale[0]
    y = new_y * scale[1]

    converted_point = [x, y]
    print("point converted from function", converted_point)
    return converted_point

def get_scale(bound_lst,height):

    bound_lst = reverse_y(bound_lst, height)
    min_lst = find_minimum_of_the_sum(bound_lst)
    max_lst = find_max_of_the_sum(bound_lst)

    pixel_distance_x = max_lst[0][0] - min_lst[0][0]
    pixel_distance_y = max_lst[0][1] - min_lst[0][1]
    print("pixel_distance_x",pixel_distance_x, "pixel_distance_y ", pixel_distance_y)
    real_world_distance = 150  # in some units

    if pixel_distance_x == 0:
        pixel_distance_x = 1
    if pixel_distance_y == 0:
        pixel_distance_y = 1

    scale_x = real_world_distance / pixel_distance_x
    scale_y = real_world_distance / pixel_distance_y
    scale = [scale_x,scale_y]
    print("scale: ", scale)
    return scale

def project_point(point, projection_matrix):
    # Oblicz rzut punktu na płaszczyznę 2D
    projected_point = np.dot(projection_matrix, point)

    return projected_point

def detect_green_cube(camera_id0=0, camera_id1=2):
    # Inicjalizacja kamery
    size_width = 600
    size_height = 600
    cap0 = cv2.VideoCapture(camera_id0)
    cap1 = cv2.VideoCapture(camera_id1)
    cap0.set(cv2.CAP_PROP_FRAME_WIDTH, size_width)
    cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, size_height)
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, size_width)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, size_height)
    # Definiowanie rzutu aksonometrycznego

    bound_red = read_coordinates('txt_files/boundaries_detect_4red.txt')
    bound_blue_red = read_coordinates('txt_files/boundaries_detect_2blue_2red.txt')

    scale_red = get_scale(bound_red, size_height)
    scale_blue_red = get_scale(bound_blue_red, size_height)

    projection_matrix = np.array([[1, 0, 0],
                                  [0, 1, 0],
                                  [0, 0, 1]])
    z_for_cam0 = 0
    z_for_cam1 = 0

    while True:
        z = 0
        # Odczyt obrazu z kamery
        ret0, frame0 = cap0.read()
        ret1, frame1 = cap1.read()
        if not ret0 or not ret1:
            print("Błąd podczas odczytu obrazu z kamery.")
            break

        # Konwersja obrazu na przestrzeń kolorów HSV
        hsv0 = cv2.cvtColor(frame0, cv2.COLOR_BGR2HSV)
        hsv1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        # Zakres koloru zielonego w przestrzeni HSV z większą tolerancją
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])

        # Utworzenie maski koloru zielonego
        mask0 = cv2.inRange(hsv0, lower_green, upper_green)
        mask1 = cv2.inRange(hsv1, lower_green, upper_green)
        # Filtracja maski, aby pozbyć się szumów
        mask0 = cv2.erode(mask0, None, iterations=2)
        mask0 = cv2.dilate(mask0, None, iterations=2)

        mask1 = cv2.erode(mask1, None, iterations=2)
        mask1 = cv2.dilate(mask1, None, iterations=2)
        # Znalezienie konturów na masce
        contours0, _ = cv2.findContours(mask0, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours1, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours0:


            # Aproksymacja konturu
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Jeśli kontur ma 6 wierzchołków (szóstkąt foremny)
            if len(approx) == 6 and cv2.contourArea(contour) > 100:
                # Rysowanie konturu z uwzględnieniem rzutu aksonometrycznego
                x_values = [point[0][0] for point in approx]
                y_values = [point[0][1] for point in approx]

                central_point = (int(np.median(x_values)), int(np.median(y_values)))
                print(central_point,"central point")
                points_real = convert_px(central_point,scale_red,bound_red,size_height)
                # points_real = convert_pixel_to_length(reverse_y_point(central_point,size_height), bound_blue_red, size_height)
                # print(f'Punkt centralny figury z cam 0: x = {central_point[0]}, y = {central_point[1]}, z = {z_for_cam0}')
                print(f'Punkt centralny figury z cam 0: x = {points_real[0]}, y = {points_real[1]}, z = {z_for_cam0}')

                z_for_cam1 = points_real[0]
                projected_contour = []
                for point in approx:
                    x, y = point[0]
                    # z = 0  # Współrzędna z
                    projected_point = project_point(np.array([x, y, z]), projection_matrix)
                    projected_contour.append((int(projected_point[0]), int(projected_point[1])))
                    # Wypisanie współrzędnych x, y i z

                    x, y, z = projected_point

                    # print(f'Współrzędne punktu: x = {x}, y = {y}, z = {z}')

                print('**************')
                cv2.polylines(frame0, [np.array(projected_contour)], True, (0, 255, 0), 1)
                cv2.putText(frame0, 'Zielona kostka', projected_contour[0], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0),
                            2)
        for contour in contours1:
            # Aproksymacja konturu

            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Jeśli kontur ma 6 wierzchołków (szóstkąt foremny)
            if len(approx) == 6 and cv2.contourArea(contour) > 100:
                # Rysowanie konturu z uwzględnieniem rzutu aksonometrycznego
                x_values = [point[0][0] for point in approx]
                y_values = [point[0][1] for point in approx]

                central_point = (int(np.median(x_values)), int(np.median(y_values)))
                print(central_point, "central point")
                points_real = convert_px(central_point, scale_blue_red, bound_blue_red, size_height)
                print(f'Punkt centralny figury z cam 1: x = {points_real[0]}, y = {points_real[1]}, z ={z_for_cam1} ')
                z_for_cam0 = points_real[0]
                projected_contour = []
                for point in approx:
                    x, y = point[0]
                    # z = 0  # Współrzędna z
                    projected_point = project_point(np.array([x, y, z]), projection_matrix)
                    projected_contour.append((int(projected_point[0]), int(projected_point[1])))
                    # Wypisanie współrzędnych x, y i z
                    x, y, z = projected_point
                    # print(f'Współrzędne punktu: x = {x}, y = {y}, z = {z}')
                # z = np.median(projected_contour)

                print('**************')
                cv2.polylines(frame1, [np.array(projected_contour)], True, (0, 255, 0), 1)
                cv2.putText(frame1, 'Zielona kostka', projected_contour[0], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0),
                            2)

        # Wyświetlenie obrazu z zaznaczoną zieloną kostką
        cv2.imshow('Green Cube Detection 0 ', frame0)
        cv2.imshow('Green Cube Detection 1 ', frame1)
        # Przerwanie pętli po naciśnięciu klawisza 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Zwolnienie kamery i zamknięcie okna OpenCV
    cap0.release()
    cap1.release()
    cv2.destroyAllWindows()



if __name__ == "__main__":

    detect_green_cube()




