import cv2
import numpy as np
import red_detection as red
global detected_points_0
def reorder_points(input_points):
    # Sort the input points based on x-coordinate
    sorted_points = sorted(input_points, key=lambda x: x[0])

    # Split the sorted points into top and bottom halves
    top_half = sorted_points[:2]
    bottom_half = sorted_points[2:]

    # Sort the top and bottom halves based on y-coordinate
    top_half = sorted(top_half, key=lambda x: x[1], reverse=True)
    bottom_half = sorted(bottom_half, key=lambda x: x[1])

    # Combine the sorted top and bottom halves
    result_points = top_half + bottom_half

    return result_points


def detect_and_visualize_colored_squares(camera_id=2):
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

    # Defining the red color range in HSV
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    # Defining the blue color range in HSV
    lower_blue = np.array([110, 50, 50])
    upper_blue = np.array([130, 255, 255])

    detected_points = []

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error reading from the camera.")
            break

        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create masks for red and blue colors
        red_mask = cv2.inRange(hsv, lower_red, upper_red)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Find contours in the red mask
        contours_red, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find contours in the blue mask
        contours_blue, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Reset the detected points for each frame
        detected_points_0 = []

        for contours in [contours_red, contours_blue]:
            for contour in contours:
                # Approximate the contour to get a polygon
                epsilon = 0.04 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                # If the polygon has four vertices (a square)
                if len(approx) == 4 and cv2.contourArea(contour) > 100:
                    # Get the centroid of the square
                    moments = cv2.moments(contour)
                    cx = int(moments['m10'] / moments['m00'])
                    cy = int(moments['m01'] / moments['m00'])
                    detected_points_0.append((cx, cy))

                    # Draw a circle at the centroid
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        # Draw lines between detected points
        if len(detected_points_0) == 4:
            detected_points_0 = reorder_points(detected_points_0)
            for i in range(4):
                cv2.line(frame, detected_points_0[i], detected_points_0[(i + 1) % 4], (0, 255, 0), 2)
            cv2.line(frame, detected_points_0[3], detected_points_0[0], (0, 255, 0), 2)

        cv2.imshow('Colored Square Detection', frame)

        # Stop the program and print the values if four points are detected
        if len(detected_points_0) == 4:
            print("Detected Points (Red and Blue):")
            for i, point in enumerate(detected_points_0):
                print(f"Point {i + 1}: {point}")

            with open('txt_files/boundaries_detect_2blue_2red.txt', 'w') as file:
                for x, y in detected_points_0:
                    file.write('{} {}\n'.format(x, y))


            # Stop the camera
            cap.release()
            cv2.destroyAllWindows()
            break

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # cap.release()
    # cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_and_visualize_colored_squares()
