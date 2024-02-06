import concurrent.futures
import cv2
import numpy as np
import red_detection as red
import blue_and_red_detection as blue_red
import detect_green_cube as green_cube
def main():
    with concurrent.futures.ProcessPoolExecutor() as executor:
        # Submit functions to the executor
        future1 = executor.submit(blue_red.detect_and_visualize_colored_squares)
        future2 = executor.submit(red.detect_and_visualize_red_square)

        # Wait for both functions to complete
        future1.add_done_callback(lambda _: executor.submit(green_cube.detect_green_cube))
        future2.add_done_callback(lambda _: executor.submit(green_cube.detect_green_cube))

        concurrent.futures.wait([future1, future2])
if __name__ == "__main__":
    main()