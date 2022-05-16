import cv2
import numpy as np
from pathlib import Path
import argparse
import os
import urllib.request
import zipfile

# define hsv-scale on which the colors are chosen
max_h = 360
max_s = 100
max_v = 100

current_bbs = []
current_bbs_bin = []
img = np.array(())


def change_bb_callback(event, x, y, flags, param):
    """
    Callback to remove or re-add a bounding box, which might have been recognized a different object than a cone.

    :param event: Click event
    :param x: mouse X
    :param y: mouse Y
    :param flags:
    :param param:
    :return: None
    """

    global current_bbs
    global img
    global current_bbs_bin
    if event == cv2.EVENT_LBUTTONDOWN:
        temp_bbs = []
        # remove bounding box from bbs which should be saved
        for bb in current_bbs:
            if point_in_rect(x, y, bb):
                current_bbs.remove(bb)
                temp_bbs.append(bb)
                cv2.rectangle(img, (bb[0], bb[1]), (bb[0] + bb[2], bb[1] + bb[3]), (0, 0, 255), thickness=2)
                cv2.imshow('img', img)
        # re-add the bounding box if it's clicked on again
        for bb in current_bbs_bin:
            if point_in_rect(x, y, bb):
                current_bbs_bin.remove(bb)
                current_bbs.append(bb)
                cv2.rectangle(img, (bb[0], bb[1]), (bb[0] + bb[2], bb[1] + bb[3]), (255, 255, 255), thickness=2)
                cv2.imshow('img', img)
        current_bbs_bin.extend(temp_bbs)


def point_in_rect(x, y, rect):
    """
    Check if a point is inside a rect

    :param x: x-coord of the point
    :param y: y-coord of the point
    :param rect: rectangle
    :return: true if point is in rect, otherwise false
    """
    return rect[0] < x < rect[0] + rect[2] and rect[1] < y < rect[1] + rect[3]


def normalize_bound(max_hsv, bounds):
    """
    The range for HSV values is not standardizes, i.e. some programs (e.g. gimp) use h in [0, 360], s&v in [0,100].
    OpenCv has ranges in (180, 255, 255).
    This function normalizes the given hsv values to openCv scale.

    :param max_hsv: triple of (h,s,v) defining the max hsv values the colors are defined in.
    :param bounds: the color bounds containing the colors which should be transformed.
    :return: Normalized colors
    """

    normalized = np.zeros_like(bounds)
    normalized[:, 0] = bounds[:, 0] / max_hsv[0] * 180
    normalized[:, 1] = bounds[:, 1] / max_hsv[1] * 255
    normalized[:, 2] = bounds[:, 2] / max_hsv[1] * 255

    return normalized


def find_bbs_for_colors(img, color_bounds, color_name='', show_img=False):
    """
    Find the bounding boxes for cones with colors within the color range

    :param img: Image to find the bounding boxes for
    :param color_bounds: numpy array containing the two colors defining
    the range of colors for which the bbs should be calculated
    :param color_name: Name of the color for which the bb is drawn (orange, yellow, blue).
    Defines color of bb when show_img=True. Also sets the window name if show_img=True.
    :param show_img: Set to true if the masked image should be displayed
    :return: List of rectangles defining the bounding boxes. Each rect is a 4-Tuple with (x[0], x[1])->upper left corner
    (x[3], x[4])-> width,height
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask_color = cv2.inRange(hsv, *color_bounds)

    # remove small pixel fragments that had the target color just by chance
    erosion_kernel = np.ones((3, 3), np.uint8)
    dilation_kernel = np.ones((5, 5), np.uint8)
    mask_color = cv2.erode(mask_color, erosion_kernel)
    mask_color = cv2.dilate(mask_color, dilation_kernel, iterations=3)
    mask_color = cv2.erode(mask_color, erosion_kernel, iterations=3)

    # apply the mask
    if show_img:
        masked_img = cv2.bitwise_and(img, img, mask=mask_color)
        cv2.imshow(f"masked_img_{color_name}", masked_img)

    # find the contours of the cones
    contours, _ = cv2.findContours(mask_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # get the bounding boxes instead of contours
    bounding_boxes = []
    for c in contours:
        rect = cv2.boundingRect(c)
        # don't add bb if it's too small
        if rect[2] * rect[3] > 400: bounding_boxes.append(rect)

    return bounding_boxes


def main():
    # read cli args
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('dir', metavar='dir', type=str, default='data/TrainingImages', nargs='?',
                        help='relative path to the image directory')

    args = parser.parse_args()
    data_dir = args.dir

    # check for valid path to images
    if not os.path.exists(data_dir):
        error_col = '\033[91m'
        print(f"{error_col}Error: No image directory found, do you want to download the data?")
        response = input("Yes[Y]/No[n] ")
        if response == "Y":
            load_data()
        else:
            print(f"{error_col}"
                  f"Error: Invalid path to the image directory. Add the relative path as command line argument,"
                  f"or move images to data/TrainingImages")
            exit(1)

    # get global variables
    global current_bbs
    global current_bbs_bin
    global img
    cv2.namedWindow('img')
    cv2.setMouseCallback('img', change_bb_callback)

    info_col = '\033[95m'
    default_col = '\x1b[0m'
    print(f"{info_col}"
          f"Press 'Q' to quit, 'S' to skip image and dont save the bounding boxes or press any other key to continue\n"
          f"Click on a bounding box to exclude it, click again to include.{default_col}")

    img_dir = Path(data_dir)
    for img_path in img_dir.iterdir():
        current_bbs = []  # reset cached bounding boxes
        current_bbs_bin = []
        print(img_path)

        img = cv2.imread(str(img_path))

        # define colors for the bounding boxes
        bbs_col = [(0, 205, 228),  # yellow
                   (255, 0, 0),  # blue
                   (9, 115, 225)]  # orange

        # define the lower and upper bounds of the colors that should be extracted
        yellow_bound = np.array([[36, 40, 40],
                                 [68, 100, 100]])

        blue_bound = np.array([[212, 55, 25],
                               [223, 90, 55]])

        orange_bound = np.array([[9, 55, 45],
                                 [26, 73, 70]])

        # normalize the colors to openCV scale
        normalized_col_bounds = [normalize_bound((max_h, max_s, max_v), bound) for bound in
                                 [yellow_bound, blue_bound, orange_bound]]

        for col_bound, bb_col in zip(normalized_col_bounds, bbs_col):
            bbs = find_bbs_for_colors(img, col_bound, color_name=str(bb_col), show_img=False)
            current_bbs += bbs

            for rect in bbs:
                cv2.rectangle(img, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), bb_col)

        # masked_img = cv2.drawContours(masked_img, contours, -1, 180)

        cv2.imshow('img', img)

        pressed_key = cv2.waitKey(0) & 0xFF
        if pressed_key == ord('q'):
            break
        elif pressed_key == ord('s'):
            # bounding boxes are not saved
            pass
        else:
            # TODO save bounding boxes
            pass


def load_data():
    urllib.request.urlretrieve("https://ruhr-uni-bochum.sciebo.de/s/2KBSUjMWISEJOj3/download", "data.zip")
    with zipfile.ZipFile('data.zip', 'r') as zip_ref:
        zipinfos = zip_ref.infolist()
        for zipinfo in zipinfos:
            zipinfo.filename = zipinfo.filename.replace(' ', '')
            zip_ref.extract(zipinfo, 'data')
    os.remove('data.zip')


if __name__ == '__main__':
    main()
