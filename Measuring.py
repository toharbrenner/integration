import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math, cv2
from skimage.measure import label, regionprops, regionprops_table


def measure():
    # Measuring the size of the branches
    # output = cv2.imread('/home/arl1/Tohar/Outputs/output_color.png')
    # label_img = label(output[:, :, 0])
    # regions = regionprops(label_img)
    # fig, ax = plt.subplots(figsize=(15, 15))
    # ax.imshow(output, cmap=plt.cm.gray)
    #
    # for props in regions:
    #     y0, x0 = props.centroid
    #     orientation = props.orientation
    #     x1 = x0 + math.cos(orientation) * 0.5 * props.minor_axis_length
    #     y1 = y0 - math.sin(orientation) * 0.5 * props.minor_axis_length
    #     x2 = x0 - math.sin(orientation) * 0.5 * props.major_axis_length
    #     y2 = y0 - math.cos(orientation) * 0.5 * props.major_axis_length
    #
    #     ax.plot((x0, x1), (y0, y1), '-r', linewidth=2.5)
    #     ax.plot((x0, x2), (y0, y2), '-r', linewidth=2.5)
    #     ax.plot(x0, y0, '.g', markersize=15)
    #
    #     minr, minc, maxr, maxc = props.bbox
    #     bx = (minc, maxc, maxc, minc, minc)
    #     by = (minr, minr, maxr, maxr, minr)
    #     ax.plot(bx, by, '-b', linewidth=2.5)
    #
    # ax.axis((0, 640, 480, 0))
    # # plt.show()
    # props = regionprops_table(label_img,properties=('centroid', 'orientation', 'major_axis_length', 'minor_axis_length'))
    # props_pd = pd.DataFrame(props)
    # filter1 = props_pd['orientation'] > 0.1
    # filter2 = props_pd['orientation'] < -0.1
    # props_pd = props_pd.where(filter1 | filter2)
    # props_branches = props_pd.dropna()
    # props_branches = props_branches.reset_index(drop=True)

    image = cv2.imread("/home/arl1/Tohar/Outputs/output.png")
    props_branches = pd.DataFrame(columns=["centroid-1", "centroid-0"])
    contours, hierarchy = cv2.findContours(image[:, :, 0], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        # calculate moments for each contour
        M = cv2.moments(c)
        # calculate x,y coordinate of center
        if M["m00"] == 0:
            cX = int(M["m10"] / 0.000001)
            cY = int(M["m01"] / 0.000001)
        else:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        # cv2.circle(image, (cX, cY), 5, (255, 255, 255), -1)
        # cv2.putText(image, "centroid", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        print(f"x: {cX} y: {cY}")
        props_branches = props_branches.append({"centroid-1": cX, "centroid-0": cY}, ignore_index=True)
    print(props_branches)
    return props_branches


def convert_real_world(Z, props_branches):
    # Convert to real world measurement
    F = 1.88  # mm
    width = 640  # pixels
    high = 480  # pixels
    x_center = 320  # pixels coordinate
    y_center = 240  # pixels coordinate
    distance_pixels = 1.4 * (10 ** (-6)) * 1000  # micron mm
    length_vector = props_branches['major_axis_length']
    orientation_vector = props_branches['orientation']
    alfha = orientation_vector[0]  # radians
    y_centers = props_branches['centroid-0']
    x_centers = props_branches['centroid-1']
    beta = 180 - alfha

    x_edge = x_centers[0] - np.sin(beta) * length_vector[0]
    y_edge = y_centers[0] - np.cos(beta) * length_vector[0]
    delta_x = (x_edge - x_center) ** 2
    delta_y = (y_edge - y_center) ** 2
    B = distance_pixels * math.sqrt(delta_x + delta_y)  # mm
    alfha1 = np.arctan(B / F)  # radians

    Y = Z * np.tan(alfha1)
    A = distance_pixels * length_vector[0] * 2
    alfha2 = np.arctan(A + B / F)  # radians
    X = Z * np.tan(alfha2) - Y  # mm
    width_branch = distance_pixels * props_branches['minor_axis_length'][0]
    length_branch = X

    print('The width of the branch is', width_branch, 'mm')
    print('The length of the branch is', length_branch, 'mm')
    return width_branch, length_branch