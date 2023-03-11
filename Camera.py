import numpy as np
import cv2
import statistics
import pyrealsense2 as rs # Intel RealSense cross-platform open-source API
from Robot import Robot
from UR5 import UR5_COM

height = 480
width = 640
# width = 848


def connect_realsense():
    print('Connecting Camera...')
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
    # Start streaming
    pipeline.start(config)
    color_frame_flag = 0
    while color_frame_flag < 15:
        frames = pipeline.wait_for_frames()
        align_to = rs.stream.color
        # align_to = rs.stream.depth
        align = rs.align(align_to)
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        else:
            color_frame_flag = color_frame_flag+1
    pipeline.stop()
    # # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(aligned_color_frame.get_data())
    print('Camera Connection Accomplished')
    return aligned_depth_frame, color_image, depth_image


def extract_pixels(aligned_depth_frame, props_branches, depth_image):
    # print(depth_image.shape)
    # cv2.imshow(".", depth_image)
    # cv2.waitKey(0)

    step_size = 50
    img = cv2.imread('/home/arl1/Tohar/Outputs/output.png')
    z_array = np.zeros(len(props_branches['centroid-1']))
    # dist_array = np.zeros(len(props_branches['centroid-1']))
    for i in range(len(props_branches['centroid-1'])):
        # print('Point = ({}, {})'.format(x_centers[i], y_centers[i]))
        x = props_branches['centroid-1'][i]
        y = props_branches['centroid-0'][i]
        z = aligned_depth_frame.get_distance(x, y)
        if z >= 0.5 or z == 0:
            z = 0.25
        # if z == 0:
        #     z = 0.1
        #     z_neighbors = []
        #     # cv2.imshow(".", img[:,:, 0])
        #     # cv2.waitKey(0)
        #     if (x - step_size) < 0 or (x + step_size) > width or (y - step_size) < 0 or (y + step_size) > height:
        #         z = 0
        #     else:
        #         small_image = img[(x - step_size):(x + step_size), (y - step_size):(y + step_size), 0]
        #         pixels = np.where(small_image == 255) # Return elements chosen
        #         for j in range(len(pixels[0])):
        #             z_neighbors.append(aligned_depth_frame.get_distance(pixels[0][j], pixels[1][j]))
        #         z = statistics.median(z_neighbors)
        z_array[i] = z
        # dist_array[i] = ((props_branches['centroid-1'][i])+(props_branches['centroid-0'][i])+(z))**0.5
    # props_branches['dist'] = dist_array
    props_branches['z_dist'] = z_array
    props_branches = props_branches.sort_values(by=['z_dist'])
    # props_branches = props_branches[props_branches.z_dist != 0]
    print(props_branches)
    x = props_branches['centroid-1'][0]
    y = props_branches['centroid-0'][0]
    z = props_branches['z_dist'][0]

    point = np.array(rs.rs2_deproject_pixel_to_point(aligned_depth_frame.profile.as_video_stream_profile().intrinsics, [x, y], z))

    print(f'[x, y, z] = {np.around(point*1000, 3)} [mm]')
    print(f'[x, y, z] = {np.around(point, 3)} [m]')
    cv2.circle(img, (x,y), 4, (0, 0, 255))
    cv2.imshow(".", img)
    cv2.waitKey(0)
    x_move = point[0]
    y_move = point[1]
    z_move = point[2] ################ https://github.com/IntelRealSense/librealsense/issues/1274
    return z, x_move, y_move, z_move, props_branches
