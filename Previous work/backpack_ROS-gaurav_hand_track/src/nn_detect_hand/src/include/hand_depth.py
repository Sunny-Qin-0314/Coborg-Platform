#!/usr/bin/env python
import cv2
import numpy as np
import pyrealsense2 as rs


def reduce_size(scale_percent,img):
        scale_percent = 50  # percent of original size
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        dim = (width, height)
        # resize image
        return cv2.resize(img, dim, interpolation=cv2.INTER_AREA)


def img_correction(un_frames,align,thr):
        depth_matrix = np.zeros((700, 700))  # correct this
        # Wait for a coherent pair of frames: depth and color
        frames = align.process(un_frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Intrinsics & Extrinsics
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(
            color_frame.profile)

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # depth_image = reduce_size(50, depth_image)
        # color_image = reduce_size(50, color_image)

        # print(depth_image.shape)

        color_image
        img_size = color_image.shape


        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))


        #curr_frame += 1
        return color_image, depth_frame, depth_intrin

def hand_distance(depth_frame,depth_intrin,cent_pix,thr):
        K=[]
        #return distance of detected hand in meters
        for i in range(len(cent_pix)):
                depth = depth_frame.get_distance(int(cent_pix[i][0]), int(cent_pix[i][1]))
                if depth < thr and depth!=0.0:
                        xyz = rs.rs2_deproject_pixel_to_point(depth_intrin, [int(cent_pix[i][0]),int(cent_pix[i][1])], depth)
                        K.append(xyz)
        return K
