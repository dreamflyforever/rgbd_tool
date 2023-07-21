# First import the library
import pyrealsense2 as rs
import cv2
import numpy as np
import pickle
# Create a context object. This object owns the handles to all connected realsense devices

# python 3.6
import zmq
import random
import time
import open3d as o3d

broker = 'localhost'
port = 1885
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 1000)}'

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:8003")

pipeline = rs.pipeline()
config = rs.config()  # 定义配置config

config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 15)
config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 15)
profile = pipeline.start(config)
#profile = pipeline.start()
align_to = rs.stream.color
align = rs.align(align_to)

def get_aligned_images():
    frames = pipeline.wait_for_frames()  # 等待获取图像帧
    aligned_frames = align.process(frames)  # 获取对齐帧
    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的depth帧
    color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的color帧
    if not aligned_depth_frame or not color_frame:
        print("wrong!")
    pc = rs.pointcloud()
    pc.map_to(color_frame)
    points = pc.calculate(aligned_depth_frame)
    print("points",type(points))
    data_p = points.get_vertices()
    #print("number", type(data_p))
    vtx = np.asanyarray(data_p)

    print("vtx", type(vtx), vtx.shape, vtx.dtype)
    np.save('points', vtx)
    print('=====')
    vtx_to = vtx.tobytes()
    socket.send(vtx_to)
    print('-------------------------------------------')

    vtx = np.frombuffer(vtx_to, dtype = vtx.dtype)
    #print(vtx.shape)
    npy_vtx = np.zeros((len(vtx), 3), float)
    for i in range(len(vtx)):
        npy_vtx[i][0] = np.float64(vtx[i][0])
        npy_vtx[i][1] = np.float64(vtx[i][1])
        npy_vtx[i][2] = -np.float64(vtx[i][2])
    #print("npy_vtx",npy_vtx.shape,npy_vtx)  

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(npy_vtx)
    #显示点云
    o3d.visualization.draw_geometries([pcd])
    ############### 相机参数的获取 #######################
    intr = color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile(
    ).intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
    '''camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                         'ppx': intr.ppx, 'ppy': intr.ppy,
                         'height': intr.height, 'width': intr.width,
                         'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                         }'''

    # 保存内参到本地
    # with open('./intrinsics.json', 'w') as fp:
    #json.dump(camera_parameters, fp)
    #######################################################

    depth_image = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）
    print(depth_image.dtype)
    cv2.imwrite("depth.png", depth_image)
    #print(depth_image)
    #print(depth_image.size)
    #strlist = str(depth_image)
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  # 深度图（8位）
    depth_image_3d = np.dstack(
        (depth_image_8bit, depth_image_8bit, depth_image_8bit))  # 3通道深度图
    color_image = np.asanyarray(color_frame.get_data())  # RGB图
    #print(color_image)
    #print(color_image.dtype)
    #print(color_image.size)
    cv2.imwrite("color.png", color_image)
    # 返回相机内参、深度参数、彩色图、深度图、齐帧中的depth帧
    return intr, depth_intrin, color_image, depth_image, aligned_depth_frame

if __name__== '__main__':
    get_aligned_images()
    pipeline.stop()
