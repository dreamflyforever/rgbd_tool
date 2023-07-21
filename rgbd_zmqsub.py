import zmq
import sys
import open3d as o3d
import numpy as np
import pickle

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://10.1.3.97:8003")

socket.setsockopt_string(zmq.SUBSCRIBE, "")

while True:
    msg = socket.recv()
    print(msg)
    #vtx = pickle.loads(msg)
    vtx = np.frombuffer(msg, dtype=[('f0', '<f4'), ('f1', '<f4'), ('f2', '<f4')])
    print(vtx)
    print('vtx', type(vtx), vtx.shape)
    #np.save('points', npstr)
    #vtx = np.load('p.npy')
    #print(vtx)
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
