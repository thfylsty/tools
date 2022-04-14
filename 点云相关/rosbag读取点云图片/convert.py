import rosbag
# import pyrosbag as rosbag
from sensor_msgs import point_cloud2
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import json
import numpy as np
# from tqdm import tqdm
import argparse

bridge = CvBridge()


extrinsic = [[[0.78652859, 0.40300682, -0.46792981, 2.6400001],
              [-0.44195294, 0.896559, 0.029301409, -0.89999998],
              [0.43133563, 0.18375656, 0.88327974, -3.24],
              [0.0, 0.0, 0.0, 1.0]],
             [[0.02959645, -0.8475315, -0.52991927, 2.5400014],
              [0.99919134, 0.039519709, -0.0074005215, -2.54],
              [0.027214428, -0.52927166, 0.84801579, -4.0200005],
              [0.0, 0.0, 0.0, 1.0]],
             [[-0.036769189, 0.97940832, -0.19851273, 1.7399999],
              [-0.99614251, -0.02008372, 0.085421182, 2.8400002],
              [0.079675347, 0.20088783, 0.97636878, -5.2399998],
              [0.0, 0.0, 0.0, 1.0]]
             ]
intrinsic = [[[4.3808785552295910e+02, 0.000000e+00, 6.0585738792418056e+02],
              [0.000000e+00, 4.3608672016711375e+02, 3.9094842550230072e+02],
              [0.000000e+00, 0.000000e+00, 1.000000e+00]],
             [[4.3348172151444970e+02, 0.000000e+00, 6.1183800398453002e+02],
              [0.000000e+00, 4.3242708204158180e+02, 3.8254277456706859e+02],
              [0.000000e+00, 0.000000e+00, 1.000000e+00]],
             [[4.3498150431641488e+02, 0.000000e+00, 6.0874976734530969e+02],
              [0.000000e+00, 4.3434032708470619e+02, 3.1742539643161552e+02],
              [0.000000e+00, 0.000000e+00, 1.000000e+00]],
             ]
R_lu2k = np.array([[0, -1, 0, 0],
                   [0, 0, -1, 0],
                   [1, 0, 0, 0],
                   [0, 0, 0, 1]])
HEADER = '''\
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4 
TYPE F F F F 
COUNT 1 1 1 1 
WIDTH {}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {}
DATA ascii
'''

class_name = {1: "Pedestrian", 2: "Bicycle", 3: "Car", 4: "Motorcycle", 5: "Bus", 6: "Truck", 7: "MotorcyleRider"}
img_path = ["front", "right", "left"]
# cam_name = {0: "front", 1: "left", 2: "right"}
cam_name = {0: "front", 1: "right", 2: "left"}


def write_pcd(gen, save_pcd_path):
    # print(save_pcd_path)
    points = []
    for p in gen:
        points.append(p)
    n = len(points)

    lines = []
    # print("n",n)
    for i in range(n):
        x, y, z, i, is_g, data_time = points[i]
        lines.append('{:.6f} {:.6f} {:.6f} {}'.format( \
            x, y, z, i))
    # print(save_pcd_path)
    with open(save_pcd_path, 'w') as f:
        f.write(HEADER.format(n, n))
        f.write('\n'.join(lines))

def save_camera(imgs, frame_id):
    for i in range(3):
        img = bridge.imgmsg_to_cv2(imgs[i], desired_encoding='bgr8')
        cv2.imwrite("./roadside/{}/camera/{}/{}.png".format(bag_name,img_path[i], frame_id), img)
        # cv2.imshow("f{}".format(i),img)
        # cv2.waitKey(1)

def save_label(objs_fuse, frame_str):
    js = []
    for obj_fuse in objs_fuse:
        x, y, z, w, h, l, r, track_id, type_id = obj_fuse.x, obj_fuse.y, obj_fuse.z, obj_fuse.width, obj_fuse.height, obj_fuse.length, obj_fuse.orientation, int(
            obj_fuse.vz), obj_fuse.id
        z -= 3.6
        # print("x,y,z,r,track_id,type_id",x,y,z,r,track_id,type_id)
        if type_id < 1 or type_id > 7:
            continue
        j = {
            "obj_id": str(track_id),
            "obj_type": class_name[type_id],
            "psr": {
                "position": {
                    "x": x,
                    "y": y,
                    "z": z
                },
                "rotation": {
                    "x": 0,
                    "y": 0,
                    "z": r
                },
                "scale": {
                    "x": l,
                    "y": w,
                    "z": h
                }
            }
        }
        js.append(j)
    # print('./roadside/{}/label/{}.json'.format(bag_name,frame_str))
    with open('./roadside/{}/label/{}.json'.format(bag_name,frame_str), 'w') as f:
        json.dump(js, f)

def campare_cam2point(imgs_list,points_list,labels_list=None):
    img_point = []
    label = None
    # import pdb
    # pdb.set_trace()
    for idx,point_time in enumerate(points_list):
        time_gap_min = 100
        point = point_time[0]
        for img_time in imgs_list:
            if abs((img_time[1]-0.2) - point_time[1])<time_gap_min:
                time_gap_min = abs(img_time[1]-0.2 - point_time[1])
                img = img_time[0]
        if time_gap_min<0.1:
            if len(labels_list)==0:
                img_point.append([point,img,None])
            else:
                img_point.append([point,img,labels_list[idx][0]])
    return img_point


def save_rosdata(bag_file,bag_name):
    bag = rosbag.Bag(bag_file)
    info = bag.get_type_and_topic_info()
    print(info)
    point_count = info.topics["/points_raw"].message_count
    bag_data = bag.read_messages(['/msg_obj_fuse', '/points_raw', '/n_camera_hive_detect'])


    imgs = None
    label_fuse = None

    imgs_list = []
    points_list = []
    labels_list = []
    # import pdb
    # pdb.set_trace()
    # lidar_num = info.topics["/points_raw"].message_count
    for idx,(topic, msg, t) in enumerate(bag_data):
        if topic == "/n_camera_hive_detect":
            imgs = msg.raw_image
            img_time = msg.header.stamp.secs+msg.header.stamp.nsecs/1000000000.0
            imgs_list.append([imgs,img_time])

        if topic == "/points_raw":
            lidar_time = msg.header.stamp.secs + msg.header.stamp.nsecs / 1000000000.0
            gen = point_cloud2.read_points(msg)
            points_list.append([gen,lidar_time])
        #     labels_list.append([label_fuse, label_time])
        #
        # if topic == "/msg_obj_fuse":
        #     label_fuse = msg.obj_fuse
        #     label_time = msg.time.sec + msg.time.msec / 100000000

    img_point = campare_cam2point(imgs_list,points_list,labels_list)
    num = len(img_point)
    frame_id = 0
    for [gen,imgs,label] in (img_point):
        frame_str = str(frame_id).zfill(6)
        write_pcd(gen, "./roadside/{}/lidar/{}.pcd".format(bag_name, frame_str))
        save_camera(imgs, frame_str)
        if not label is None:
            save_label(label, frame_str)
        frame_id += 1
        print("===={}/{}=====".format(frame_id,num-1))



def save_calib(bag_name):
    for cam_num in range(3):
        c = extrinsic[cam_num]
        P = np.array(c)

        p = np.matmul(np.array([[1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, 3.6],
                                [0, 0, 0, 1]]), P)


        c = np.dot(R_lu2k, p).tolist()
        j = {
                "extrinsic": c[0]+c[1]+c[2]+c[3],
                "intrinsic": intrinsic[cam_num][0][:3]+intrinsic[cam_num][1][:3]+intrinsic[cam_num][2][:3],
                }
        with open('./roadside/{}/calib/camera/{}.json'.format(bag_name,cam_name[cam_num]), 'w') as f:
            json.dump(j,f)

def mkdir(path):
    if not os.path.exists(path):
        os.mkdir(path)

def mkdir_init(bag_name,force=False):
    if force:
        os.system("rm -rf ./roadside/{}/camera/ ./roadside/{}/lidar/".format(bag_name,bag_name))
    mkdir("./roadside/{}".format(bag_name))

    mkdir("./roadside/{}/calib/".format(bag_name))
    mkdir("./roadside/{}/calib/camera".format(bag_name))

    mkdir("./roadside/{}/label/".format(bag_name))
    mkdir("./roadside/{}/lidar/".format(bag_name))

    mkdir("./roadside/{}/camera/".format(bag_name))
    mkdir("./roadside/{}/camera/right".format(bag_name))
    mkdir("./roadside/{}/camera/front".format(bag_name))
    mkdir("./roadside/{}/camera/left".format(bag_name))


args = argparse.ArgumentParser()
args.add_argument("--bag",default=None, type=str)
# args.add_argument("--bags",default=None, type=str)
args.add_argument("--force", action='store_true', default=False,)
arg = args.parse_args()

if not os.path.isdir(arg.bag):
    print("========get one bag========")
    bag_file = arg.bag
    bag_name = bag_file.split("/")[-1].split(".")[0]
    print("=============START=============", bag_name)
    mkdir_init(bag_name,arg.force)
    save_calib(bag_name)
    save_rosdata(bag_file,bag_name)
    print("=============OVER=============", bag_name)
else:
    print("========get bags========")
    bags_path = os.listdir(arg.bag)
    bags_num = len(bags_path)
    for bag_idx,bag_path in enumerate(bags_path):
        bag_file = os.path.join(arg.bag,bag_path)
        [bag_name,file_type] = bag_file.split("/")[-1].split(".")
        if file_type == "bag":
            if not arg.force:
                if os.path.exists("./roadside/"+bag_name):
                    print("========{} is exist, pass=====".format(bag_name))
                    continue
            print("=============START {}====={}/{}========".format(bag_name,bag_idx,bags_num))
            mkdir_init(bag_name,arg.force)
            save_calib(bag_name)
            save_rosdata(bag_file,bag_name)
            print("=============OVER {}======{}/{}=======".format(bag_name,bag_idx,bags_num))

os.system("chmod 777 /disk2/fuyu/roadside_data/ -R")
