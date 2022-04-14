import os
import math
import json
from tqdm import tqdm
class_num_limit={"Car":50,"Bus":100,"MotorcyleRider":30,"Pedestrian":20}

def check_point_in_box(pts, box):
    """
    pts[x,y,z]
    box[c_x,c_y,c_z,dx,dy,dz,heading]
    """
    shift_x = pts[0] - box[0]
    shift_y = pts[1] - box[1]
    shift_z = pts[2] - box[2]
    cos_a = math.cos(box[6])
    sin_a = math.sin(box[6])
    dx,dy,dz = box[3], box[4], box[5]
    local_x = shift_x * cos_a + shift_y * sin_a
    local_y = shift_y * cos_a - shift_x * sin_a
    if abs(shift_z)>dz/2.0 or abs(local_x)>dx/2.0 or abs(local_y)>dy/2.0:
        return False
    return True

def loadpcd(pcd_path):
    with open(pcd_path) as f:
        points = [[ float('-inf' if xyz=="nan" else xyz) for xyz in point.split("\n")[0].split(" ")] for point in f.readlines()[11:]]

    return points

def loadlabel(label_path):
    with open(label_path) as f:
        labels = json.load(f)
        objs = []
        for label in labels:
            obj = {}
            position = label['psr']['position']
            rotation = label['psr']['rotation']['z']
            scale = label['psr']['scale']
            x, y, z = position["x"], position["y"], position["z"]
            xs, ys, zs = scale["x"], scale["y"], scale["z"]
            obj["obj_type"],obj["obj_id"]  = label['obj_type'],label['obj_id']
            obj["box"] = [x, y, z, xs, ys, zs, rotation]
            objs.append(obj)
    return objs

def count_num(points,objs):
    for point in points:
        for obj in objs:
            flag = check_point_in_box(point[:3], obj["box"])
            if flag:
                obj["num"] = obj.get("num", 0) + 1

def get_remove_objs_id(pcd_path,label_path):
    points = loadpcd(pcd_path)
    objs = loadlabel(label_path)

    count_num(points, objs)
    remove_objs_id = []
    for obj in objs:
        # print(pcd_path,obj["obj_id"], obj["obj_type"], obj["num"])
        if obj["num"] < class_num_limit[obj["obj_type"]]:
            remove_objs_id.append(obj["obj_id"])
    return remove_objs_id

def mkdir(path):
    if not os.path.exists(path):
        os.mkdir(path)

def remove_save_label(remove_objs_id,label_path):
    label_filter_path = label_path.replace("label", "label_filter")
    mkdir(os.path.dirname(label_filter_path))
    with open(label_path,"r") as f:
        labels = json.load(f)
        new_labels = []
        for label in labels:
            obj_id = label['obj_id']
            if not obj_id in remove_objs_id:
                new_labels.append(label)
    with open(label_filter_path,"w") as f:
        json.dump(new_labels,f,indent=1)



def main():
    dataname = "./roadside/"
    bags_name = os.listdir(dataname)

    for bag_idx,bag_n in enumerate(bags_name):
        bag_dir = os.path.join(dataname, bag_n)
        lidar_dir = "{}/lidar/".format(bag_dir)
        lidar_files = os.listdir(lidar_dir)
        bar = tqdm(lidar_files)
        # if bag_idx < 2:
        #     continue
        for lidar_file in bar:
            pcd_path = os.path.join(lidar_dir, lidar_file)
            label_path = pcd_path.replace("/lidar/", "/label/").replace(".pcd", ".json")
            remove_objs_id = get_remove_objs_id(pcd_path, label_path)
            remove_save_label(remove_objs_id, label_path)
            bar.set_description("bag/all {}/{}".format(bag_idx+1,len(bags_name)))


if __name__ == '__main__':
    main()
