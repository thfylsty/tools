import os
from tqdm import tqdm

def lns(source_path,target_path):
    source_files = os.listdir(source_path)
    for f in tqdm(source_files):
        source_name = os.path.join(source_path,f)
        target_name = os.path.join(target_path,f)
        if os.path.exists(target_name):
            print("{} is ln ready, pass".format(target_name))
            print("========================")
            continue
        print(source_name)
        print("->->->->->->->->->->->")
        print(target_name)
        print("========================")
        os.system("sudo ln -s {} {}".format(source_name,target_name))

source_path = "/disk2/fuyu/roadside_data/roadside"
target_path = "/home/wcf/hive-label-service/roadSide/data"
lns(source_path,target_path)
os.system("chmod 777 /disk2/fuyu/roadside_data/ -R")