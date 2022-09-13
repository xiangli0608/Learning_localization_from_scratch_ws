#!/usr/bin/env python3
import argparse
import os
import csv
import numpy as np
from scipy.spatial.transform import Rotation as R

if __name__ == "__main__":
    # 传入urban_08文件夹的路径 python3 kaist2evo.py -p [the path of urban bag] 
    parser = argparse.ArgumentParser(description="path")
    parser.add_argument("-p", "--path", type=str, help="the path of urban bag")
    parser.add_argument("-o", "--output", default=None,type=str, help="the path of result")
    args = parser.parse_args()
    urbag_path = args.path
    output_path = args.output
    
    # 默认保存在urban_bag路径下
    if output_path is None:
        output_path = urbag_path
    print(f'urban_bag的路径为:{urbag_path}')
    print(f'保存结果的路径为:{output_path}')
    
    gps_file = os.path.join(urbag_path, "sensor_data","vrs_gps.csv")
    tum_vrs_gps = os.path.join(output_path, "tum_vrs_gps.txt")
    
    global_pose_file = os.path.join(urbag_path, "global_pose.csv")
    tum_ground_truth = os.path.join(output_path, "tum_ground_truth.txt")
    
    # 将vrs_gps.csv数据写成tum格式的轨迹
    with open(gps_file, 'r') as f1, open(tum_vrs_gps, 'w') as f2:
        f1 = list(csv.reader(f1))
        for gps_line in f1:
            # print(gps_line)
            # sensor_data/vrs_gps.csv保存为tum格式, 姿态写成0.0
            tum_var_gps_line = str(float(gps_line[0]) * 0.000000001) + " " \
                + str(float(gps_line[3])) + " " + str(float(gps_line[4])) + " " + str(float(gps_line[5])) + " " \
                + str(0.0) + " " + str(0.0) + " " +str(0.0) + " " + str(0.0)
            f2.write(tum_var_gps_line + "\n")
    
    # 将global_pose.csv写成tum格式的轨迹
    with open(global_pose_file, 'r') as f1, open(tum_ground_truth, 'w') as f2:
        f1 = list(csv.reader(f1))
        for global_pose_line in f1:
            # print(global_pose_line)
            # 以下是从/global_pose.csv读取的位姿数据
            x = float(global_pose_line[4])
            y = float(global_pose_line[8])
            z = float(global_pose_line[12])

            rotation_matrix = np.array([float(global_pose_line[1]), float(global_pose_line[2]),float(global_pose_line[3]),
                                        float(global_pose_line[5]), float(global_pose_line[6]), float(global_pose_line[7]),
                                        float(global_pose_line[9]), float(global_pose_line[10]), float(global_pose_line[11])])
            rotation_matrix = rotation_matrix.reshape(3,3)
            R_quat = R.from_matrix(rotation_matrix).as_quat()
            q_x = R_quat[0]
            q_y = R_quat[1]
            q_z = R_quat[2]
            q_w = R_quat[3]

            # 将global_pose.csv保存为tum格式
            tum_ground_truth_line = str(float(global_pose_line[0]) * 0.000000001) + " " \
                + str(float(x)) + " " + str(float(y)) + " " + str(float(z)) + " " \
                + str(q_x) + " " + str(q_y) + " " +str(q_z) + " " + str(q_w)
            f2.write(tum_ground_truth_line + "\n")