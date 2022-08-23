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
    global_pose_file = os.path.join(urbag_path, "global_pose.csv")
    
    kitti_ground_truth = os.path.join(output_path, "kitti_ground_truth.txt")
    tum_ground_truth = os.path.join(output_path, "tum_ground_truth.txt")
    kitti_gps_truth = os.path.join(output_path, "kitti_vrs_gps.txt")
    
    with open(gps_file,'r') as f1, open(global_pose_file,'r') as f2, open(kitti_ground_truth, 'w') as f3, open(tum_ground_truth, 'w') as f4, open(kitti_gps_truth, 'w') as f5:
        f1 = list(csv.reader(f1))
        f2 = list(csv.reader(f2))
        i = 0
        j = 0
        while i < len(f1) and j < len(f2):
            t1 = float(f1[i][0])
            t2 = float(f2[j][0])
            # 以下是从/sensor_data/vrs_gps.csv读取的位置数据
            x1 = float(f1[i][3])
            y1 = float(f1[i][4])
            z1 = float(f1[i][5])

            # 以下是从/global_pose.csv读取的位姿数据
            x2 = float(f2[j][4])
            y2 = float(f2[j][8])
            z2 = float(f2[j][12])

            rotation_matrix = np.array([float(f2[j][1]), float(f2[j][2]),float(f2[j][3]),
                                        float(f2[j][5]), float(f2[j][6]), float(f2[j][7]),
                                        float(f2[j][9]), float(f2[j][10]), float(f2[j][11])])
            rotation_matrix = rotation_matrix.reshape(3,3)
            R_quat = R.from_matrix(rotation_matrix).as_quat()
            q_x = R_quat[0]
            q_y = R_quat[1]
            q_z = R_quat[2]
            q_w = R_quat[3]

            # /global_pose.csv保存为kitti格式
            kitti_ground_truth_line =  str(f2[j][1]) + " " + str(f2[j][2]) + " "  + str(f2[j][3]) + " " +  str(x2-float(f2[0][4])) \
                                +" " + str(f2[j][5]) + " " + str(f2[j][6]) + " "  + str(f2[j][7]) + " " +  str(y2-float(f2[0][8]))\
                                +" " + str(f2[j][9]) + " " + str(f2[j][10]) + " " + str(f2[j][11]) + " " + str(z2-float(f2[0][12]))
            f3.write(kitti_ground_truth_line + "\n")

            # /global_pose.csv保存为tum格式
            tum_ground_truth_line = str(float(f2[j][0])*0.000000001) + " " + str(x2-float(f2[0][4])) + " " +str(y2-float(f2[0][8])) + " " + str(z2-float(f2[0][12])) + " " \
            + str(q_x) + " " + str(q_y) + " " +str(q_z) + " " + str(q_w)
            f4.write(tum_ground_truth_line + "\n")

            # /sensor_data/vrs_gps.csv保存为kitti格式,使用了/global_pose.csv的姿态
            kitti_gps_truth_line = str(f2[j][1]) + " " + str(f2[j][2]) + " " +str(f2[j][3]) + " " +str(x1-float(f1[0][3])) \
                            +" " + str(f2[j][5]) + " " + str(f2[j][6]) + " " + str(f2[j][7]) + " " + str(y1-float(f1[0][4]))\
                            +" " + str(f2[j][9]) + " " + str(f2[j][10]) + " " + str(f2[j][11]) + " " + str(z1-float(f1[0][5]))
            f5.write(kitti_gps_truth_line + "\n")
            
            # /sensor_data/vrs_gps.csv使用一定时间范围内的/global_pose.csv的姿态
            if abs(t2-t1) > 500000000:
                i += 1
            j += 1 
            
        

    

        