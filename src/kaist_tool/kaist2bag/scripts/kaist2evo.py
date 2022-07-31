#!/usr/bin/env python2

import os
import csv
from scipy.spatial.transform import Rotation as R

if __name__ == "__main__":
    src_path = os.path.abspath(os.path.join(os.getcwd(), "../../.."))
    gps_file = os.path.join(src_path,"data/urban08/sensor_data/vrs_gps.csv")
    imu_file = os.path.join(src_path,"data/urban08/sensor_data/xsens_imu.csv")
    kitti_ground_truth = os.path.join(src_path,"data/kitti_ground_truth.txt")
    tum_ground_truth = os.path.join(src_path,"data/tum_ground_truth.txt")
    with open(gps_file,'r') as f1,open(imu_file,'r') as f2,open(kitti_ground_truth, 'w') as f3,open(tum_ground_truth, 'w') as f4:
        f1 = list(csv.reader(f1))
        f2 = list(csv.reader(f2))
        i = 0
        j = 0
        while i < len(f1) and j < len(f2):
            i = int(j/103)
            x = float(f1[i][3])
            y = float(f1[i][4])
            z = float(f1[i][5])
            q_x = float(f2[j][1])
            q_y = float(f2[j][2])
            q_z = float(f2[j][3])
            q_w = float(f2[j][4])
            r = R.from_quat([q_x,q_y, q_z,q_w])
            rotation_matrix = r.as_matrix()
            kitti_line = str(rotation_matrix[0][0]) + " " + str(rotation_matrix[0][1]) + " " +str(rotation_matrix[0][2]) + " " +str(x-float(f1[0][3])) \
            +" " + str(rotation_matrix[1][0]) + " " + str(rotation_matrix[1][1]) + " " + str(rotation_matrix[1][2]) + " " + str(y-float(f1[0][4]))\
            +" " + str(rotation_matrix[2][0]) + " " + str(rotation_matrix[2][1]) + " " + str(rotation_matrix[2][2]) + " " + str(z-float(f1[0][5]))
            f3.write(kitti_line + "\n")
            tum_line = str(float(f2[j][0])*0.000000001) + " " + str(x-float(f1[0][3])) + " " +str(y-float(f1[0][4])) + " " + str(z-float(f1[0][5])) + " " \
            + str(q_x) + " " + str(q_y) + " " +str(q_z) + " " + str(q_w)
            f4.write(tum_line + "\n")
            j+=1
            
        

    

        