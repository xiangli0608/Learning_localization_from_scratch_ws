#!/usr/bin/env python2

import os
import csv
from scipy.spatial.transform import Rotation as R

if __name__ == "__main__":
    
    gps_file = "/home/touchair/kaist_urban_dataset/urban08/sensor_data/vrs_gps.csv"
    global_pose_file = "/home/touchair/kaist_urban_dataset/urban08/global_pose.csv"
    kitti_ground_truth = "/home/touchair/LIO-SAM/kitti_ground_truth.txt"
    kitti_gps_truth = "/home/touchair/LIO-SAM/kitti_vrs_gps.txt"
    
    with open(gps_file,'r') as f1, open(global_pose_file,'r') as f2, open(kitti_ground_truth, 'w') as f3, open(kitti_gps_truth, 'w') as f4:
        f1 = list(csv.reader(f1))
        f2 = list(csv.reader(f2))
        i = 0
        j = 0
        while i < len(f1) and j < len(f2):
            i = int(j/93.233766234)
            print(i)
            x1 = float(f1[i][3])
            y1 = float(f1[i][4])
            z1 = float(f1[i][5])

            x2 = float(f2[j][4])
            y2 = float(f2[j][8])
            z2 = float(f2[j][12])

            kitti_ground_truth_line =  str(f2[j][1]) + " " + str(f2[j][2]) + " "  + str(f2[j][3]) + " " +  str(x2-float(f2[0][4])) \
                                +" " + str(f2[j][5]) + " " + str(f2[j][6]) + " "  + str(f2[j][7]) + " " +  str(y2-float(f2[0][8]))\
                                +" " + str(f2[j][9]) + " " + str(f2[j][10]) + " " + str(f2[j][11]) + " " + str(z2-float(f2[0][12]))
            f3.write(kitti_ground_truth_line + "\n")

            kitti_gps_truth_line = str(f2[j][1]) + " " + str(f2[j][2]) + " " +str(f2[j][3]) + " " +str(x1-float(f1[0][3])) \
                            +" " + str(f2[j][5]) + " " + str(f2[j][6]) + " " + str(f2[j][7]) + " " + str(y1-float(f1[0][4]))\
                            +" " + str(f2[j][9]) + " " + str(f2[j][10]) + " " + str(f2[j][11]) + " " + str(z1-float(f1[0][5]))
            f4.write(kitti_gps_truth_line + "\n")


            j+=1
            
        

    

        