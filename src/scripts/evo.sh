#!/bin/bash

# 命令 格式 参考轨迹 估计轨迹 [可选项]
# evo_rpe tum 文件1.txt 文件2.txt -r trans_part --plot --plot_mode xy --save_results results/保存的结果文件夹的名字.zip
# evo_rpe tum 文件1.txt 文件2.txt -r trans_part --plot --plot_mode xy --save_results results/保存的结果文件夹的名字.zip

txt1="/home/trunk/0-ws/Learning_localization_from_scratch_ws/src/doc/ground_truth/tum_ground_truth.txt"
txt2="/home/trunk/0-ws/Learning_localization_from_scratch_ws/src/doc/mapping_results/tum_lio_sam_pose.txt"

evo_ape tum $txt1 $txt2 -r trans_part --plot --plot_mode xy -a &
evo_rpe tum $txt1 $txt2 -r trans_part --plot --plot_mode xy -a &
evo_traj tum $txt2 --ref=$txt1 -a --plot --plot_mode xy

