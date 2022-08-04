# Learning_localization_from_scratch_ws
## evo(kitti/tum)
```
mkdir src/data
cd src/kaist_tool/kaist2bag/scripts/
pip3 install -r requirements.txt
python3 kaist2evo.py
roslaunch lio_sam run.launch 
```

```
cd src/data
evo_traj tum tum_lio_sam_pose.txt --ref=tum_ground_truth.txt -a -p
```

