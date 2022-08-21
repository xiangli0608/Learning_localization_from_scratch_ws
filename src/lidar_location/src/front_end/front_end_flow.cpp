#include "lidar_localization/front_end/front_end_flow.hpp"

namespace lidar_localization {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {
    cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "/ns1/velodyne_points", 100000);
    cloud_sub_ptr_ns2 = std::make_shared<CloudSubscriber>(nh, "/ns2/velodyne_points", 100000);
    imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/imu/data_raw", 1000000);
    gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/vrs_gps_data", 1000000);

    cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");
    local_map_pub_ptr = std::make_shared<CloudPublisher>(nh, "local_map", 100, "/map");
    global_map_pub_ptr = std::make_shared<CloudPublisher>(nh, "global_map", 100, "/map");
    laser_odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "laser_odom", "map", "lidar", 100);
   
    front_end_ptr = std::make_shared<FrontEnd>();
    
    // 坐标系之间的转换矩阵
    vehilcle_to_ns1 << -0.515105,-0.702383,-0.491249,-0.438343,0.487008,-0.711468,0.506593,0.395882,-0.70533,0.0217062,0.708547,1.94095,0,0,0,1;
    vehilcle_to_ns2 << -0.512695,0.700506,-0.496422,-0.436669,-0.497416,-0.713622,-0.493276,-0.411346,-0.6998,-0.00597189,0.714313,1.94785,0,0,0,1;
    vehilcle_to_imu << 1,0,0,-0.07,0,1,0,0,0,0,1,1.7,0,0,0,1;

    ns2_to_ns1 = vehilcle_to_ns2.inverse() * vehilcle_to_ns1;
    ns1_to_imu = vehilcle_to_ns1.inverse() * vehilcle_to_imu;
    ns2_to_imu = vehilcle_to_ns2.inverse() * vehilcle_to_imu;
    
    front_end_pose_inited = false;
    local_map_ptr.reset(new CloudData::CLOUD());
    global_map_ptr.reset(new CloudData::CLOUD());
    current_scan_ptr.reset(new CloudData::CLOUD());

    run_time = 0.0;
    init_time = 0.0;
    time_inited = false;
    has_global_map_published = false;
}

bool FrontEndFlow::Run() {
    ReadData();
    if(!front_end_ptr->IfOfflineMap()) {
        Workflow();
    }
    else {
        WorkflowWithMap();
    }
    return true;
}

bool FrontEndFlow::SaveMap() {
    bool result = front_end_ptr->SaveMap();
    if (front_end_ptr->GetNewGlobalMap(global_map_ptr)) {
        global_map_pub_ptr->Publish(global_map_ptr);
        has_global_map_published = true;
    }
    return result;
}

bool FrontEndFlow::ReadData() {
    cloud_sub_ptr->ParseData(cloud_data_buff);
    cloud_sub_ptr_ns2->ParseData(cloud_data_buff_ns2);
    return true;
}

bool FrontEndFlow::Workflow(){
    while (cloud_data_buff.size() > 0 && cloud_data_buff_ns2.size() > 0) {
        CloudData cloud_data = cloud_data_buff.front();
        CloudData cloud_data_ns2 = cloud_data_buff_ns2.front();
        // 把ns2的点云转换到ns1下
        pcl::transformPointCloud (*(cloud_data_ns2.cloud_ptr), *(cloud_data_ns2.cloud_ptr), ns2_to_ns1);
        *(cloud_data.cloud_ptr) = *(cloud_data.cloud_ptr) +  *(cloud_data_ns2.cloud_ptr);
        
        if (!time_inited) {
            time_inited = true;
            init_time = cloud_data.time;
        } else {
            run_time = cloud_data.time - init_time;
        }

        double d_time = cloud_data.time - cloud_data_ns2.time;
        if (d_time < -0.05) {
            cloud_data_buff.pop_front();
        } else if (d_time > 0.05) {
            cloud_data_buff_ns2.pop_front();
        } else {
            cloud_data_buff.pop_front();
            cloud_data_buff_ns2.pop_front();
            // 定义初始化的位姿
            Eigen::Matrix4f odometry_matrix = Eigen::Matrix4f::Identity();
            odometry_matrix(0,3) = 0;
            odometry_matrix(1,3) = 0;
            odometry_matrix(2,3) = 0;
            Eigen::Vector3f vector(0, 1, 0);
            double angle = M_PI / 4;
            Eigen::AngleAxisf rotationVector(angle, vector.normalized());
            Eigen::Matrix3f rotationMatrix = Eigen::Matrix3f::Identity();
            rotationMatrix = rotationVector.toRotationMatrix();
            odometry_matrix.block<3,3>(0,0) = rotationMatrix;

            if (!front_end_pose_inited) {
                front_end_pose_inited = true;
                front_end_ptr->SetInitPose(odometry_matrix);
            }
            Eigen::Matrix4f laser_matrix = front_end_ptr->Update(cloud_data);
            laser_odom_pub_ptr->Publish(laser_matrix);

            front_end_ptr->GetCurrentScan(current_scan_ptr);
            cloud_pub_ptr->Publish(current_scan_ptr);
            if (front_end_ptr->GetNewLocalMap(local_map_ptr))
                local_map_pub_ptr->Publish(local_map_ptr);
            }
        }
        return true;     
}

bool FrontEndFlow::WorkflowWithMap(){
    // 带有离线地图时
    while (cloud_data_buff.size() > 0 && cloud_data_buff_ns2.size() > 0) {
        CloudData cloud_data = cloud_data_buff.front();
        CloudData cloud_data_ns2 = cloud_data_buff_ns2.front();
        // 把ns1\ns2的点云转换到imu下
        pcl::transformPointCloud (*(cloud_data.cloud_ptr), *(cloud_data.cloud_ptr), ns1_to_imu);
        pcl::transformPointCloud (*(cloud_data_ns2.cloud_ptr), *(cloud_data_ns2.cloud_ptr), ns2_to_imu);
        *(cloud_data.cloud_ptr) = *(cloud_data.cloud_ptr) +  *(cloud_data_ns2.cloud_ptr);
        
        if (!time_inited) {
            time_inited = true;
            init_time = cloud_data.time;
        } else {
            run_time = cloud_data.time - init_time;
        }

        double d_time = cloud_data.time - cloud_data_ns2.time;
        if (d_time < -0.05) {
            cloud_data_buff.pop_front();
        } else if (d_time > 0.05) {
            cloud_data_buff_ns2.pop_front();
        } else {
            cloud_data_buff.pop_front();
            cloud_data_buff_ns2.pop_front();

            if (!front_end_pose_inited) {
                front_end_pose_inited = true;
                // 定义初始化的位姿
                imu_sub_ptr->ParseData(imu_data_buff);
                gnss_sub_ptr->ParseData(gnss_data_buff);
                IMUData imu_data = imu_data_buff.front();
                GNSSData gnss_data = gnss_data_buff.front();
                gnss_data.InitOriginPosition();
                gnss_data.UpdateXYZ();
                
                Eigen::Matrix4f odometry_matrix = Eigen::Matrix4f::Identity();
                odometry_matrix(0,3) = gnss_data.local_E;
                odometry_matrix(1,3) = gnss_data.local_N;
                odometry_matrix(2,3) = gnss_data.local_U;
                odometry_matrix.block<3,3>(0,0) = imu_data.GetOrientationMatrix();


                front_end_ptr->SetInitPose(odometry_matrix);
            }
            Eigen::Matrix4f laser_matrix = front_end_ptr->UpdateWithOfflineMap(cloud_data);
            if(has_global_map_published == false) {
                    if (front_end_ptr->GetNewGlobalMap(global_map_ptr)) {
                        global_map_pub_ptr->Publish(global_map_ptr);
                        has_global_map_published = true;
                    }
            }
            has_global_map_published = true;
            laser_odom_pub_ptr->Publish(laser_matrix);

            front_end_ptr->GetCurrentScan(current_scan_ptr);
            cloud_pub_ptr->Publish(current_scan_ptr);
        }
    }
        return true;     
}
}