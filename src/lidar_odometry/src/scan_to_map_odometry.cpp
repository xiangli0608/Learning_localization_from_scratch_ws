#include <cmath>
#include <vector>
#include <string>
#include <time.h>
#include<iostream>
#include <queue>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#define PCL_NO_PRECOMPILE

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include<ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>

#include "scan_to_map_odometry.h"


struct VelodynePointXYZIR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIR,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring)
)

using PointXYZIR = VelodynePointXYZIR;


//curr_point_a_为当前帧点a，地图平面中心点map_point_center和法向量map_norm

struct LidarSurfaceNormFactor
{
    LidarSurfaceNormFactor(Eigen::Vector3d curr_point_a_, 
		     Eigen::Vector3d surface_point_center_, Eigen::Vector3d surface_norm_):
		     curr_point_a(curr_point_a_),surface_point_center(surface_point_center_),
		     surface_norm(surface_norm_){}

    template <typename T>
    bool operator()(const T* q,const T* t,T* residual)const
    {
        Eigen::Matrix<T, 3, 1> p_a_curr{T(curr_point_a.x()), T(curr_point_a.y()), T(curr_point_a.z())};
        Eigen::Matrix<T, 3, 1> p_b_last{T(surface_point_center.x()), T(surface_point_center.y()), T(surface_point_center.z())};
        Eigen::Matrix<T, 3, 1> p_norm{T(surface_norm.x()), T(surface_norm.y()), T(surface_norm.z())};

        Eigen::Quaternion<T> rot_q{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> rot_t{t[0], t[1], t[2]};

        // 当前帧的点转换到世界坐标系
        Eigen::Matrix<T, 3, 1> p_a_last;
        p_a_last = rot_q * p_a_curr + rot_t;

        residual[0] = ((p_a_last - p_b_last).dot(p_norm));
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_a_, const Eigen::Vector3d surface_point_center_, 
                                       const Eigen::Vector3d surface_norm_)
    {
        return (new ceres::AutoDiffCostFunction<LidarSurfaceNormFactor, 1, 4, 3>(
            new LidarSurfaceNormFactor(curr_point_a_, surface_point_center_, surface_norm_)));
    }


    const Eigen::Vector3d curr_point_a, surface_point_center, surface_norm;
};


double parameters[7] = {0, 0, 0, 1, 0, 0, 0};

//当前帧到世界坐标系(map)
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters+4);


// odom坐标系到map坐标系
Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0);
Eigen::Vector3d t_wmap_wodom(0, 0, 0);

// 当前帧到odom坐标系
Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0);
Eigen::Vector3d t_wodom_curr(0, 0, 0);



class ScanToMapOdometry
{
public:
    ros::NodeHandle nh;

    // 当前帧surface点
    pcl::PointCloud<PointXYZIR>::Ptr laserCloudSurf;

    // 点云地图
    pcl::PointCloud<PointXYZIR>::Ptr laserCloudMap;


    // 地图降采样
    pcl::VoxelGrid<PointXYZIR> downSizeFilterMap;

    int mapFilterNum = 0;

    pcl::KdTreeFLANN<PointXYZIR> kdtreeSurf;

    double timeLaserCloud;
    double timeLaserOdometry;

    ros::Subscriber cloudOdomSurface;
    ros::Subscriber laserLocalOdom;

    ros::Publisher pubLaserOdometry;
    ros::Publisher pubLaserPath;
    // ros::Publisher pubLaserMap;
    //定义路径，用于保存帧的位置，发布于pubLaserPath
    nav_msgs::Path laserPath;


    std::queue<sensor_msgs::PointCloud2ConstPtr> surfaceBuf;
    std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
    std::mutex veloLock;

public:
    ScanToMapOdometry()
    {
        
        cloudOdomSurface = nh.subscribe("/laser_odom_cloud", 100, &ScanToMapOdometry::cloudOdomSurfaceHandler, this, ros::TransportHints().tcpNoDelay());
        laserLocalOdom = nh.subscribe("/laser_odom_to_init", 100, &ScanToMapOdometry::laserOdomHandler, this, ros::TransportHints().tcpNoDelay());

        // 发布map后的轨迹
        pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_map_odometry", 100);
        pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_map_path", 100);
        // pubLaserMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_map_cloud", 100);

        allowMemory();

    }

    void allowMemory()
    {
        downSizeFilterMap.setLeafSize(0.4, 0.4, 0.4);

        laserCloudSurf.reset(new pcl::PointCloud<PointXYZIR>());
        laserCloudMap.reset(new pcl::PointCloud<PointXYZIR>());
    }


    void cloudOdomSurfaceHandler(const sensor_msgs::PointCloud2Ptr &cloud_surface)
    {
        veloLock.lock();
        surfaceBuf.push(cloud_surface);
        veloLock.unlock();
    }

    void laserOdomHandler(const nav_msgs::Odometry::ConstPtr &laser_odometry)
    {
        veloLock.lock();
        odometryBuf.push(laser_odometry);
        veloLock.unlock();
    }


    void odometryThread()
    {

        ros::Rate rate(10);
        while (ros::ok())
        {
            rate.sleep();

            // std::cout << "surfaceBuf: " << surfaceBuf.size() << std::endl;
            // std::cout << "odometry: " << odometryBuf.size() << std::endl;

            while(!surfaceBuf.empty() && !odometryBuf.empty())
            {
                
                veloLock.lock();

                timeLaserCloud = surfaceBuf.front()->header.stamp.toSec();
                timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();

                if(timeLaserCloud != timeLaserOdometry)
                {
                    std::cout << "not sync..." << std::endl;
                    break;
                }

                
                laserCloudSurf->clear();
                pcl::fromROSMsg(*surfaceBuf.front(), *laserCloudSurf);
                surfaceBuf.pop();

                
                downSizeFilterMap.setInputCloud(laserCloudSurf);
                downSizeFilterMap.filter(*laserCloudSurf);

                // odom坐标系下位姿
                q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;
                q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;
                q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;
                q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;
                t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;
                t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;
                t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;
                odometryBuf.pop();
                
                veloLock.unlock();

                // 第一帧
                if(laserCloudMap->points.size() < 10)
                {
                    laserCloudMap->clear();
                    *laserCloudMap += *laserCloudSurf;
                }
                else
                {
                    // 地图降采样
                    mapFilterNum++;
                    if(mapFilterNum % 5 == 0)
                    {
                        downSizeFilterMap.setInputCloud(laserCloudMap);
                        downSizeFilterMap.filter(*laserCloudMap);
                    }

                    // 当前帧到世界坐标系(map)
                    q_w_curr = q_wmap_wodom * q_wodom_curr;
                    t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;


                    // 地图送入KD树
                    kdtreeSurf.setInputCloud(laserCloudMap);

                    int surface_num = laserCloudSurf->points.size();

                    for(int iter_num = 0; iter_num < 2; iter_num++)
                    {
                        // 构建优化问题
                        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                        ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();

                        ceres::Problem::Options problem_options;
                        ceres::Problem problem(problem_options);

                        // 添加参数块
                        problem.AddParameterBlock(parameters, 4, q_parameterization);
                        problem.AddParameterBlock(parameters+4, 3);

                        int surface_count = 0;

                        // 将当前帧点转换到世界坐标系，找世界坐标系内五个最近的点
                        for(int i = 0; i < surface_num; i++)
                        {
                            PointXYZIR pointOri, pointSel;
                            std::vector<int> pointSearchInd;
                            std::vector<float> pointSearchSqDis;

                            pointOri = laserCloudSurf->points[i];
                            transformToMap(&pointOri, &pointSel);
                            kdtreeSurf.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                            // 最远的点小于2m
                            if(pointSearchSqDis[4] < 2.0)
                            {
                                // 找这五个点的中心点和这五个点形成平面的法向量
                                Eigen::Vector3d center(0, 0, 0);

                                for(int j = 0; j < 5; j++)
                                {
                                    Eigen::Vector3d tmp_point(laserCloudMap->points[pointSearchInd[j]].x,
                                                              laserCloudMap->points[pointSearchInd[j]].y,
                                                              laserCloudMap->points[pointSearchInd[j]].z);
                                    center = center + tmp_point;
                                }

                                center = center / 5.0;
                                Eigen::Matrix<double, 5, 3> matA0;
                                Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                                for(int j = 0; j < 5; j++)
                                {
                                    matA0(j, 0) = laserCloudMap->points[pointSearchInd[j]].x;
                                    matA0(j, 1) = laserCloudMap->points[pointSearchInd[j]].y;
                                    matA0(j, 2) = laserCloudMap->points[pointSearchInd[j]].z;
                                }

                                Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                                norm.normalize();

                                //将五个点和中心点形成向量，向量与法向量求点乘，如果大于0.1那么后面就不把这组点放入优化了
                                bool surfaceValid = true;
                                for(int j = 0; j < 5; j++)
                                {
                                    Eigen::Vector3d tmp_vector(laserCloudMap->points[pointSearchInd[j]].x - center.x(),
                                                               laserCloudMap->points[pointSearchInd[j]].y - center.y(),
                                                               laserCloudMap->points[pointSearchInd[j]].z - center.z());

                                    if(fabs(norm(0) * tmp_vector.x() + norm(1) * tmp_vector.y() + norm(2) * tmp_vector.z()) > 0.1)
                                    {
                                        surfaceValid = false;
                                        break;
                                    }
                                }

                                if(surfaceValid)
                                {
                                    Eigen::Vector3d curr_point_a(laserCloudSurf->points[i].x,
                                                                 laserCloudSurf->points[i].y,
                                                                 laserCloudSurf->points[i].z);
                                    

                                    ceres::CostFunction *cost_function = LidarSurfaceNormFactor::Create(curr_point_a, center, norm);
                                    problem.AddResidualBlock(cost_function, loss_function, parameters, parameters+4);
                                    surface_count++;
                                }
                            }
                        }

                        
                        if(surface_count > 40)
                        {
                            // 当前帧点a遍历后，进行优化
                            ceres::Solver::Options options;
                            options.linear_solver_type = ceres::DENSE_QR;
                            options.max_num_iterations = 5;
                            options.minimizer_progress_to_stdout = false;
                            options.gradient_check_relative_precision = 1e-4;
                            ceres::Solver::Summary summary;
                            ceres::Solve(options, &problem, &summary);
                        }
                            
            
                        // 优化后更新map和odom之间的变换
                        q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
                        t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
                    }

                    // 保存当前点
                    for(int i = 0; i < surface_num; i++)
                    {
                        PointXYZIR pointOri, pointSel;
                        
                        pointOri = laserCloudSurf->points[i];
                        transformToMap(&pointOri, &pointSel);
                        laserCloudMap->points.push_back(pointSel);
                    }
                }

                // 输出轨迹和path
                nav_msgs::Odometry laserOdometry;
                laserOdometry.header.frame_id = "/map";
                laserOdometry.child_frame_id = "/base_link";
                laserOdometry.header.stamp = ros::Time().fromSec(timeLaserCloud);
                laserOdometry.pose.pose.orientation.x = q_w_curr.x();
                laserOdometry.pose.pose.orientation.y = q_w_curr.y();
                laserOdometry.pose.pose.orientation.z = q_w_curr.z();
                laserOdometry.pose.pose.orientation.w = q_w_curr.w();
                laserOdometry.pose.pose.position.x = t_w_curr.x();
                laserOdometry.pose.pose.position.y = t_w_curr.y();
                laserOdometry.pose.pose.position.z = t_w_curr.z();
                pubLaserOdometry.publish(laserOdometry);
                

                geometry_msgs::PoseStamped laserPose;
                laserPose.header = laserOdometry.header;
                laserPose.pose = laserOdometry.pose.pose;
                laserPath.header.stamp = laserOdometry.header.stamp;
                laserPath.poses.push_back(laserPose);
                laserPath.header.frame_id = "/map";
                pubLaserPath.publish(laserPath);


                // 输出地图显示
                // pcl::PointCloud<PointXYZIR>::Ptr laserMapCloud(new pcl::PointCloud<PointXYZIR>());
                // long map_num = laserCloudMap->points.size();
                // for(int i = 0; i < map_num; i++)
                // {
                //     laserMapCloud->points.push_back(laserCloudMap->points[i]);
                // }

                // if(mapFilterNum % 10 == 0 && mapFilterNum != 0)
                // {
                //     sensor_msgs::PointCloud2 laserCloudMapMsg;
                //     pcl::toROSMsg(*laserMapCloud, laserCloudMapMsg);
                //     laserCloudMapMsg.header.stamp = ros::Time().fromSec(timeLaserCloud);
                //     laserCloudMapMsg.header.frame_id = "/map";
                //     pubLaserMap.publish(laserCloudMapMsg);
                // }
            }
        }
    }

    // 当前帧的点转换到世界坐标系
    void transformToMap(PointXYZIR const *const pi, PointXYZIR *const po)
    {
        Eigen::Vector3d point_in(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_out;

        point_out = q_w_curr * point_in + t_w_curr;
        po->x = point_out.x();
        po->y = point_out.y();
        po->z = point_out.z();
        po->intensity = pi->intensity;
        po->ring = pi->ring;
    }

};


int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "scan_to_map_odometry");
    
    ScanToMapOdometry STMO;

    ROS_INFO("\033[1;32m----> Scan To Map Odometry Started.\033[0m");

    // 主线程
    std::thread mainThread(&ScanToMapOdometry::odometryThread, &STMO);

    ros::spin();

    mainThread.join();

    return 0;
}

