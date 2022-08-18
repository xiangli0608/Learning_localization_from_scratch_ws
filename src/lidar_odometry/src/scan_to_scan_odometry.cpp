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

#include "scan_to_scan_odometry.h"


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


//curr_point_a_为这一帧中的点a，last_point_b_为点a旋转后和上一帧里最近的点
//last_point_c_为点b同线或上线号的点，last_point_d_为点b下线号的点
//b，c，d与a点距离不超过3m
//surface_norm为根据向量bc和bd求出的法向量
struct LidarSurfaceFactor
{
    LidarSurfaceFactor(Eigen::Vector3d curr_point_a_, Eigen::Vector3d last_point_b_,
		     Eigen::Vector3d last_point_c_, Eigen::Vector3d last_point_d_):
		     curr_point_a(curr_point_a_),last_point_b(last_point_b_),
		     last_point_c(last_point_c_),last_point_d(last_point_d_)
    {
        surface_norm = (last_point_d - last_point_b).cross(last_point_c - last_point_b);
        surface_norm.normalize();
    }

    template <typename T>
    //surface_norm点乘向量ab为a点距面bcd的距离，即残差
    bool operator()(const T* q,const T* t,T* residual)const
    {
        Eigen::Matrix<T, 3, 1> p_a_curr{T(curr_point_a.x()), T(curr_point_a.y()), T(curr_point_a.z())};
        Eigen::Matrix<T, 3, 1> p_b_last{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};
        Eigen::Matrix<T, 3, 1> bcd{T(surface_norm.x()), T(surface_norm.y()), T(surface_norm.z())};

        Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};

        // 当前帧的点a转换到上一帧
        Eigen::Matrix<T, 3, 1> p_a_last;
        p_a_last = q_last_curr * p_a_curr + t_last_curr;

        residual[0] = (p_a_last - p_b_last).dot(bcd);
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_a_, const Eigen::Vector3d last_point_b_, 
                                       const Eigen::Vector3d last_point_c_, const Eigen::Vector3d last_point_d_)
    {
        return (new ceres::AutoDiffCostFunction<LidarSurfaceFactor, 1, 4, 3>(
            new LidarSurfaceFactor(curr_point_a_, last_point_b_, last_point_c_, last_point_d_)));
    }


    Eigen::Vector3d curr_point_a, last_point_b, last_point_c, last_point_d;
    Eigen::Vector3d surface_norm;
};



//当前帧到世界坐标系
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);

//当前帧到上一帧
double para_q[4] = {0, 0, 0, 1};
double para_t[3] = {0, 0, 0};
//四元数Q，这帧到上帧
Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
//这帧到上帧位移量t，配合四元数累加在一起就是当前帧到最开始帧也就是世界坐标系
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);
//后面要进行距离比较的参数,实际为两点相距3m为阈值
constexpr double DISTANCE_SQ_THRESHOLD = 9;
//找点进行匹配优化时的线数距离(13线-10线>2.5就break介样用)
constexpr double NEARBY_SCAN = 2.5;


class ScanToScanOdometry
{
public:
    ros::NodeHandle nh;

    // 上一帧surface点
    pcl::PointCloud<PointXYZIR> laserCloudSurfLast;

    // 当前帧surface点
    pcl::PointCloud<PointXYZIR> laserCloudSurfCurr;


    // pcl::VoxelGrid<PointXYZIR> downSizeFilterSurf;

    int laserCloudSurfLastNum = 0;
    int laserCloudSurfCurrNum = 0;

    pcl::KdTreeFLANN<PointXYZIR> kdtreeSurfLast;

    double timeLaserInfoCur;

    ros::Subscriber cloudSurface;

    //输出里程计，路径，当前全部点，地图点
    ros::Publisher pubLaserOdometry;
    ros::Publisher pubLaserPath;
    ros::Publisher pubLaserOdomCloud;
    //定义路径，用于保存帧的位置，发布于pubLaserPath
    nav_msgs::Path laserPath;


    std::queue<sensor_msgs::PointCloud2ConstPtr> surfaceBuf;
    std::mutex veloLock;

public:
    ScanToScanOdometry()
    {
        
        cloudSurface = nh.subscribe("/feature/cloud_surface", 100, &ScanToScanOdometry::cloudSurfaceHandler, this, ros::TransportHints().tcpNoDelay());

        //pubLaserOdometry包括当前帧四元数Q和位置t,pubLaserPath包含当前帧的位置t
        pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
        pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);

        // 发布当前帧特征点云(odom坐标系下)
        pubLaserOdomCloud = nh.advertise<sensor_msgs::PointCloud2>("/laser_odom_cloud", 100);

    }


    void cloudSurfaceHandler(const sensor_msgs::PointCloud2Ptr &cloud_surface)
    {
        veloLock.lock();
        surfaceBuf.push(cloud_surface);
        veloLock.unlock();
    }

    void odometryThread()
    {

        ros::Rate rate(10);
        while (ros::ok())
        {
            rate.sleep();

            if(!surfaceBuf.empty())
            {
                
                timeLaserInfoCur = surfaceBuf.front()->header.stamp.toSec();

                veloLock.lock();
                pcl::fromROSMsg(*surfaceBuf.front(), laserCloudSurfCurr);
                surfaceBuf.pop();
                veloLock.unlock();


                // 构建优化问题
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();

                ceres::Problem::Options problem_options;
                ceres::Problem problem(problem_options);

                // 添加参数块
                problem.AddParameterBlock(para_q, 4, q_parameterization);
                problem.AddParameterBlock(para_t, 3);

        
                laserCloudSurfLastNum = laserCloudSurfLast.points.size();
                laserCloudSurfCurrNum = laserCloudSurfCurr.points.size();

                // std::cout << laserCloudSurfLastNum << std::endl;
                // std::cout << laserCloudSurfCurrNum << std::endl;

        
                if(laserCloudSurfLastNum < 10)
                {
                    laserCloudSurfLast = laserCloudSurfCurr;
                }
                else
                {
                    kdtreeSurfLast.setInputCloud(laserCloudSurfLast.makeShared());

                    for(int iterNum = 0; iterNum < 2; iterNum++)
                    {
                        // 对于这一帧每个点a，寻找上一帧最近点b，并根据b寻找c、d进行点面距离计算，引入优化问题
                        for(int i = 0; i < laserCloudSurfCurrNum; i++)
                        {
                            PointXYZIR pointOri, pointSel;
                            std::vector<int> pointSearchInd;
                            std::vector<float> pointSearchSqDis;

                            pointOri = laserCloudSurfCurr.points[i];
                            transformToStart(&pointOri, &pointSel);
                            kdtreeSurfLast.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                            // b,c,d点的id
                            int closestPointInd = pointSearchInd[0];
                            int minPointInd2 = -1, minPointInd3 = -1;
                            int closestPointScanID = laserCloudSurfLast.points[pointSearchInd[0]].ring;

                            if(pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                            {
                                // 在b的线号向上寻找c点
                                for(int j = closestPointInd + 2; j < laserCloudSurfLastNum && minPointInd2 == -1; j++)
                                {
                                    // 不是相邻scan，跳过
                                    if(laserCloudSurfLast.points[j].ring > closestPointScanID + NEARBY_SCAN)
                                    {
                                        continue;
                                    }
                                    else
                                    {
                                        double distance_a_c   = (laserCloudSurfLast.points[j].x - pointSel.x) * 
                                                                (laserCloudSurfLast.points[j].x - pointSel.x) + 
                                                                (laserCloudSurfLast.points[j].y - pointSel.y) * 
                                                                (laserCloudSurfLast.points[j].y - pointSel.y) +
                                                                (laserCloudSurfLast.points[j].z - pointSel.z) *
                                                                (laserCloudSurfLast.points[j].z - pointSel.z);

                                        if(distance_a_c > DISTANCE_SQ_THRESHOLD)
                                        {
                                            continue;
                                        }
                                        else
                                        {
                                            minPointInd2 = j;
                                        }
                                    }
                                }

                                // 在b的线号向下寻找d点
                                for(int j = closestPointInd - 1; j > 0 && minPointInd3 == -1; j--)
                                {
                                    if(laserCloudSurfLast.points[j].ring < (closestPointScanID - NEARBY_SCAN) || (laserCloudSurfLast.points[j].ring == closestPointScanID))
                                    {
                                        continue;
                                    }
                                    else
                                    {
                                        double distance_a_d  =  (laserCloudSurfLast.points[j].x - pointSel.x) * 
                                                                (laserCloudSurfLast.points[j].x - pointSel.x) + 
                                                                (laserCloudSurfLast.points[j].y - pointSel.y) * 
                                                                (laserCloudSurfLast.points[j].y - pointSel.y) +
                                                                (laserCloudSurfLast.points[j].z - pointSel.z) *
                                                                (laserCloudSurfLast.points[j].z - pointSel.z);

                                        if(distance_a_d > DISTANCE_SQ_THRESHOLD)
                                        {
                                            continue;
                                        }
                                        else
                                        {
                                            minPointInd3 = j;
                                        }
                                    }                                
                                }

                                // 如果c，d都找到
                                if(minPointInd2 == -1 || minPointInd3 == -1)
                                {
                                    continue;
                                }
                                else
                                {
                                    Eigen::Vector3d curr_point_a(laserCloudSurfCurr.points[i].x,
                                                                 laserCloudSurfCurr.points[i].y,
                                                                 laserCloudSurfCurr.points[i].z);
                                    
                                    Eigen::Vector3d last_point_b(laserCloudSurfLast.points[closestPointInd].x,
                                                                 laserCloudSurfLast.points[closestPointInd].y,
                                                                 laserCloudSurfLast.points[closestPointInd].z);

                                    Eigen::Vector3d last_point_c(laserCloudSurfLast.points[minPointInd2].x,
                                                                 laserCloudSurfLast.points[minPointInd2].y,
                                                                 laserCloudSurfLast.points[minPointInd2].z);

                                    Eigen::Vector3d last_point_d(laserCloudSurfLast.points[minPointInd3].x,
                                                                 laserCloudSurfLast.points[minPointInd3].y,
                                                                 laserCloudSurfLast.points[minPointInd3].z);

                                    ceres::CostFunction *cost_function = LidarSurfaceFactor::Create(curr_point_a, last_point_b, last_point_c, last_point_d);
                                    problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                                }
                            }
                        }  
                    }
                }

                // 当前帧点a遍历后，进行优化
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 5;
                options.minimizer_progress_to_stdout = false;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);

                // 优化后，位姿累积，当前帧变前帧
                laserCloudSurfLast = laserCloudSurfCurr;
                t_w_curr = t_w_curr + q_w_curr * t_last_curr;
                q_w_curr = q_w_curr * q_last_curr;


                // 输出轨迹和path
                nav_msgs::Odometry laserOdometry;
                laserOdometry.header.frame_id = "/map";
                laserOdometry.child_frame_id = "/base_link";
                laserOdometry.header.stamp = ros::Time().fromSec(timeLaserInfoCur);
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


                // 发布当前帧特征点云
                pcl::PointCloud<PointXYZIR>::Ptr laserOdomCloud(new pcl::PointCloud<PointXYZIR>());
                for(int i = 0; i < laserCloudSurfCurrNum; i++)
                {
                    PointXYZIR point_in_map;
                    transformToStart(&laserCloudSurfCurr.points[i], &point_in_map);
                    laserOdomCloud->push_back(point_in_map);
                }

                sensor_msgs::PointCloud2 laserCloudMapMsg;
                pcl::toROSMsg(*laserOdomCloud, laserCloudMapMsg);
                laserCloudMapMsg.header.stamp = ros::Time().fromSec(timeLaserInfoCur);
                laserCloudMapMsg.header.frame_id = "/map";
                pubLaserOdomCloud.publish(laserCloudMapMsg);
            }
        }
    }

    // 当前帧的点转换到上一帧lidar坐标系
    void transformToStart(PointXYZIR const *const pi, PointXYZIR *const po)
    {
        Eigen::Vector3d point_in(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_out;

        point_out = q_last_curr * point_in + t_last_curr;
        po->x = point_out.x();
        po->y = point_out.y();
        po->z = point_out.z();
        po->intensity = pi->intensity;
        po->ring = pi->ring;
    }

};


int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "scan_to_scan_odometry");
    
    ScanToScanOdometry STSO;

    ROS_INFO("\033[1;32m----> Scan To Scan Odometry Started.\033[0m");

    // 主线程
    std::thread mainThread(&ScanToScanOdometry::odometryThread, &STSO);

    ros::spin();

    mainThread.join();

    return 0;
}