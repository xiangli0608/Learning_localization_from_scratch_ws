
/*
 * Copyright 2021 The Project Author: lixiang
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef LESSON5_LIDAR_UNDISTORTION_H_
#define LESSON5_LIDAR_UNDISTORTION_H_

#include <iostream>
#include <chrono>
#include <vector>
#include <deque>
#include <mutex>
#include <cmath>

// ros
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <irp_sen_msgs/encoder.h>

// tf
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <tf/transform_broadcaster.h>


// pcl_ros
#include <pcl_ros/point_cloud.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>


#include <Eigen/Core>

//yaml 
#include <yaml-cpp/yaml.h>

//sstream

#include <sstream>

//ceres

#include "ceres/ceres.h"

class OdomImuFlow
{

    // 使用PCL中点的数据结构 pcl::PointXYZ
    typedef pcl::PointXYZ PointT;
    // 使用PCL中点云的数据结构 pcl::PointCloud<pcl::PointXYZ>
    typedef pcl::PointCloud<PointT> PointCloudT;

private:
    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::NodeHandle private_node_;          // ros中的私有句柄
    ros::Subscriber imu_subscriber_;        // 声明一个Subscriber
    ros::Subscriber odom_subscriber_;       // 声明一个Subscriber
    ros::Subscriber kitti_vel_subscriber;   // kitt  

    //kaist encoder data
    ros::Subscriber encoder_subscriber;
 
    
    // ros::Publisher imu_odom_publisher_; // 查看imu姿积分的样子
    tf::TransformBroadcaster imu_br;

    bool use_imu_, use_odom_;

    std::deque<sensor_msgs::Imu> imu_queue_;
    std::deque<sensor_msgs::Imu> imu_sync_queue_;
    std::deque<irp_sen_msgs::encoder> encoder_queue_;
    std::deque<nav_msgs::Odometry> odom_queue_;
    std::deque<geometry_msgs::TwistStamped> vel_queue_;

    std::mutex imu_lock_;
    std::mutex odom_lock_;
    std::mutex vel_lock_;
    std::mutex encoder_lock_;


    ceres::Problem problem;
    ceres::CostFunction* cost_function;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    std::string ENCODER_TOPIC;

    int Encoder_resolution; //resolution of encoder
    double Encoder_left_wheel_diameter;
    double Encoder_right_wheel_diameter;
    double Encoder_wheel_base;

public:
    OdomImuFlow();
    ~OdomImuFlow();
    //eskf
    bool fusion2_Run();
    //imu odom直接融合
    bool fusion1_Run();
    //图优化的融合方式
    bool fusion3_Run();

    //另一种融合方式
    bool fusion5_Run();

    void ImuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg);
    void EncoderCallback(const irp_sen_msgs::encoder::ConstPtr &encoderMsg);
    
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &odometryMsg);
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &laserScanMsg);
    void vel_msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

    bool init_filter();

    bool Correctpitch();

    bool initPose();
    template <typename _T1> struct OdomImuVelResidual
    {
        OdomImuVelResidual(const Eigen::Vector3d &odom_vel_b,
        const Eigen::Vector3d &imu_vel_b):odom_vel_b_(odom_vel_b), imu_vel_b_(imu_vel_b){
        }

        template <typename _T2>
        bool operator()(const _T2*  params,  _T2* residuals)const
            {
            
            //Eigen::Matrix<_T2, 3, 3> trans_pose;
            Eigen::Matrix<_T2, 3, 1> odom_vb(_T2(odom_vel_b_.x()),_T2(odom_vel_b_.y()),_T2(odom_vel_b_.z()));
            Eigen::Matrix<_T2, 3, 1> imu_vb(_T2(imu_vel_b_.x()),_T2(imu_vel_b_.y()),_T2(imu_vel_b_.z()));                    
            Eigen::Quaternion<_T2> q_last_curr{params[3], params[0], params[1], params[2]};
            
            // trans_pose(0,0) = (params[0]);  trans_pose(0,1) = (params[1]);  trans_pose(0,2) =  (params[2]);
            // trans_pose(1,0) = (params[3]);  trans_pose(1,1) =  (params[4]);  trans_pose(1,2) =  (params[5]);
            // trans_pose(2,0) =  (params[6]);  trans_pose(2,1) =  (params[7]);  trans_pose(2,2) =  (params[8]);

            Eigen::Matrix<_T2, 3, 1> test;
            test = odom_vb - q_last_curr * imu_vb;
            residuals[0] = _T2(test(0));
            residuals[1] = _T2(test(1));
            residuals[2] = _T2(test(2));

            return true;
        }
        
        static ceres::CostFunction* Create(const Eigen::Vector3d &odom_vel_b,const Eigen::Vector3d &imu_vel_b)
        {
            return (new ceres::AutoDiffCostFunction<OdomImuVelResidual,3,4>(
                new OdomImuVelResidual(odom_vel_b, imu_vel_b)));
        }
        const Eigen::Vector3d odom_vel_b_;
        const Eigen::Vector3d imu_vel_b_;
    };

public:
    void HasData();
    bool ValidIMUData();
    bool ValidOdomData();
    bool UpdateLocalization();
    bool InitialFilter();
    void UpdateOdomEstimation(Eigen::Vector3d &linear_acc_mid, Eigen::Vector3d &angular_vel_mid);
    bool GetAngularDelta(const size_t index_curr, const size_t index_prev,
                       Eigen::Vector3d &angular_delta,
                       Eigen::Vector3d &angular_vel_mid);
    Eigen::Vector3d GetUnbiasedAngularVel(const Eigen::Vector3d &angular_vel,
                                        const Eigen::Matrix3d &R);
    
    void UpdateOrientation(const Eigen::Vector3d &angular_delta,
                         Eigen::Matrix3d &R_curr, Eigen::Matrix3d &R_prev);

    bool GetVelocityDelta(const size_t index_curr, const size_t index_prev,
                        const Eigen::Matrix3d &R_curr,
                        const Eigen::Matrix3d &R_prev, double &T,
                        Eigen::Vector3d &velocity_delta,
                        Eigen::Vector3d &linear_acc_mid);
    Eigen::Vector3d GetUnbiasedLinearAcc(const Eigen::Vector3d &linear_acc,
                                       const Eigen::Matrix3d &R);
    void UpdatePosition(const double &T, const Eigen::Vector3d &velocity_delta);

    void UpdateErrorEstimation(const double &T,
                             const Eigen::Vector3d &linear_acc_mid,
                             const Eigen::Vector3d &angular_vel_mid);

    void UpdateProcessEquation(const Eigen::Vector3d &linear_acc_mid,
                             const Eigen::Vector3d &angular_vel_mid);

    void SetProcessEquation(const Eigen::Matrix3d &C_nb,
                          const Eigen::Vector3d &f_n,
                          const Eigen::Vector3d &w_n);
    bool CorrectLocalization();
    bool CorrectLocalizationPosVel();
    bool CorrectLocalizationPosOriVel();

    bool IsCovStable(const int INDEX_OFSET, const double THRESH = 1.0e-5);

    bool IsTurning(const Eigen::Vector3d &w_b);

    bool IsRotaton(const Eigen::Vector3d &w_b);
    void ApplyMotionConstraint(void);

    void InitWithConfig();

    void ReadGroundTruth();
private:

    double imu_last_time = 0.0;
    bool has_initial = false;
    Eigen::Vector3d initial_vel = Eigen::Vector3d::Zero();
    nav_msgs::Odometry cur_odom;
    double last_odom_time = -1;
    geometry_msgs::TwistStamped cur_vel;
    //预测 估计的当前imu
    sensor_msgs::Imu cur_imu;
    
    std::deque<sensor_msgs::Imu> imu_data_buff_;
    
    Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
    Eigen::Vector3d vel_ = Eigen::Vector3d::Zero();

    Eigen::Matrix4d measur_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Vector3d measur_vel_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d accl_bias_ = Eigen::Vector3d::Zero();

    ros::Publisher odom_publisher_;
    ros::Publisher ground_truth_publisher_;

    ros::Publisher odom_measurement_publisher_;
    sensor_msgs::Imu last_imu;
    sensor_msgs::Imu cur_imu_sync_raw;

    bool odom_or_vel = false;

    std::string imu_topic_string = "";
    std::string ground_truth_path = "";    
    
    std::vector<std::vector<double>> all_ground_truth_path;
private:
    static constexpr int kDimState{15};

    static constexpr int kIndexErrorPos{0};
    static constexpr int kIndexErrorVel{3};
    static constexpr int kIndexErrorOri{6};
    static constexpr int kIndexErrorAccel{9};
    static constexpr int kIndexErrorGyro{12};

    static constexpr int kDimProcessNoise{12};

    static constexpr int kIndexNoiseAccel{0};
    static constexpr int kIndexNoiseGyro{3};
    static constexpr int kIndexNoiseBiasAccel{6};
    static constexpr int kIndexNoiseBiasGyro{9};

    // dimensions:
    static constexpr int kDimMeasurementPose{6};
    static constexpr int kDimMeasurementPoseNoise{6};

    static constexpr int kDimMeasurementPoseVel{9};
    static constexpr int kDimMeasurementPoseVelNoise{9};

    static constexpr int kDimMeasurementPosiVel{6};
    static constexpr int kDimMeasurementPosiVelNoise{6};

    static constexpr int kDimMeasurementVel{3};
    static constexpr int kDimMeasurementVelNoise{3};


    // dimensions:
    static const int DIM_STATE = 15;
    static const int DIM_PROCESS_NOISE = 6;

    static const int DIM_MEASUREMENT_POSE = 6;
    static const int DIM_MEASUREMENT_POSE_NOISE = 6;
    static const int DIM_MEASUREMENT_POSE_VEL = 9;
    static const int DIM_MEASUREMENT_POSE_VEL_NOISE = 9;
    static const int DIM_MEASUREMENT_POSI = 3;
    static const int DIM_MEASUREMENT_POSI_NOISE = 3;
    static const int DIM_MEASUREMENT_POSI_VEL = 6;
    static const int DIM_MEASUREMENT_POSI_VEL_NOISE = 6;

    // indices:
    static const int INDEX_ERROR_POS = 0;
    static const int INDEX_ERROR_VEL = 3;
    static const int INDEX_ERROR_ORI = 6;
    static const int INDEX_ERROR_GYRO = 9;
    static const int INDEX_ERROR_ACCEL = 12;


        
    // state:
    using VectorX=Eigen::Matrix<double, kDimState, 1>;
    using MatrixP=Eigen::Matrix<double, kDimState, kDimState>;

    // process equation:
    using MatrixF=Eigen::Matrix<double, kDimState, kDimState>;
    using MatrixB=Eigen::Matrix<double, kDimState, kDimProcessNoise>;
    using MatrixQ=Eigen::Matrix<double, kDimProcessNoise, kDimProcessNoise>;

    // measurement equation:
    using MatrixGPose=Eigen::Matrix<double, kDimMeasurementPose,kDimState> ;
    using MatrixCPose=Eigen::Matrix<double, kDimMeasurementPose,kDimMeasurementPoseNoise>;
    using MatrixRPose=Eigen::Matrix<double, kDimMeasurementPoseNoise,kDimMeasurementPoseNoise>;

    using MatrixGPoseVel=Eigen::Matrix<double, kDimMeasurementPoseVel,kDimState> ;
    using MatrixCPoseVel=Eigen::Matrix<double, kDimMeasurementPoseVel,kDimMeasurementPoseVelNoise>;
    using MatrixRPoseVel=Eigen::Matrix<double, kDimMeasurementPoseVelNoise,kDimMeasurementPoseVelNoise>;

    using MatrixGPosiVel=Eigen::Matrix<double, kDimMeasurementPosiVel,kDimState> ;

    
    //vel
    using MatrixGVel = Eigen::Matrix<double, kDimMeasurementVel,kDimState>;
    using MatrixCVel = Eigen::Matrix<double, kDimMeasurementVel,kDimMeasurementVelNoise>;
    using MatrixRVel = Eigen::Matrix<double, kDimMeasurementVel,kDimMeasurementVelNoise>;
    

    using MatrixCPosiVel=Eigen::Matrix<double, kDimMeasurementPosiVel,kDimMeasurementPosiVelNoise>;
    using MatrixRPosiVel=Eigen::Matrix<double, kDimMeasurementPosiVelNoise,kDimMeasurementPosiVelNoise>;

    // measurement:
    using VectorYPose=Eigen::Matrix<double, kDimMeasurementPose, 1>;
    using VectorYPoseVel=Eigen::Matrix<double, kDimMeasurementPoseVel, 1>;
    using VectorYPosiVel=Eigen::Matrix<double, kDimMeasurementPosiVel, 1>;

    using VectorYVel = Eigen::Matrix<double, kDimMeasurementVel,1>;
    // Kalman gain:
    using MatrixKPose=Eigen::Matrix<double, kDimState, kDimMeasurementPose>;
    using MatrixKPoseVel=Eigen::Matrix<double, kDimState, kDimMeasurementPoseVel>;
    using MatrixKPosiVel=Eigen::Matrix<double, kDimState, kDimMeasurementPosiVel>;

    // state observality matrix:
    using MatrixQPose=Eigen::Matrix<double, kDimState * kDimMeasurementPose, kDimState>;
    using MatrixQPoseVel=Eigen::Matrix<double, kDimState * kDimMeasurementPoseVel, kDimState>;
    using MatrixQPosiVel=Eigen::Matrix<double, kDimState * kDimMeasurementPosiVel, kDimState>;
    using MatrixQVel=Eigen::Matrix<double, kDimState * kDimMeasurementPosiVel, kDimState>;
  
  
    typedef Eigen::Matrix<double,   DIM_MEASUREMENT_POSI_VEL - 1,                      DIM_STATE> MatrixGPosiVelCons;
    typedef Eigen::Matrix<double,   DIM_MEASUREMENT_POSI_VEL - 1, DIM_MEASUREMENT_POSI_VEL_NOISE> MatrixCPosiVelCons;
    typedef Eigen::Matrix<double,   DIM_MEASUREMENT_POSI_VEL - 1,                              1> VectorYPosiVelCons;
    typedef Eigen::Matrix<double, DIM_MEASUREMENT_POSI_VEL_NOISE - 1, DIM_MEASUREMENT_POSI_VEL_NOISE - 1> MatrixRPosiVelCons;
    typedef Eigen::Matrix<double,   DIM_MEASUREMENT_POSE_VEL - 1,                      DIM_STATE> MatrixGPoseVelCons;
    typedef Eigen::Matrix<double,   DIM_MEASUREMENT_POSE_VEL - 1, DIM_MEASUREMENT_POSE_VEL_NOISE> MatrixCPoseVelCons;
    typedef Eigen::Matrix<double,   DIM_MEASUREMENT_POSE_VEL - 1,                              1> VectorYPoseVelCons;
    typedef Eigen::Matrix<double, DIM_MEASUREMENT_POSE_VEL_NOISE - 1, DIM_MEASUREMENT_POSE_VEL_NOISE - 1> MatrixRPoseVelCons;




    // state:
    VectorX X_ = VectorX::Zero();
    MatrixP P_ = MatrixP::Zero();
    // process & measurement equations:
    MatrixF F_ = MatrixF::Zero();
    MatrixB B_ = MatrixB::Zero();
    MatrixQ Q_ = MatrixQ::Zero();

    MatrixGPose GPose_ = MatrixGPose::Zero();
    MatrixCPose CPose_ = MatrixCPose::Zero();
    MatrixRPose RPose_ = MatrixRPose::Zero();
    MatrixQPose QPose_ = MatrixQPose::Zero();

    MatrixGPoseVel GPoseVel_ = MatrixGPoseVel::Zero();
    MatrixCPoseVel CPoseVel_ = MatrixCPoseVel::Zero();
    MatrixRPoseVel RPoseVel_ = MatrixRPoseVel::Zero();
    MatrixQPoseVel QPoseVel_ = MatrixQPoseVel::Zero();

    
    //vel 
    MatrixGVel GVel = MatrixGVel::Zero();
    MatrixCVel CVel = MatrixCVel::Zero();
    MatrixRVel RVel = MatrixRVel::Zero();
    MatrixQVel QVel = MatrixQVel::Zero();

    MatrixGPosiVelCons GPosiVelCons_ = MatrixGPosiVelCons::Zero();
    MatrixCPosiVelCons CPosiVelCons_ = MatrixCPosiVelCons::Zero();

    MatrixGPoseVelCons GPoseVelCons_ = MatrixGPoseVelCons::Zero();
    MatrixCPoseVelCons CPoseVelCons_ = MatrixCPoseVelCons::Zero();

    MatrixGPosiVel GPosiVel_ = MatrixGPosiVel::Zero();
    MatrixCPosiVel CPosiVel_ = MatrixCPosiVel::Zero();
    MatrixRPosiVel RPosiVel_ = MatrixRPosiVel::Zero();
    MatrixQPosiVel QPosiVel_ = MatrixQPosiVel::Zero();

    // measurement:
    VectorYPose YPose_ = VectorYPose::Zero();
    VectorYPoseVel YPoseVel_ = VectorYPoseVel::Zero(); 
    VectorYPosiVel YPosiVel_ = VectorYPosiVel::Zero(); 

    VectorYVel YVel_ = VectorYVel::Zero();

    //eskf config
    private:
        struct {
            struct {
            double POSI;
            double VEL;
            double ORI;
            double EPSILON;
            double DELTA;
            } PRIOR;
            struct {
            double GYRO;
            double ACCEL;
            double BIAS_ACCEL;
            double BIAS_GYRO;
            } PROCESS;
            struct {
            struct {
                double POSI;
                double ORI;
            } POSE;
            double POSI;
            double VEL;
            double ORI;
            double MAG;
            } MEASUREMENT;
        } COV;
        // c. motion constraint:
        struct {
            bool ACTIVATED;
            double W_B_THRESH;
        } MOTION_CONSTRAINT;


      Eigen::Vector3d w_b_global = Eigen::Vector3d::Zero();
};

#endif // LESSON5_LIDAR_UNDISTORTION_H_
