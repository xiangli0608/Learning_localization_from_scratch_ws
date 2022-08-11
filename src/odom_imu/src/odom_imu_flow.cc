#include "tools/odom_imu_flow.h"




static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / M_PI * 180.0;
}

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
{
    typedef typename Derived::Scalar Scalar_t;

    Scalar_t y = ypr(0) / 180.0 * M_PI;
    Scalar_t p = ypr(1) / 180.0 * M_PI;
    Scalar_t r = ypr(2) / 180.0 * M_PI;

    Eigen::Matrix<Scalar_t, 3, 3> Rz;
    Rz << cos(y), -sin(y), 0,
        sin(y), cos(y), 0,
        0, 0, 1;

    Eigen::Matrix<Scalar_t, 3, 3> Ry;
    Ry << cos(p), 0., sin(p),
        0., 1., 0.,
        -sin(p), 0., cos(p);

    Eigen::Matrix<Scalar_t, 3, 3> Rx;
    Rx << 1., 0., 0.,
        0., cos(r), -sin(r),
        0., sin(r), cos(r);

    return Rz * Ry * Rx;
}

Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta)
{
    Eigen::Matrix3d R_x;    // 计算旋转矩阵的X分量
    R_x <<
            1,              0,               0,
            0,  cos(theta[0]),  -sin(theta[0]),
            0,  sin(theta[0]),   cos(theta[0]);

    Eigen::Matrix3d R_y;    // 计算旋转矩阵的Y分量
    R_y <<
            cos(theta[1]),   0, sin(theta[1]),
            0,   1,             0,
            -sin(theta[1]),  0, cos(theta[1]);

    Eigen::Matrix3d R_z;    // 计算旋转矩阵的Z分量
    R_z <<
            cos(theta[2]), -sin(theta[2]), 0,
            sin(theta[2]),  cos(theta[2]), 0,
            0,              0,             1;
    Eigen::Matrix3d R = R_z * R_y * R_x;
    return R;
}

Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
{
    assert(isRotationMatirx(R));
    double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2( R(2,1), R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2( R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return {x, y, z};
}

Eigen::Matrix3d g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();
    Eigen::Vector3d ng2{0, 0, 1.0};
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    double yaw = R2ypr(R0).x();
    R0 = ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}

// imu deque 的数据长度
const int queueLength = 2000;

OdomImuFlow::OdomImuFlow()
{
    ROS_INFO_STREAM("\033[1;32m----> odom imu node started.\033[0m");

    //初始化一些参数
    InitWithConfig();
    
    
    // imu 话题接受
    imu_subscriber_ = node_handle_.subscribe(
        imu_topic_string, 2000, &OdomImuFlow::ImuCallback, this, ros::TransportHints().tcpNoDelay());
 
    //编码器话题接受
    encoder_subscriber = node_handle_.subscribe(ENCODER_TOPIC, 2000, &OdomImuFlow::EncoderCallback, this, ros::TransportHints().tcpNoDelay()); // wallong

    // ESKF 结果输出
    odom_publisher_ = node_handle_.advertise<nav_msgs::Odometry>("imu_odom", 1000);

    // 编码器的观测轨迹 输出
    odom_measurement_publisher_ = node_handle_.advertise<nav_msgs::Odometry>("measurement_odom", 1000);
    
    //滤波相关的参数
    
    //  process noise:
    Q_.block<3, 3>(kIndexNoiseAccel, kIndexNoiseAccel) = COV.PROCESS.ACCEL * Eigen::Matrix3d::Identity();
    
    //roll pitch yaw
    Q_.block<3, 3>(kIndexNoiseGyro, kIndexNoiseGyro) = COV.PROCESS.GYRO * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(kIndexNoiseBiasAccel, kIndexNoiseBiasAccel) = COV.PROCESS.BIAS_ACCEL * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(kIndexNoiseBiasGyro, kIndexNoiseBiasGyro) = COV.PROCESS.BIAS_GYRO * Eigen::Matrix3d::Identity();


    // measurement noise:
    RVel = COV.MEASUREMENT.VEL * Eigen::Matrix3d::Identity();
    
    CVel = Eigen::Matrix3d::Identity();
    

    RPosiVel_.block<3, 3>(0, 0) = COV.MEASUREMENT.POSI*Eigen::Matrix3d::Identity();
    RPosiVel_.block<3, 3>(3, 3) = COV.MEASUREMENT.VEL*Eigen::Matrix3d::Identity();

    GPoseVel_.block<3, 3>(0, INDEX_ERROR_POS) = Eigen::Matrix3d::Identity();
    GPoseVel_.block<3, 3>(3, INDEX_ERROR_ORI) = Eigen::Matrix3d::Identity();
    CPoseVel_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    CPoseVel_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
    CPoseVel_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
    // RPosiVel_.block<3, 3>(0, 0) = 1.0e-4*Eigen::Matrix3d::Identity();
    // RPosiVel_.block<3, 3>(3, 3) = 2.5e-3*Eigen::Matrix3d::Identity();

    // GPosiVel_.block<3, 3>(0, kIndexErrorPos) = Eigen::Matrix3d::Identity();
    // CPosiVel_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    // CPosiVel_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
    GPosiVel_.block<3, 3>(0, INDEX_ERROR_POS) = Eigen::Matrix3d::Identity();
    CPosiVel_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    CPosiVel_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

    GPosiVelCons_.block<4, DIM_STATE>(0, 0) = GPosiVel_.block<4, DIM_STATE>(0, 0);
    GPosiVelCons_.block<1, DIM_STATE>(4, 0) = GPosiVel_.block<1, DIM_STATE>(5, 0);
    CPosiVelCons_.block<4, 4>(0, 0) = Eigen::Matrix4d::Identity();
    CPosiVelCons_(4, 5) = 1.0;

    GPoseVelCons_.block<7, DIM_STATE>(0, 0) = GPoseVel_.block<7, DIM_STATE>(0, 0);
    GPoseVelCons_.block<1, DIM_STATE>(7, 0) = GPoseVel_.block<1, DIM_STATE>(8, 0);
    CPoseVelCons_.block<7, 7>(0, 0) = Eigen::Matrix<double, 7, 7>::Identity();
    CPoseVelCons_(7, 8) = 1.0;

    // init soms:
    QPoseVel_.block<DIM_MEASUREMENT_POSE_VEL, DIM_STATE>(0, 0) = GPoseVel_;
    QPosiVel_.block<DIM_MEASUREMENT_POSI_VEL, DIM_STATE>(0, 0) = GPosiVel_;

    // e. process equation:
    F_.block<3, 3>(kIndexErrorPos, kIndexErrorVel) = Eigen::Matrix3d::Identity();
    F_.block<3, 3>(kIndexErrorOri, kIndexErrorGyro) = -Eigen::Matrix3d::Identity();

    B_.block<3, 3>(kIndexErrorOri, kIndexNoiseGyro) = Eigen::Matrix3d::Identity();
    B_.block<3, 3>(kIndexErrorAccel, kIndexNoiseBiasAccel) = Eigen::Matrix3d::Identity();
    B_.block<3, 3>(kIndexErrorGyro, kIndexNoiseBiasGyro) = Eigen::Matrix3d::Identity();

  

    P_ = MatrixP::Zero();

    P_.block<3, 3>(kIndexErrorPos, kIndexErrorPos) =
        COV.PRIOR.POSI * Eigen::Matrix3d::Identity();
    P_.block<3, 3>(kIndexErrorVel, kIndexErrorVel) =
        COV.PRIOR.VEL * Eigen::Matrix3d::Identity();
    P_.block<3, 3>(kIndexErrorOri, kIndexErrorOri) =
        COV.PRIOR.ORI * Eigen::Matrix3d::Identity();

    // P_(7,7) = 2; //把pitch角的信任度降低
    //gyro cov
    P_.block<3, 3>(kIndexErrorGyro, kIndexErrorGyro) =
        COV.PRIOR.EPSILON * Eigen::Matrix3d::Identity();
    //acc cov
    P_.block<3, 3>(kIndexErrorAccel, kIndexErrorAccel) =
        COV.PRIOR.DELTA * Eigen::Matrix3d::Identity();


    
}

OdomImuFlow::~OdomImuFlow()
{

}



void OdomImuFlow::InitWithConfig(){
    
    std::string config_file_path =  "/home/maotian/RosProject/slammaotian/Localization_lixiang/Learning_localization_from_scratch_ws-master/src/odom_imu/config/filter.yaml";

    YAML::Node node = YAML::LoadFile(config_file_path);

    imu_topic_string = node["imu_topic"].as<std::string>();

    ENCODER_TOPIC = node["ENCODER_TOPIC"].as<std::string>();


    odom_or_vel = node["odom_or_vel"].as<bool>();


    Encoder_resolution = node["Encoder_resolution"].as<int>();
    Encoder_left_wheel_diameter = node["Encoder_left_wheel_diameter"].as<double>();
    Encoder_right_wheel_diameter = node["Encoder_right_wheel_diameter"].as<double>();
    Encoder_wheel_base = node["Encoder_wheel_base"].as<double>();



    COV.PRIOR.POSI = node["covariance"]["prior"]["pos"].as<double>();

    COV.PRIOR.POSI = node["covariance"]["prior"]["pos"].as<double>();
    COV.PRIOR.VEL = node["covariance"]["prior"]["vel"].as<double>();
    COV.PRIOR.ORI = node["covariance"]["prior"]["ori"].as<double>();
    COV.PRIOR.EPSILON = node["covariance"]["prior"]["epsilon"].as<double>();
    COV.PRIOR.DELTA = node["covariance"]["prior"]["delta"].as<double>();

    // c. process noise:
    COV.PROCESS.ACCEL = node["covariance"]["process"]["accel"].as<double>();
    COV.PROCESS.GYRO = node["covariance"]["process"]["gyro"].as<double>();
    COV.PROCESS.BIAS_ACCEL =
        node["covariance"]["process"]["bias_accel"].as<double>();
    COV.PROCESS.BIAS_GYRO =
        node["covariance"]["process"]["bias_gyro"].as<double>();

    // d. measurement noise:
    COV.MEASUREMENT.POSE.POSI =
        node["covariance"]["measurement"]["pose"]["pos"].as<double>();
    COV.MEASUREMENT.POSE.ORI =
        node["covariance"]["measurement"]["pose"]["ori"].as<double>();
    COV.MEASUREMENT.POSI = 
        node["covariance"]["measurement"]["pos"].as<double>();
    COV.MEASUREMENT.VEL = 
        node["covariance"]["measurement"]["vel"].as<double>();
    // e. motion constraint:
    MOTION_CONSTRAINT.ACTIVATED = 
        node["motion_constraint"]["activated"].as<bool>();
    MOTION_CONSTRAINT.W_B_THRESH = 
        node["motion_constraint"]["w_b_thresh"].as<double>();


    std::cout << std::endl
            << "\tprior cov. pos.: " << COV.PRIOR.POSI << std::endl
            << "\tprior cov. vel.: " << COV.PRIOR.VEL << std::endl
            << "\tprior cov. ori: " << COV.PRIOR.ORI << std::endl
            << "\tprior cov. epsilon.: " << COV.PRIOR.EPSILON << std::endl
            << "\tprior cov. delta.: " << COV.PRIOR.DELTA << std::endl
            << std::endl
            << "\tprocess noise gyro.: " << COV.PROCESS.GYRO << std::endl
            << "\tprocess noise accel.: " << COV.PROCESS.ACCEL << std::endl
            << std::endl
            << "\tmeasurement noise pose.: " << std::endl
            << "\t\tpos: " << COV.MEASUREMENT.POSE.POSI
            << ", ori.: " << COV.MEASUREMENT.POSE.ORI << std::endl
            << "\tmeasurement noise pos.: " << COV.MEASUREMENT.POSI << std::endl
            << "\tmeasurement noise vel.: " << COV.MEASUREMENT.VEL << std::endl
            << std::endl
            << "\tmotion constraint: " << std::endl 
            << "\t\tactivated: " << (MOTION_CONSTRAINT.ACTIVATED ? "true" : "false") << std::endl
            << "\t\tw_b threshold: " << MOTION_CONSTRAINT.W_B_THRESH << std::endl
            << std::endl;
}


// ESKF 融合程序
bool OdomImuFlow::fusion2_Run(){

        
        while (!imu_queue_.empty() && !odom_queue_.empty())
        {
            if(!has_initial){
                // 初始化 imu 与 odom的一个对齐点 并初始化速度
                if ( ValidOdomData() ) {
                    InitialFilter();
                }
            }   
            else{
                //找到和imu相近的 odom 数据
                if(!odom_queue_.empty() && ValidOdomData()){

                    // 轮速计的位移观测
                    static nav_msgs::Odometry last_odom = cur_odom;
                    
                    double dt = cur_odom.header.stamp.toSec() - last_odom.header.stamp.toSec();
                    double pre_w = last_odom.twist.twist.angular.z;
                    double cur_w = cur_odom.twist.twist.angular.z;
                    Eigen::Vector3d angular_delta{0.0,0.0,0.5*dt*(pre_w + cur_w)};

                    Eigen::Matrix3d  R_curr  =  Eigen::Matrix3d::Identity();
                    Eigen::Matrix3d  R_prev =  Eigen::Matrix3d::Identity();
                    double angular_delta_mag = angular_delta.norm();
                    // direction:
                    Eigen::Vector3d angular_delta_dir = angular_delta.normalized();

                    // build delta q:
                    double angular_delta_cos = cos(angular_delta_mag / 2.0);
                    double angular_delta_sin = sin(angular_delta_mag / 2.0);
                    Eigen::Quaterniond dq(angular_delta_cos,
                                            angular_delta_sin * angular_delta_dir.x(),
                                            angular_delta_sin * angular_delta_dir.y(),
                                            angular_delta_sin * angular_delta_dir.z());

                    // std::cout << " -----------------------------------  pose_.block<3, 3>(0, 0) " << pose_.block<3, 3>(0, 0) << std::endl;
                    Eigen::Quaterniond q(pose_.block<3, 3>(0, 0));

                    // update:
                    q = q * dq;

                    // write back:
                    R_prev = pose_.block<3, 3>(0, 0);

                    R_curr = q.normalized().toRotationMatrix(); 
                    
                    Eigen::Vector3d pre_v_b{last_odom.twist.twist.linear.x, 0.0, 0.0};
                    Eigen::Vector3d cur_v_b{cur_odom.twist.twist.linear.x, 0.0, 0.0};

                    Eigen::Vector3d velocity_ = 0.5 *(R_prev * pre_v_b + R_curr * cur_v_b);
                    std::cout << " dt : : : " << dt << std::endl;
                    measur_pose_.block<3,1>(0,3) = measur_pose_.block<3,1>(0,3) +  velocity_ * dt;

                    measur_pose_.block<3,3>(0,0) = R_curr;

                    last_odom = cur_odom;


                    //发布 里程计预测位
                    Eigen::Quaterniond q_cur_measur(measur_pose_.block<3,3>(0,0));

                    nav_msgs::Odometry odometry_measure_;



                    // imu_br.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "map", "robot"));
                    
                    odometry_measure_.header.frame_id = "map";
                    odometry_measure_.child_frame_id = "robot_measurement";
                    // set the pose
                    odometry_measure_.pose.pose.position.x = measur_pose_(0,3);
                    odometry_measure_.pose.pose.position.y = measur_pose_(1,3);
                    odometry_measure_.pose.pose.position.z = measur_pose_(2,3);

                    odometry_measure_.pose.pose.orientation.x = q_cur_measur.x();
                    odometry_measure_.pose.pose.orientation.y = q_cur_measur.y();
                    odometry_measure_.pose.pose.orientation.z = q_cur_measur.z();
                    odometry_measure_.pose.pose.orientation.w = q_cur_measur.w();


                    odom_measurement_publisher_.publish(odometry_measure_);

                    if(!imu_queue_.empty()){
                        while (!imu_queue_.empty() && ValidIMUData() && 
                                cur_imu.header.stamp.toSec() < cur_odom.header.stamp.toSec())
                        {

                            // std::cout << " come in process imu odom" << std::endl;
                            //IMU 状态与方差更新
                            UpdateLocalization();
                            
                            if (
                                cur_imu.header.stamp.toSec() >= cur_odom.header.stamp.toSec()
                            ) {
                                imu_data_buff_.push_back(cur_imu);
                            }

                            Eigen::Quaterniond q_cur(pose_.block<3,3>(0,0));

                            tf::Transform transform_;
                            nav_msgs::Odometry odometry_;


                            transform_.setRotation(tf::Quaternion(q_cur.x(), q_cur.y(), q_cur.z(), q_cur.w()));
                            transform_.setOrigin(tf::Vector3(pose_(0,3), pose_(1,3), pose_(2,3)));



                            imu_br.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "map", "robot"));
                            
                            odometry_.header.frame_id = "map";
                            odometry_.child_frame_id = "robot";
                            // set the pose
                            odometry_.pose.pose.position.x = pose_(0,3);
                            odometry_.pose.pose.position.y = pose_(1,3);
                            odometry_.pose.pose.position.z = pose_(2,3);

                            odometry_.pose.pose.orientation.x = q_cur.x();
                            odometry_.pose.pose.orientation.y = q_cur.y();
                            odometry_.pose.pose.orientation.z = q_cur.z();
                            odometry_.pose.pose.orientation.w = q_cur.w();


                            odom_publisher_.publish(odometry_);


                        }

                        
                    }
                    // if(IsRotaton())
                    // CorrectLocalization();
                    //观测纠正 位移 + body坐标系的速度
                    CorrectLocalizationPosVel();
                    // CorrectLocalizationPosOriVel();
                }
                

                if(!imu_queue_.empty() && ValidIMUData()){
                    UpdateLocalization();
                }

            }
        }
        
    

    
    
    return true;
}


// 提取 当前 imu
bool OdomImuFlow::ValidIMUData(){

    cur_imu = imu_queue_.front();

    imu_queue_.pop_front();

    return true;
}


// imu 和 odom 的 同步
bool OdomImuFlow::ValidOdomData(){


    while (!odom_queue_.empty() && !imu_sync_queue_.empty())
    {
        cur_odom = odom_queue_.front();
        cur_imu_sync_raw = imu_sync_queue_.front();

        const double cur_sync_imu_time = cur_imu_sync_raw.header.stamp.toSec();
        const double cur_odom_time = cur_odom.header.stamp.toSec();
        
        double diff_imu_time = cur_odom_time - cur_sync_imu_time;

        if ( diff_imu_time < -0.01) {
            odom_queue_.pop_front();
            continue;
        }

        if (diff_imu_time > 0.01) {
            imu_sync_queue_.pop_front();
            continue;
        }


        imu_sync_queue_.pop_front();
    
        odom_queue_.pop_front();
        break;
    }
 
    // std::cout << "imu_sync_queue_ size  " << imu_sync_queue_.size() << std::endl;
    // std::cout << "imu_queue_ size  " << imu_queue_.size() << std::endl;

    return true;
}


//IMU 预测状态以及方差更新 
bool OdomImuFlow::UpdateLocalization()
{

    if (imu_last_time < cur_imu.header.stamp.toSec())
    {
        // std::cout << " start prediction process " << std::endl;
        // update IMU odometry:
        Eigen::Vector3d linear_acc_mid;
        Eigen::Vector3d angular_vel_mid;
        imu_data_buff_.push_back(cur_imu);
        UpdateOdomEstimation(linear_acc_mid, angular_vel_mid);
        imu_data_buff_.pop_front();

        // update error estimation:
        double T = cur_imu.header.stamp.toSec() - imu_last_time;
        UpdateErrorEstimation(T, linear_acc_mid, angular_vel_mid); //  更新误差估计

        imu_last_time = cur_imu.header.stamp.toSec();

        return true;
    }

    return false;
}


//只有 body 坐标系的速度做观测
bool OdomImuFlow::CorrectLocalization() {
    


    double time_delta = cur_odom.header.stamp.toSec() - imu_last_time;
    Eigen::VectorXd Y;
    Eigen::MatrixXd G, K;

    if (time_delta > -0.01) {  
        if(imu_last_time < cur_odom.header.stamp.toSec()){
            UpdateLocalization();
        }
        std::cout << " time_delta : " << time_delta << std::endl;
        
        Eigen::Vector3d  v_b_  =  {cur_odom.twist.twist.linear.x,  0,  0};           //  measurment   velocity  (body  系) ， 伪观测 (vy 、vz  = 0)
        // std::cout << " vel pre : " << pose_.block<3,  3>(0, 0).transpose() *vel_ << std::endl;
        // std::cout << " vel measurement : " << v_b_ << std::endl;
        Eigen::Vector3d  dv  =   pose_.block<3,  3>(0, 0).transpose() *vel_  -  v_b_ ;                  //  delta v  ,  v_x 来自轮速里程计
        std::cout << " dv : " << dv << std::endl;
        YVel_ = dv;
        Y = YVel_;

        //   set measurement  G
        GVel.setZero();
        GVel.block<3, 3>(0, kIndexErrorVel)   =   pose_.block<3,  3>(0, 0).transpose();
        Eigen::Vector3d rv_vec = pose_.block<3,  3>(0, 0).transpose() *vel_;
        
        Eigen::Matrix3d rv_hat;
        rv_hat << 0,     -rv_vec(2),    rv_vec(1),
                rv_vec(2),       0,    -rv_vec(0),
                -rv_vec(1),  rv_vec(0),   0;
        
        GVel.block<3, 3>(0, kIndexErrorOri)   =   rv_hat;
        G  =   GVel;  

        //   set measurement  C
        CVel.setIdentity();
        Eigen::MatrixXd  C  =   CVel;
        // TODO: set Kalman gain:
        Eigen::MatrixXd R = RVel;    //  观测噪声
        K =  P_  *  G.transpose() * ( G  *  P_  *  G.transpose( )  +  C * R*  C.transpose() ).inverse() ;

        //
        // TODO: perform Kalman correct:
        P_ = (MatrixP::Identity() -  K*G)  *  P_ ;          //  后验方差
        // std:: cout << " 225 X_ : " << X_ << std::endl;
        // std:: cout << " Y_ : " << Y << std::endl;
        X_ =  X_ +  K * (Y - G*X_);                                                      //  更新后的状态量

        //      误差状态量  X_  :   15*1
        // TODO: correct state estimation using the state of ESKF
        //
        // a. position:
        // do it!
        // std::cout << " X_ : " << X_ << std::endl;
        // std::cout << " before vel_ : " << vel_ << std::endl;
        pose_.block<3, 1>(0, 3)  -=  X_.block<3, 1>(kIndexErrorPos, 0 );   //  减去error
        // b. velocity:
        // do it!
        vel_ -=  X_.block<3,1>(kIndexErrorVel, 0 );   

        // std::cout << " after vel_ : " << vel_ << std::endl;
        Eigen::Vector3d  dv_opti  =   pose_.block<3,  3>(0, 0).transpose() *vel_  -  v_b_ ;                  //  delta v  ,  v_x 来自轮速里程计
        std::cout << " dv_opti : " << dv_opti << std::endl;

        // c. orientation:
        // do it!
        Eigen::Matrix3d dtheta_cross_hat;
        dtheta_cross_hat <<       0,     -X_(8,0),    X_(7,0),
                            X_(8,0),            0,   -X_(6,0),
                        -X_(7,0),      X_(6,0),          0;
        
        Eigen::Matrix3d   dtheta_cross =  dtheta_cross_hat;         //   失准角的反对称矩阵
        pose_.block<3, 3>(0, 0) =  pose_.block<3, 3>(0, 0) * (Eigen::Matrix3d::Identity() - dtheta_cross);     
        Eigen::Quaterniond  q_tmp(pose_.block<3, 3>(0, 0) );
        q_tmp.normalize();        //  为了保证旋转矩阵是正定的
        pose_.block<3, 3>(0, 0) = q_tmp.toRotationMatrix();  

        // d. gyro bias:
        // std::cout << " gyro_bias_ :" << gyro_bias_ << std::endl;
        // std::cout << " acc_bias_ :" << accl_bias_ << std::endl;

        if (IsCovStable(kIndexErrorGyro)) {
            // std::cout << " p_ gyro ok " << std::endl;
            gyro_bias_ += X_.block<3, 1>(kIndexErrorGyro, 0);           //  判断gyro_bias_error是否可观
        }
        std::cout << " --------------------------------------- " << std::endl;
        // e. accel bias:
        if (IsCovStable(kIndexErrorAccel)) {
            // std::cout << " p_ acc ok " << std::endl;
            accl_bias_ += X_.block<3, 1>(kIndexErrorAccel, 0);          //   判断accel_bias_error是否可观 
        }

        X_ = VectorX::Zero();
        
    }
   
       

    
    return false;
}

// body 坐标系的速度 以及 位置 观测
bool OdomImuFlow::CorrectLocalizationPosVel(){
  
        double time_delta = cur_odom.header.stamp.toSec() - imu_last_time;
        Eigen::VectorXd Y;
        Eigen::MatrixXd G, K;

        if (time_delta > -0.05) {  
            if(imu_last_time < cur_odom.header.stamp.toSec()){
                UpdateLocalization();
            }

            std::cout << " time_delta : " << time_delta << std::endl;

            //IESKF
            std::cout << " - - - - - - - - - IESKF Start - - - - - - - - -" << std::endl; 
            Eigen::Vector3d  v_b  =  {cur_odom.twist.twist.linear.x,  0,  0};           //  measurment   velocity  (body  系) ， 伪观测 (vy 、vz  = 0)
            std::cout << " v_b : " << v_b << std::endl;
            
            Eigen::Vector3d  dv  =   pose_.block<3,  3>(0, 0).transpose() *vel_  -  v_b ;                  //  delta v  ,  v_x 来自轮速里程计
            
            Eigen::Vector3d dypr = pose_.block<3,3>(0,0).eulerAngles(2,1,0); // ZYX顺序，yaw,pitch,roll

            std::cout << " d pitch  " << dypr[1] * 57.3<< std::endl;

            for(int i = 0; i < 5 ; ++i){
                
                // parse measurement:
                Eigen::Vector3d P_nn_obs = pose_.block<3, 1>(0,3) - measur_pose_.block<3, 1>(0,3);
                Eigen::Vector3d v_bb_obs = pose_.block<3, 3>(0,0).transpose()*vel_ - v_b;

                YPosiVel_.block<3, 1>(0, 0) = P_nn_obs;
                YPosiVel_.block<3, 1>(3, 0) = v_bb_obs;

                Y = YPosiVel_;
                Eigen::Vector3d rv_vec = pose_.block<3,  3>(0, 0).transpose() *vel_;
                
                Eigen::Matrix3d rv_hat;
                rv_hat << 0,     -rv_vec(2),    rv_vec(1),
                        rv_vec(2),       0,    -rv_vec(0),
                        -rv_vec(1),  rv_vec(0),   0;    
                // set measurement equation:
                GPosiVel_.block<3, 3>(3, INDEX_ERROR_VEL) =  pose_.block<3, 3>(0,0).transpose();
                GPosiVel_.block<3, 3>(3, INDEX_ERROR_ORI) = rv_hat;

                G = GPosiVel_;

                if ( !IsTurning(w_b_global)) {
                    // set measurement, with motion constraint:
                    VectorYPosiVelCons YPosiVelCons = VectorYPosiVelCons::Zero();
                    YPosiVelCons.block<4, 1>(0, 0) = YPosiVel_.block<4, 1>(0, 0);
                    YPosiVelCons(4, 0) = YPosiVel_(5, 0);

                    Y = YPosiVelCons;

                    // set measurement equation, with motion constraint:
                    GPosiVelCons_.block<1, DIM_STATE>(3, 0) = GPosiVel_.block<1, DIM_STATE>(3, 0);
                    GPosiVelCons_.block<1, DIM_STATE>(4, 0) = GPosiVel_.block<1, DIM_STATE>(5, 0);

                    G = GPosiVelCons_;

                    // set Kalman gain:
                    MatrixRPosiVelCons RCons = GPosiVelCons_*P_*GPosiVelCons_.transpose() + CPosiVelCons_*RPosiVel_*CPosiVelCons_.transpose();
                    K = P_*GPosiVelCons_.transpose()*RCons.inverse();
                } else {
                    // set Kalman gain:
                    MatrixRPosiVel R = GPosiVel_*P_*GPosiVel_.transpose() + RPosiVel_;
                    K = P_*GPosiVel_.transpose()*R.inverse();
                }  
                        //
                // TODO: perform Kalman correct:
                P_ = (MatrixP::Identity() -  K*G)  *  P_ ;          //  后验方差
                X_ =  X_ +  K * (Y - G*X_);                                                      //  更新后的状态量

                //      误差状态量  X_  :   15*1
                // TODO: correct state estimation using the state of ESKF
                //
                // a. position:
                // do it!
                // std::cout << " X_ : " << X_ << std::endl;
                // std::cout << " before vel_ : " << vel_ << std::endl;
                pose_.block<3, 1>(0, 3)  -=  X_.block<3, 1>(kIndexErrorPos, 0 );   //  减去error
                // b. velocity:
                // do it!
                vel_ -=  X_.block<3,1>(kIndexErrorVel, 0 );   
                
                // std::cout << " after vel_ : " << vel_ << std::endl;

                // c. orientation:
                // do it!
                Eigen::Matrix3d dtheta_cross_hat;
                dtheta_cross_hat <<       0,     -X_(8,0),    X_(7,0),
                                    X_(8,0),            0,   -X_(6,0),
                                -X_(7,0),      X_(6,0),          0;
                
                Eigen::Matrix3d   dtheta_cross =  dtheta_cross_hat;         //   失准角的反对称矩阵
                
                pose_.block<3, 3>(0, 0) =  pose_.block<3, 3>(0, 0) * (Eigen::Matrix3d::Identity() - dtheta_cross);     
                Eigen::Quaterniond  q_tmp(pose_.block<3, 3>(0, 0) );
                q_tmp.normalize();        //  为了保证旋转矩阵是正定的
                pose_.block<3, 3>(0, 0) = q_tmp.toRotationMatrix();  

                // // d. gyro bias:
                // std::cout << " gyro_bias_ :" << gyro_bias_ << std::endl;
                // std::cout << " acc_bias_ :" << accl_bias_ << std::endl;

                if (IsCovStable(kIndexErrorGyro)) {
                    // std::cout << " p_ gyro ok " << std::endl;
                    gyro_bias_ += X_.block<3, 1>(kIndexErrorGyro, 0);           //  判断gyro_bias_error是否可观
                }
                // std::cout << " --------------------------------------- " << std::endl;
                // e. accel bias:
                if (IsCovStable(kIndexErrorAccel)) {
                    // std::cout << " p_ acc ok " << std::endl;
                    accl_bias_ += X_.block<3, 1>(kIndexErrorAccel, 0);          //   判断accel_bias_error是否可观 
                }

                X_ = VectorX::Zero();
            }
            Eigen::Vector3d  dv_opti  =   pose_.block<3,  3>(0, 0).transpose() *vel_  -  v_b ;                  //  delta v  ,  v_x 来自轮速里程计
            std::cout << " dv_opti : " << dv_opti << std::endl;
            Eigen::Vector3d dypr_opti = pose_.block<3,3>(0,0).eulerAngles(2,1,0); // ZYX顺序，yaw,pitch,roll

            std::cout << " d pitch after " << dypr_opti[1] * 57.3 << std::endl;
            
            
        }
  
    
}

// body 坐标系的速度  位置  姿态 观测
bool OdomImuFlow::CorrectLocalizationPosOriVel(){
 
    double time_delta = cur_odom.header.stamp.toSec() - imu_last_time;
    Eigen::VectorXd Y;
    Eigen::MatrixXd G, K;

    if (time_delta > -0.05) {  
        if(imu_last_time < cur_odom.header.stamp.toSec()){
            UpdateLocalization();
        }
        std::cout << " time_delta : " << time_delta << std::endl;
        Eigen::Vector3d  v_b  =  {cur_odom.twist.twist.linear.x,  0,  0};           //  measurment   velocity  (body  系) ， 伪观测 (vy 、vz  = 0)
        
        // set measurement:
        Eigen::Vector3d P_nn_obs = pose_.block<3, 1>(0,3) - measur_pose_.block<3, 1>(0,3);
        Eigen::Matrix3d C_nn_obs = pose_.block<3, 3>(0,0) * measur_pose_.block<3, 3>(0,0).transpose();
        Eigen::Vector3d v_bb_obs = pose_.block<3, 3>(0,0).transpose()*vel_ - v_b;


        Eigen::Matrix3d temp_matrix = Eigen::Matrix3d::Identity() - C_nn_obs;
        Eigen::Vector3d temp_vee(temp_matrix(2,1),temp_matrix(0,2),temp_matrix(1,0));
        YPoseVel_.block<3, 1>(0, 0) = P_nn_obs;
        YPoseVel_.block<3, 1>(3, 0) = temp_vee;
        YPoseVel_.block<3, 1>(6, 0) = v_bb_obs;

        Y = YPoseVel_;
        Eigen::Vector3d rv_vec = pose_.block<3,  3>(0, 0).transpose() *vel_;
        
        Eigen::Matrix3d rv_hat;
        rv_hat << 0,     -rv_vec(2),    rv_vec(1),
                rv_vec(2),       0,    -rv_vec(0),
                -rv_vec(1),  rv_vec(0),   0;    
        // set measurement equation:
        GPoseVel_.block<3, 3>(6, INDEX_ERROR_VEL) =  pose_.block<3, 3>(0,0).transpose();
        GPoseVel_.block<3, 3>(6, INDEX_ERROR_ORI) = rv_hat;

        G = GPoseVel_;

        if ( !IsTurning(w_b_global)) {
            // set measurement, with motion constraint:
            VectorYPoseVelCons YPoseVelCons = VectorYPoseVelCons::Zero();
            YPoseVelCons.block<7, 1>(0, 0) = YPoseVel_.block<7, 1>(0, 0);
            YPoseVelCons(7, 0) = YPoseVel_(8, 0);

            Y = YPoseVelCons;

            // set measurement equation, with motion constraint:
            GPoseVelCons_.block<1, DIM_STATE>(6, 0) = GPoseVel_.block<1, DIM_STATE>(6, 0);
            GPoseVelCons_.block<1, DIM_STATE>(7, 0) = GPoseVel_.block<1, DIM_STATE>(8, 0);

            G = GPoseVelCons_;

            // set Kalman gain, with motion constraint:
            MatrixRPoseVelCons RCons = GPoseVelCons_*P_*GPoseVelCons_.transpose() + CPoseVelCons_*RPoseVel_*CPoseVelCons_.transpose();
            K = P_*GPoseVelCons_.transpose()*RCons.inverse();
        } else {
            // set Kalman gain:
            MatrixRPoseVel R = GPoseVel_*P_*GPoseVel_.transpose() + RPoseVel_;
            K = P_*GPoseVel_.transpose()*R.inverse();
        }
                    //
        // TODO: perform Kalman correct:
        P_ = (MatrixP::Identity() -  K*G)  *  P_ ;          //  后验方差
        std:: cout << " Y_ : " << Y << std::endl;
        X_ =  X_ +  K * (Y - G*X_);                                                      //  更新后的状态量

        //      误差状态量  X_  :   15*1
        // TODO: correct state estimation using the state of ESKF
        //
        // a. position:
        // do it!
        std::cout << " X_ : " << X_ << std::endl;
        std::cout << " before vel_ : " << vel_ << std::endl;
        pose_.block<3, 1>(0, 3)  -=  X_.block<3, 1>(kIndexErrorPos, 0 );   //  减去error
        // b. velocity:
        // do it!
        vel_ -=  X_.block<3,1>(kIndexErrorVel, 0 );   

        std::cout << " after vel_ : " << vel_ << std::endl;

        // c. orientation:
        // do it!
        Eigen::Matrix3d dtheta_cross_hat;
        dtheta_cross_hat <<       0,     -X_(8,0),    X_(7,0),
                            X_(8,0),            0,   -X_(6,0),
                        -X_(7,0),      X_(6,0),          0;
        
        Eigen::Matrix3d   dtheta_cross =  dtheta_cross_hat;         //   失准角的反对称矩阵
        pose_.block<3, 3>(0, 0) =  pose_.block<3, 3>(0, 0) * (Eigen::Matrix3d::Identity() - dtheta_cross);     
        Eigen::Quaterniond  q_tmp(pose_.block<3, 3>(0, 0) );
        q_tmp.normalize();        //  为了保证旋转矩阵是正定的
        pose_.block<3, 3>(0, 0) = q_tmp.toRotationMatrix();  

        // d. gyro bias:
        std::cout << " gyro_bias_ :" << gyro_bias_ << std::endl;
        std::cout << " acc_bias_ :" << accl_bias_ << std::endl;

        if (IsCovStable(kIndexErrorGyro)) {
            std::cout << " p_ gyro ok " << std::endl;
            gyro_bias_ += X_.block<3, 1>(kIndexErrorGyro, 0);           //  判断gyro_bias_error是否可观
        }
        std::cout << " --------------------------------------- " << std::endl;
        // e. accel bias:
        if (IsCovStable(kIndexErrorAccel)) {
            std::cout << " p_ acc ok " << std::endl;
            accl_bias_ += X_.block<3, 1>(kIndexErrorAccel, 0);          //   判断accel_bias_error是否可观 
        }

        X_ = VectorX::Zero();
        
    }
 
    
}
/**
 * @brief  is covariance stable
 * @param  INDEX_OFSET, state index offset
 * @param  THRESH, covariance threshold, defaults to 1.0e-5
 * @return void
 */
bool OdomImuFlow::IsCovStable(const int INDEX_OFSET,
                                         const double THRESH) {
  for (int i = 0; i < 3; ++i) {
    if (P_(INDEX_OFSET + i, INDEX_OFSET + i) > THRESH) {
      return false;
    }
  }

  return true;
}





void OdomImuFlow::UpdateOdomEstimation(                  //  更新名义值 
    Eigen::Vector3d &linear_acc_mid, Eigen::Vector3d &angular_vel_mid) {

    static Eigen::Vector3d w_b = Eigen::Vector3d::Zero();

  //
  // TODO: this is one possible solution to previous chapter, IMU Navigation,
  // assignment
  //
  // get deltas:
    size_t   index_curr_  = 1;
    size_t   index_prev_ = 0;
    Eigen::Vector3d  angular_delta = Eigen::Vector3d::Zero();            
    GetAngularDelta(index_curr_,  index_prev_,   angular_delta,  angular_vel_mid);           //   获取等效旋转矢量,   保存角速度中值
  // update orientation:
    Eigen::Matrix3d  R_curr_  =  Eigen::Matrix3d::Identity();
    Eigen::Matrix3d  R_prev_ =  Eigen::Matrix3d::Identity();
    angular_delta(0) =0.0;
    UpdateOrientation(angular_delta, R_curr_, R_prev_);                         //     更新四元数
  // get velocity delta:
    double   delta_t_;
    Eigen::Vector3d  velocity_delta_;
    
    GetVelocityDelta(index_curr_, index_prev_,  R_curr_,  R_prev_, delta_t_,  velocity_delta_,  linear_acc_mid);             //  获取速度差值， 保存线加速度中值
    std::cout << " velocity_delta_ : " <<  velocity_delta_ << std::endl;
  
  // save mid-value unbiased linear acc for error-state update:
    // std::cout << "delte_t " << delta_t_ << std::endl;
  // update position:
    UpdatePosition(delta_t_,  velocity_delta_);

    // apply motion constraint:
    w_b = 1.0 / delta_t_ * angular_delta;

    w_b_global = w_b;
    if ( !IsTurning(w_b)) ApplyMotionConstraint();

}


/**
 * @brief  apply motion constraint on velocity estimation
 * @param  void
 * @return void
 */
void OdomImuFlow::ApplyMotionConstraint(void) {
    const Eigen::Matrix3d C_nb = pose_.block<3, 3>(0, 0);

    Eigen::Vector3d v_b = C_nb.transpose() * vel_;
    v_b.y() = 0.0;
    vel_ = C_nb * v_b;
}

/**
 * @brief  whether the ego vehicle is turning
 * @param  void
 * @return void
 */
bool OdomImuFlow::IsTurning(const Eigen::Vector3d &w_b) {
    Eigen::Vector3d w_b_unbiased = GetUnbiasedAngularVel(
        w_b, pose_.block<3, 3>(0, 0)
    );
    // std::cout << " w_b_unbiased.norm() : "  << w_b_unbiased.norm() << std::endl;
    // TODO: move this to config 
    return w_b_unbiased.norm() > 0.08;
}

bool OdomImuFlow::IsRotaton(const Eigen::Vector3d &w_b){

    double v_b;
    if(use_odom_){
        v_b = cur_odom.twist.twist.linear.x;   
    }
    else{
        v_b = cur_vel.twist.linear.x;   
    }

    return w_b.norm() > 0.13 && abs(v_b) < 0.01;
}

/**
 * @brief  update error estimation
 * @param  linear_acc_mid, input mid-value unbiased linear acc
 * @return void
 */
void OdomImuFlow::UpdateErrorEstimation(                       //  更新误差值
    const double &T, const Eigen::Vector3d &linear_acc_mid,
    const Eigen::Vector3d &angular_vel_mid) {
    static MatrixF F_1st;
    static MatrixF F_2nd;
    // TODO: update process equation:         //  更新状态方程
    UpdateProcessEquation(linear_acc_mid ,  angular_vel_mid);
    // TODO: get discretized process equations:         //   非线性化
    F_1st  =  F_ *  T;        //  T kalman 周期
    MatrixF   F = MatrixF::Identity()  +   F_1st;
    MatrixB  B =  MatrixB::Zero();
    B.block<3,  3>(kIndexErrorVel,  kIndexNoiseAccel)  =      B_.block<3,  3>(kIndexErrorVel,  kIndexNoiseAccel) * T;
    B.block<3,  3>(kIndexErrorOri,  kIndexNoiseGyro)  =      B_.block<3,  3>(kIndexErrorOri,  kIndexNoiseGyro) *T;
    B.block<3,  3>(kIndexErrorAccel,  kIndexNoiseBiasAccel)  =    B_.block<3,  3>(kIndexErrorAccel,  kIndexNoiseBiasAccel)* sqrt(T);
    B.block<3,  3>(kIndexErrorGyro,  kIndexNoiseBiasGyro)  =      B_.block<3,  3>(kIndexErrorGyro,  kIndexNoiseBiasGyro)* sqrt(T);

    // TODO: perform Kalman prediction
    X_ = F * X_;
    P_ =  F * P_ * F.transpose()   +  B * Q_ * B.transpose();             //   只有方差进行了计算
}


/**
 * @brief  update process equation
 * @param  imu_data, input IMU measurement
 * @param  T, output time delta
 * @return void
 */
void OdomImuFlow::UpdateProcessEquation(
    const Eigen::Vector3d &linear_acc_mid,
    const Eigen::Vector3d &angular_vel_mid) {
  // set linearization point:
  Eigen::Matrix3d C_nb = pose_.block<3, 3>(0, 0);           //   b2n   转换矩阵
  Eigen::Vector3d f_b = linear_acc_mid ;    //+ g_;                     //   加速度
  Eigen::Vector3d w_b = angular_vel_mid;                         //   角速度

  // set process equation:
  SetProcessEquation(C_nb, f_b, w_b);
}

void OdomImuFlow::SetProcessEquation(const Eigen::Matrix3d &C_nb,                      //   更新状态方程  F矩阵
                                                const Eigen::Vector3d &f_b,
                                                const Eigen::Vector3d &w_b) {
    // TODO: set process / system equation:
    // a. set process equation for delta vel:
    F_.setZero();
    F_.block<3,  3>(kIndexErrorPos,  kIndexErrorVel)  =  Eigen::Matrix3d::Identity();
    Eigen::Matrix3d f_b_hat;
    f_b_hat << 0,     -f_b(2),    f_b(1),
                f_b(2),       0,    -f_b(0),
                -f_b(1),  f_b(0),   0;

    F_.block<3, 3>(kIndexErrorVel,   kIndexErrorOri)  =  - C_nb * f_b_hat;
    F_.block<3, 3>(kIndexErrorVel,   kIndexErrorAccel) =  -C_nb;

    Eigen::Matrix3d w_b_hat;
    w_b_hat << 0,     -w_b(2),    w_b(1),
                w_b(2),       0,    -w_b(0),
                -w_b(1),  w_b(0),   0;
    F_.block<3, 3>(kIndexErrorOri,   kIndexErrorOri) =   - w_b_hat;
    F_.block<3, 3>(kIndexErrorOri,   kIndexErrorGyro) =   - Eigen::Matrix3d::Identity();
    // b. set process equation for delta ori:
    B_.setZero();
    B_.block<3, 3>(kIndexErrorVel,  kIndexNoiseAccel)  =    C_nb;
    B_.block<3, 3>(kIndexErrorOri,  kIndexNoiseGyro)  =     Eigen::Matrix3d::Identity();
    B_.block<3, 3>(kIndexErrorAccel,  kIndexNoiseBiasAccel)  =     Eigen::Matrix3d::Identity();
    B_.block<3, 3>(kIndexErrorGyro,  kIndexNoiseBiasGyro)    =     Eigen::Matrix3d::Identity();
}
/**
 * @brief  get angular delta
 * @param  index_curr, current imu measurement buffer index
 * @param  index_prev, previous imu measurement buffer index
 * @param  angular_delta, angular delta output
 * @return true if success false otherwise
 */
bool OdomImuFlow::GetAngularDelta(const size_t index_curr,
                                             const size_t index_prev,
                                             Eigen::Vector3d &angular_delta,
                                             Eigen::Vector3d &angular_vel_mid) {
  if (index_curr <= index_prev || imu_data_buff_.size() <= index_curr) {
    return false;
  }

  const sensor_msgs::Imu &imu_data_curr = imu_data_buff_.at(index_curr);
  const sensor_msgs::Imu &imu_data_prev = imu_data_buff_.at(index_prev);

  double delta_t = imu_data_curr.header.stamp.toSec() - imu_data_prev.header.stamp.toSec();

  Eigen::Vector3d angular_vel_curr = Eigen::Vector3d(
      imu_data_curr.angular_velocity.x, imu_data_curr.angular_velocity.y,
      imu_data_curr.angular_velocity.z);
  Eigen::Matrix3d R_curr = pose_.block<3,3>(0,0);
  angular_vel_curr = angular_vel_curr  - gyro_bias_;

  Eigen::Vector3d angular_vel_prev = Eigen::Vector3d(
      imu_data_prev.angular_velocity.x, imu_data_prev.angular_velocity.y,
      imu_data_prev.angular_velocity.z);


  angular_vel_prev = angular_vel_prev - gyro_bias_;

  angular_delta = 0.5 * delta_t * (angular_vel_curr + angular_vel_prev);

  angular_vel_mid = 0.5 * (angular_vel_curr + angular_vel_prev);
  return true;
}

inline Eigen::Vector3d OdomImuFlow::GetUnbiasedAngularVel(
    const Eigen::Vector3d &angular_vel, const Eigen::Matrix3d &R) {
  return angular_vel - gyro_bias_;
}



/**
 * @brief  update orientation with effective rotation angular_delta
 * @param  angular_delta, effective rotation
 * @param  R_curr, current orientation
 * @param  R_prev, previous orientation
 * @return void
 */
void OdomImuFlow::UpdateOrientation(
    const Eigen::Vector3d &angular_delta, Eigen::Matrix3d &R_curr,
    Eigen::Matrix3d &R_prev) {
  // magnitude:
  double angular_delta_mag = angular_delta.norm();
  // direction:
  Eigen::Vector3d angular_delta_dir = angular_delta.normalized();

  // build delta q:
  double angular_delta_cos = cos(angular_delta_mag / 2.0);
  double angular_delta_sin = sin(angular_delta_mag / 2.0);
  Eigen::Quaterniond dq(angular_delta_cos,
                        angular_delta_sin * angular_delta_dir.x(),
                        angular_delta_sin * angular_delta_dir.y(),
                        angular_delta_sin * angular_delta_dir.z());
  Eigen::Quaterniond q(pose_.block<3, 3>(0, 0));

  // update:
  q = q * dq;

  // write back:
  R_prev = pose_.block<3, 3>(0, 0);
  pose_.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  R_curr = pose_.block<3, 3>(0, 0);
}


/**
 * @brief  get velocity delta
 * @param  index_curr, current imu measurement buffer index
 * @param  index_prev, previous imu measurement buffer index
 * @param  R_curr, corresponding orientation of current imu measurement
 * @param  R_prev, corresponding orientation of previous imu measurement
 * @param  velocity_delta, velocity delta output
 * @param  linear_acc_mid, mid-value unbiased linear acc
 * @return true if success false otherwise
 */
bool OdomImuFlow::GetVelocityDelta(
    const size_t index_curr, const size_t index_prev,
    const Eigen::Matrix3d &R_curr, const Eigen::Matrix3d &R_prev, double &T,
    Eigen::Vector3d &velocity_delta, Eigen::Vector3d &linear_acc_mid) {
  if (index_curr <= index_prev || imu_data_buff_.size() <= index_curr) {
    return false;
  }

  const sensor_msgs::Imu &imu_data_curr = imu_data_buff_.at(index_curr);
  const sensor_msgs::Imu &imu_data_prev = imu_data_buff_.at(index_prev);

  T = imu_data_curr.header.stamp.toSec() - imu_data_prev.header.stamp.toSec();

  Eigen::Vector3d linear_acc_curr = Eigen::Vector3d(
      imu_data_curr.linear_acceleration.x, imu_data_curr.linear_acceleration.y,
      imu_data_curr.linear_acceleration.z);
  Eigen::Vector3d  a_curr = GetUnbiasedLinearAcc(linear_acc_curr, R_curr);        //  w系下的a_curr
  Eigen::Vector3d linear_acc_prev = Eigen::Vector3d(
      imu_data_prev.linear_acceleration.x, imu_data_prev.linear_acceleration.y,
      imu_data_prev.linear_acceleration.z);
  Eigen::Vector3d  a_prev = GetUnbiasedLinearAcc(linear_acc_prev, R_prev);        //  w系下的a_prev
  // mid-value acc can improve error state prediction accuracy:
  linear_acc_mid = 0.5 * (a_curr + a_prev);     //  w 系下的linear_acc_mid , 用于更新pos_w 和 vel_w
  velocity_delta = T * linear_acc_mid;

  linear_acc_mid = 0.5 * (linear_acc_curr + linear_acc_prev) - accl_bias_;      //  b 系下的linear_acc_mid

  return true;
}


inline Eigen::Vector3d
OdomImuFlow::GetUnbiasedLinearAcc(const Eigen::Vector3d &linear_acc,
                                             const Eigen::Matrix3d &R) {
//   const double norm_g = (linear_acc - accl_bias_).norm();
//   std::cout << " norm_g : " << norm_g << std::endl;

  static bool initial_gravity = false;
  static bool return_choise = false;
  const Eigen::Vector3d g_(0.0,0.0,linear_acc.norm());      
  

  return R * (linear_acc - accl_bias_) - g_;
//   if(!initial_gravity){
//       if(linear_acc(2) > 5){
//           return_choise = true;
//           initial_gravity = true;
//       }
//       else{
//           return_choise = false;
//           initial_gravity = true;
//       }
//   }
  
//   if(return_choise){
//       std::cout << " acc world : " << R * (linear_acc - accl_bias_) - g_ <<std::endl;
//       return R * (linear_acc - accl_bias_) - g_;
//   }
//   else{
//       std::cout << " acc world : " << R * (linear_acc*9.8 - accl_bias_) - g_ <<std::endl;
//       return R * (linear_acc*9.8 - accl_bias_) - g_;
//   }

}


void OdomImuFlow::UpdatePosition(const double &T, const Eigen::Vector3d &velocity_delta) {
    


    

    // 使用自定义函数将旋转矩阵转换为欧拉角

    pose_.block<3, 1>(0, 3) += T * vel_ + 0.5 * T * velocity_delta;
    vel_ += velocity_delta;

//   std::cout << " pose_ : "  << pose_ << std::endl;
//   std::cout << " vel_ : "  << vel_ << std::endl;
}



bool OdomImuFlow::InitialFilter(){
    // Eigen::Matrix3d C_nb = Eigen::Matrix3d::Identity();        //  body2nav

    // pose_.block<3, 3>(0, 0) = C_nb;
    sensor_msgs::Imu intial_imu = imu_queue_.front();
    Eigen::Quaterniond imu_q(intial_imu.orientation.w,intial_imu.orientation.x,intial_imu.orientation.y,intial_imu.orientation.z);

    // pose_.block<3,3>(0,0) = imu_q.normalized().toRotationMatrix();
    
    Eigen::Matrix3d R0 = imu_q.toRotationMatrix();
    Eigen::Vector3d ypr = R2ypr(R0);


    std::cout << " pitch angle " << ypr.y() << std::endl;
    // initial_pose = true;
    vel_(0) = cur_odom.twist.twist.linear.x;
    

    pose_.block<3,3>(0,0) = R0;

    std::cout << " pose_ " << pose_ << std::endl;
   
    
    imu_data_buff_.clear();
    imu_data_buff_.push_back(cur_imu_sync_raw);                            //   获取IMU数据

    imu_last_time = cur_imu_sync_raw.header.stamp.toSec();

    Eigen::Vector3d linear_acc_init(cur_imu_sync_raw.linear_acceleration.x,
                                    cur_imu_sync_raw.linear_acceleration.y,
                                    cur_imu_sync_raw.linear_acceleration.z);
    Eigen::Vector3d angular_vel_init(cur_imu_sync_raw.angular_velocity.x,
                                     cur_imu_sync_raw.angular_velocity.y,
                                     cur_imu_sync_raw.angular_velocity.z);

                                     
    linear_acc_init =  linear_acc_init - accl_bias_;            //  body 系下
    angular_vel_init = angular_vel_init - gyro_bias_;      // body 系下

    UpdateProcessEquation(linear_acc_init, angular_vel_init);

    has_initial = true;

    return true;
}



void OdomImuFlow::ImuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg)
{
    std::lock_guard<std::mutex> lock(imu_lock_);


    imu_queue_.push_back(*imuMsg);
    

    

    imu_sync_queue_.push_back(*imuMsg);
    
}



void OdomImuFlow::EncoderCallback(const irp_sen_msgs::encoder::ConstPtr &encoderMsg)
{
    std::lock_guard<std::mutex> lock(encoder_lock_);
    nav_msgs::Odometry encoder_odom;

    static bool first_encoder = false;
    static irp_sen_msgs::encoder last_encoder_msg;
    if(!first_encoder){
        last_encoder_msg = *encoderMsg;

        first_encoder = true;
        return;
    }

    static int i = 0;
    i++;
    if(i == 2){
        //左 右 轮的差
        double dt = encoderMsg->header.stamp.toSec() - last_encoder_msg.header.stamp.toSec();
        std::cout << " dt odom 1688 " << dt << std::endl;
        double enc_vel_left = (double)(encoderMsg->left_count - last_encoder_msg.left_count) / Encoder_resolution * M_PI * Encoder_left_wheel_diameter / dt;
        double enc_vel_right = (double)(encoderMsg->right_count - last_encoder_msg.right_count) / Encoder_resolution * M_PI * Encoder_right_wheel_diameter / dt;

        
        double enc_v = (dt == 0)?  0 :0.5 * (enc_vel_left + enc_vel_right);
        double enc_omega = (dt == 0)? 0 : (enc_vel_right - enc_vel_left) / Encoder_wheel_base; 

        double timestamp = 0.5 * (last_encoder_msg.header.stamp.toSec() + encoderMsg->header.stamp.toSec());

        encoder_odom.twist.twist.linear.x = enc_v;

        encoder_odom.twist.twist.angular.z = enc_omega * dt;

        encoder_odom.header.stamp = ros::Time(timestamp);
        
        // std::cout << " enc_v : " << enc_v << std::endl;
        // std::cout << " enc_omega dt : " << enc_omega * dt << std::endl;
        odom_queue_.push_back(encoder_odom);
        last_encoder_msg = *encoderMsg;
        i = 0;
    }
    
    

    
    

}

// odom的回调函数
void OdomImuFlow::OdomCallback(const nav_msgs::Odometry::ConstPtr &odometryMsg)
{
    std::lock_guard<std::mutex> lock(odom_lock_);
    static int div_f = 0;
    if(div_f == 1){
        // static nav_msgs::Odometry last_odom = *odometryMsg;
        // nav_msgs::Odometry _cur_odom = *odometryMsg;
        odom_queue_.push_back(*odometryMsg);
        
     
        div_f = 0;
    }
    else{
        div_f++;
    }

}


