//
// Created by tao on 7/21/21.
//

#include "encoder_converter.h"

#include <rosbag/bag.h>
#include <irp_sen_msgs/encoder.h>
#include <sensor_msgs/JointState.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

namespace kaist2bag {


// EncoderConverter::EncoderConverter(const std::string &dataset_dir, const std::string &save_dir,
//                                    const std::string &irp_topic, const std::string &raw_topic)
//                                    : Converter(dataset_dir, save_dir), irp_topic_(irp_topic), raw_topic_(raw_topic) {
//     irp_bag_name_ = FilterSlash(irp_topic_) + ".bag";
//     raw_bag_name_ = FilterSlash(raw_topic_) + ".bag";
// }

EncoderConverter::EncoderConverter(const std::string &dataset_dir, const std::string &save_dir,
                                   const std::string &wheel_odom_topic)
                                   : Converter(dataset_dir, save_dir), wheel_odom_topic_(wheel_odom_topic) {
    // irp_bag_name_ = FilterSlash(irp_topic_) + ".bag";
    // raw_bag_name_ = FilterSlash(raw_topic_) + ".bag";
    wheel_odom_bag_name_ = FilterSlash(wheel_odom_topic_) + ".bag";
}


int EncoderConverter::Convert() {
    CheckAndCreateSaveDir();

    // boost::filesystem::path irp_bag_file = boost::filesystem::path(save_dir_) / irp_bag_name_;
    // boost::filesystem::path raw_bag_file = boost::filesystem::path(save_dir_) / raw_bag_name_;
    // rosbag::Bag irp_bag(irp_bag_file.string(), rosbag::bagmode::Write);
    // rosbag::Bag raw_bag(raw_bag_file.string(), rosbag::bagmode::Write);
    // ROS_INFO("saving %s", irp_bag_file.c_str());
    // ROS_INFO("saving %s", raw_bag_file.c_str());

    boost::filesystem::path wheel_odom_bag_file = boost::filesystem::path(save_dir_) / wheel_odom_bag_name_;
    rosbag::Bag wheel_odom_bag(wheel_odom_bag_file.string(), rosbag::bagmode::Write);
    ROS_INFO("saving %s", wheel_odom_bag_file.c_str());

    // sensor_msgs::JointState joint_data;
    // joint_data.name.push_back("front_left_wheel_joint");
    // joint_data.name.push_back("front_right_wheel_joint");
    // joint_data.name.push_back("back_left_wheel_joint");
    // joint_data.name.push_back("back_right_wheel_joint");
    // joint_data.position.push_back(0.0);
    // joint_data.position.push_back(0.0);
    // joint_data.position.push_back(0.0);
    // joint_data.position.push_back(0.0);
    // joint_data.velocity.push_back(0.0);
    // joint_data.velocity.push_back(0.0);
    // joint_data.velocity.push_back(0.0);
    // joint_data.velocity.push_back(0.0);
    bool has_previous = true;
    bool encoder_param_load_flag_ = false;
    int64_t pre_stamp = 0;
    int64_t current_stamp = 0;
    int64_t pre_left_count = 0;
    int64_t pre_right_count = 0;
    int64_t stamp, left_count, right_count;

    int encoder_resolution_ = 0;
    double encoder_left_diameter_ = 0.0;
    double encoder_right_diameter_ = 0.0;
    double encoder_wheel_base_ = 0.0;
    double encoder_x_ = 0.0;
    double encoder_y_ = 0.0;
    double encoder_theta_ = 0.0;
    const std::string calib_file = dataset_dir_ + "/" + default_calib_file;
    std::ifstream file(calib_file.c_str());
    std::string str;
    while (std::getline(file, str)) {
        std::vector<std::string> strs;
        boost::split(strs, str, boost::is_any_of(" "));
        if(!strs[1].compare("resolution:")){
            encoder_resolution_ = std::stoi(strs[2]);
        }
        if(!strs[1].compare("left")){
            encoder_left_diameter_ = std::stod(strs[4]);
        }
        if(!strs[1].compare("right")){
            encoder_right_diameter_ = std::stod(strs[4]);
        }
        if(!strs[1].compare("wheel")){
            encoder_wheel_base_ = std::stod(strs[3]);
        }
    }

    if(encoder_resolution_ != 0)
    {
        encoder_param_load_flag_ = true;
    }

    const std::string data_file = dataset_dir_ + "/" + default_data_file;
    FILE* fp = fopen(data_file.c_str(), "r");
    // irp_sen_msgs::encoder encoder_data;
    
    while (fscanf(fp, "%ld,%ld,%ld\n", &stamp, &left_count, &right_count) == 3) {
        // encoder_data.header.stamp.fromNSec(stamp);
        // encoder_data.header.frame_id = "encoder";
        // encoder_data.left_count = left_count;
        // encoder_data.right_count = right_count;
        // irp_bag.write(irp_topic_, encoder_data.header.stamp, encoder_data);

        if(pre_stamp == 0)
        {
            pre_stamp = stamp - 10000000;
        }

        if(pre_left_count == 0 && pre_right_count == 0){
            pre_left_count = left_count;
            pre_right_count = right_count;
        }

        current_stamp = stamp;


        if(encoder_param_load_flag_) {

            int64_t d_left_cnt = left_count - pre_left_count;
            int64_t d_right_cnt = right_count - pre_right_count;

            double left_distnace = ((double)d_left_cnt/(double)encoder_resolution_)*encoder_left_diameter_*M_PI;
            double right_distance = ((double)d_right_cnt/(double)encoder_resolution_)*encoder_right_diameter_*M_PI;
            double stamp_diff = static_cast<double>(current_stamp - pre_stamp);
            double dx = (left_distnace + right_distance)*0.5;
            double dy = 0.0;
            double dtheta = (right_distance - left_distnace)/encoder_wheel_base_;
            double vx = dx/stamp_diff;
            double vy = dy/stamp_diff;
            double vth = dtheta/stamp_diff;

            double delta_x = (dx * cos(encoder_theta_));
            double delta_y = (dx * sin(encoder_theta_));
            double delta_th = dtheta;

            encoder_x_ += delta_x;
            encoder_y_ += delta_y;
            encoder_theta_ += delta_th;
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(encoder_theta_);
            //Odometry message
            nav_msgs::Odometry odom;
            odom.header.stamp.fromNSec(current_stamp);
            odom.header.frame_id = "map";

            //set the position
            odom.pose.pose.position.x = encoder_x_;
            odom.pose.pose.position.y = encoder_y_;;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;

            //pose covariance (6x6)
            odom.pose.covariance[0] = 1;
            odom.pose.covariance[7] = 1;
            odom.pose.covariance[14] = 1;
            odom.pose.covariance[21] = 1;
            odom.pose.covariance[28] = 1;
            odom.pose.covariance[35] = 1;
            //twist covariance(6x6)
            odom.twist.covariance[0] = 1;
            odom.twist.covariance[7] = 1;
            odom.twist.covariance[14] = 1;
            odom.twist.covariance[21] = 1;
            odom.twist.covariance[28] = 1;
            odom.twist.covariance[35] = 1;

            //set the velocity
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = vy;
            odom.twist.twist.angular.z = vth;

            wheel_odom_bag.write(wheel_odom_topic_, odom.header.stamp, odom);
            // double wl = degl / stamp_diff;
            // double wr = degr / stamp_diff;
            
            // joint_data.header.stamp.fromNSec(stamp);
            // joint_data.header.frame_id = "encoder";
            // joint_data.position[0] += degl;
            // joint_data.position[1] += degr;
            // joint_data.position[2] += degl;
            // joint_data.position[3] += degr;
            // joint_data.velocity[0] = wl;
            // joint_data.velocity[1] = wr;
            // joint_data.velocity[2] = wl;
            // joint_data.velocity[3] = wr;
            // raw_bag.write(raw_topic_, joint_data.header.stamp, joint_data);

        }
        // has_previous = true;
        pre_stamp = current_stamp;
        pre_left_count = left_count;
        pre_right_count = right_count;

    }
    // irp_bag.close();
    // raw_bag.close();
    wheel_odom_bag.close();
    fclose(fp);
    ROS_INFO("done saving %s", wheel_odom_bag_file.c_str());
    // ROS_INFO("done saving %s", raw_bag_file.c_str());

    return 0;
}
} // namespace kaist2bag

