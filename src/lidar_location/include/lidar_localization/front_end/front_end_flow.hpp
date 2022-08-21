#ifndef LIDAR_LOCALIZATION_FRONT_END_FRONT_END_FLOW_HPP_
#define LIDAR_LOCALIZATION_FRONT_END_FRONT_END_FLOW_HPP_

#include <pcl/common/transforms.h>
#include <cmath>
#include <ros/ros.h>

#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"

#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/front_end/front_end.hpp"

namespace lidar_localization {
class FrontEndFlow {
  public:
    FrontEndFlow(ros::NodeHandle& nh);
    bool Run();
    bool SaveMap();

  private:
    bool ReadData();
    bool Workflow();
    bool WorkflowWithMap();

  private:
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr;
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_ns2;
    std::shared_ptr<IMUSubscriber> imu_sub_ptr;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr;

    std::shared_ptr<CloudPublisher> cloud_pub_ptr;
    std::shared_ptr<CloudPublisher> local_map_pub_ptr;
    std::shared_ptr<CloudPublisher> global_map_pub_ptr;
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr;
    std::shared_ptr<FrontEnd> front_end_ptr;

    std::deque<CloudData> cloud_data_buff;
    std::deque<CloudData> cloud_data_buff_ns2;
    std::deque<IMUData> imu_data_buff;
    std::deque<GNSSData> gnss_data_buff;

    Eigen::Matrix4f vehilcle_to_ns1;
    Eigen::Matrix4f vehilcle_to_ns2;
    Eigen::Matrix4f vehilcle_to_imu;
    Eigen::Matrix4f ns2_to_ns1;
    Eigen::Matrix4f ns1_to_imu;
    Eigen::Matrix4f ns2_to_imu;
    bool front_end_pose_inited;
    CloudData::CLOUD_PTR local_map_ptr;
    CloudData::CLOUD_PTR global_map_ptr;
    CloudData::CLOUD_PTR current_scan_ptr;
    double run_time;
    double init_time;
    bool time_inited;
    bool has_global_map_published;
};
}

#endif


