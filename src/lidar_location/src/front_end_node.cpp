#include <ros/ros.h>
#include <lidar_location/saveMap.h>
#include "lidar_localization/front_end/front_end_flow.hpp"

using namespace lidar_localization;

std::shared_ptr<FrontEndFlow> _front_end_flow_ptr;

bool save_map_callback(lidar_location::saveMap::Request &request, lidar_location::saveMap::Response &response) {
    response.succeed = _front_end_flow_ptr->SaveMap();
    return response.succeed;
}



int main(int argc, char *argv[]) {
    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;
   
    ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);
    ros::Rate rate(100);
    _front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);
    while (ros::ok()) {
        ros::spinOnce();
        _front_end_flow_ptr->Run();
        rate.sleep();
    }
    return 0;
}
