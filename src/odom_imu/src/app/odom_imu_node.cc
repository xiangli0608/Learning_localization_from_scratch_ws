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

#include "tools/odom_imu_flow.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_imu_node");

    OdomImuFlow odom_imu_flow_;

    // 开启3个线程同时工作
    // ros::AsyncSpinner spinner(3);
    // spinner.start();
    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();

        // odom_imu_flow_.fusion2_Run();
        odom_imu_flow_.fusion2_Run();

        rate.sleep();
    }

    // ros::waitForShutdown();
    return (0);
}