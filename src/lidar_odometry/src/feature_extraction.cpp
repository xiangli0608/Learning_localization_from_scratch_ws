#include <cmath>
#include <vector>
#include <string>
#include <deque>
#include <thread>
#include <mutex>

#include <opencv/cv.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include "feature_extraction.h"


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

int cloudSortInd[400000];
//曲率存放
double cloudCurv[400000];

double cloudPicked[400000];

bool sort_value (int i,int j) { return (cloudCurv[i]<cloudCurv[j]); }

class FeatureExtraction
{
public:

    ros::NodeHandle nh;

    std::mutex veloLock;

    int N_SCAN;
    ros::Subscriber left_cloud_sub;
    ros::Publisher pubSurfacePoints;
    pcl::PointCloud<PointXYZIR>::Ptr pointCloudLeftIn;
    pcl::PointCloud<PointXYZIR>::Ptr surfaceCloud;
    pcl::PointCloud<PointXYZIR>::Ptr laserCloudAll;
    std_msgs::Header cloudHeader;
    float surfThreshold;

public:
    FeatureExtraction()
    {
        left_cloud_sub = nh.subscribe("/ns2/velodyne_points", 100, &FeatureExtraction::left_cloud_handler, this, ros::TransportHints().tcpNoDelay());

        // 发布面特征点云
        pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("/feature/cloud_surface", 100);

        initParameters();
        resetParameters();
    }

    void initParameters()
    {
        N_SCAN = 16;
        surfThreshold = 0.1;
        pointCloudLeftIn.reset(new pcl::PointCloud<PointXYZIR>());
        surfaceCloud.reset(new pcl::PointCloud<PointXYZIR>());
        laserCloudAll.reset(new pcl::PointCloud<PointXYZIR>());
        resetParameters();
    }

    void resetParameters()
    {
        pointCloudLeftIn->clear();
    }

    void left_cloud_handler(const sensor_msgs::PointCloud2 left_cloud)
    {

        cloudHeader = left_cloud.header;

        pcl::fromROSMsg(left_cloud, *pointCloudLeftIn);

        int cloudSize = pointCloudLeftIn->size();

        // 存储每线点云
        std::vector<pcl::PointCloud<PointXYZIR>> laserCloudScans(N_SCAN);

        laserCloudAll->clear();
        surfaceCloud->clear();

        for(int i = 0; i < cloudSize; ++i)
        {
            PointXYZIR thisPoint;
            thisPoint.x = pointCloudLeftIn->points[i].x;
            thisPoint.y = pointCloudLeftIn->points[i].y;
            thisPoint.z = pointCloudLeftIn->points[i].z;
            thisPoint.intensity = pointCloudLeftIn->points[i].intensity;
    
            float angle = atan(thisPoint.z / sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;

            int scanID = 0;
            scanID = int((angle + 15) / 2 + 0.5);
            
            if(scanID < 0 || scanID > N_SCAN-1)
            {
                continue;
            }

            thisPoint.ring = scanID;
            laserCloudScans[scanID].push_back(thisPoint);

        }

        long ind_count = 0;
        int cloudScanEndInd[N_SCAN+1];
        cloudScanEndInd[0] = 0;

        for(int i = 0; i < N_SCAN; i++)
        {
            for(int j = 5; j < int(laserCloudScans[i].size()) - 5; j++)
            {
                float diffX = laserCloudScans[i].points[j - 5].x + laserCloudScans[i].points[j - 4].x + laserCloudScans[i].points[j - 3].x + laserCloudScans[i].points[j - 2].x + laserCloudScans[i].points[j - 1].x - 10 * laserCloudScans[i].points[j].x + laserCloudScans[i].points[j + 1].x + laserCloudScans[i].points[j + 2].x + laserCloudScans[i].points[j + 3].x + laserCloudScans[i].points[j + 4].x + laserCloudScans[i].points[j + 5].x;
                float diffY = laserCloudScans[i].points[j - 5].y + laserCloudScans[i].points[j - 4].y + laserCloudScans[i].points[j - 3].y + laserCloudScans[i].points[j - 2].y + laserCloudScans[i].points[j - 1].y - 10 * laserCloudScans[i].points[j].y + laserCloudScans[i].points[j + 1].y + laserCloudScans[i].points[j + 2].y + laserCloudScans[i].points[j + 3].y + laserCloudScans[i].points[j + 4].y + laserCloudScans[i].points[j + 5].y;
                float diffZ = laserCloudScans[i].points[j - 5].z + laserCloudScans[i].points[j - 4].z + laserCloudScans[i].points[j - 3].z + laserCloudScans[i].points[j - 2].z + laserCloudScans[i].points[j - 1].z - 10 * laserCloudScans[i].points[j].z + laserCloudScans[i].points[j + 1].z + laserCloudScans[i].points[j + 2].z + laserCloudScans[i].points[j + 3].z + laserCloudScans[i].points[j + 4].z + laserCloudScans[i].points[j + 5].z;
                cloudCurv[ind_count]=(diffX * diffX + diffY * diffY + diffZ * diffZ);
                cloudPicked[ind_count] = 0;
                laserCloudAll->points.push_back(laserCloudScans[i].points[j]);
                ind_count++;
                cloudSortInd[ind_count] = ind_count;
            }
            cloudScanEndInd[i+1] = laserCloudAll->points.size();
        }

        int laser_num = laserCloudAll->points.size();

        // // 将每条线分成6块
        for(int i = 0; i < N_SCAN; i++)
        {
            // 每条线起止
            int sp = cloudScanEndInd[i];
            int ep = cloudScanEndInd[i+1];

            for(int j = 0; j < 6; j++)
            {
                int start = sp + ((ep - sp) / 6) * j;
                int end = sp + ((ep - sp) / 6) * (j + 1);

                std::sort (cloudSortInd + start, cloudSortInd + end, sort_value);

                int surface_num = 0;

                for(int k = start; k < end && surface_num < 5; k++)
                {
                    long ind = cloudSortInd[k];

                    if(cloudPicked[ind] == 0 && cloudCurv[ind] < surfThreshold)
                    {
                        surface_num++;
                        surfaceCloud->push_back(laserCloudAll->points[ind]);

                        // 临近点不可选
                        for (int l = 1; l <= 5; l++)
                        {
                            cloudPicked[ind + l] = 1;
                        }

                        for (int l = -1; l >= -5; l--)
                        {
                            cloudPicked[ind + l] = 1;
                        }
                    }
                }
            }
        }

        sensor_msgs::PointCloud2 laserCloudSurface;
        pcl::toROSMsg(*surfaceCloud, laserCloudSurface);
        laserCloudSurface.header.stamp = cloudHeader.stamp;
        laserCloudSurface.header.frame_id = "base_link";
        pubSurfacePoints.publish(laserCloudSurface);

        resetParameters();
    }

};


int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "feature_extraction");

    FeatureExtraction FE;

    ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");
    
    ros::spin();

    return 0;
}
