#ifndef PRIMESENSE_PKGS_OBJECT_IDENTIFIER_H
#define PRIMESENSE_PKGS_OBJECT_IDENTIFIER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <object_finder/Positions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

#define POINTTYPE pcl::PointXYZRGB

namespace primesense_pkgs{

class ObjectIdentifier
{
public:
    ObjectIdentifier();
//    ~ObjectIdentifier();
    void identifyObjects();

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, object_finder::Positions> syncPolicy;
//    ros::Subscriber cloudSub;
//    ros::Subscriber positionSub;
    ros::Publisher objectPub;
    ros::Publisher debugPub;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloudSub;
    message_filters::Subscriber<object_finder::Positions> *positionSub;
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, object_finder::Positions> *synchronizer;
    message_filters::Synchronizer<syncPolicy> *approxSynchronizer;

    pcl::PointCloud<POINTTYPE>::Ptr inputCloud;

    std::vector<pcl::PointXYZ> objectPositions;
    std::vector<pcl::PointCloud<POINTTYPE>::Ptr> objectClouds;

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void positionCallback(const object_finder::Positions::ConstPtr &msg);
    void cloudPositionCallback(const sensor_msgs::PointCloud2::ConstPtr &cloudMsg, const object_finder::Positions::ConstPtr &posMsg);
    void approxCallback(const sensor_msgs::PointCloud2::ConstPtr &cloudMsg, const object_finder::Positions::ConstPtr &posMsg);
    void extractObjectClouds();

};
} //namespace primesense_pkgs

#endif // PRIMESENSE_PKGS_OBJECT_IDENTIFIER_H
