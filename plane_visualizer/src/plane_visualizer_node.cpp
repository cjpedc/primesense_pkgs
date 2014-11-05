#include <ros/ros.h>

//messages
#include <sensor_msgs/PointCloud2.h>
#include <plane_segment/CloudPlanes.h>
#include <plane_segment/PlaneModel.h>

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

//PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <vector>

#define POINTTYPE pcl::PointXYZRGB

namespace PlaneVisualizer{

using message_filters::Subscriber;
using message_filters::TimeSynchronizer;
using sensor_msgs::PointCloud2;
using plane_segment::CloudPlanes;

class PlaneVisualizer{

private:
    Subscriber<PointCloud2>* cloudSub;
    Subscriber<CloudPlanes>* planeSub;
    TimeSynchronizer<PointCloud2, CloudPlanes>* synchronizer;
    ros::Publisher pub;

    pcl::PointCloud<POINTTYPE>::Ptr cloud;

    std::vector<Eigen::VectorXf> planeModels;

public:
    PlaneVisualizer(){

        ros::NodeHandle nh;

        cloudSub = new Subscriber<PointCloud2>(nh, "/camera/depth_registered/points", 1);
        planeSub = new Subscriber<CloudPlanes>(nh, "/plane_segment/planes", 1);

        synchronizer = new TimeSynchronizer<PointCloud2, CloudPlanes>(*cloudSub, *planeSub, 20);
        synchronizer->registerCallback(&PlaneVisualizer::cloudPlaneCallback, this);

        pub = nh.advertise<sensor_msgs::PointCloud2>("/plane_visualizer/cloud", 1);

        cloud = pcl::PointCloud<POINTTYPE>::Ptr(new pcl::PointCloud<POINTTYPE>);
    }

    ~PlaneVisualizer(){
        delete cloudSub;
        delete planeSub;
        delete synchronizer;
    }

    void cloudPlaneCallback(const PointCloud2::ConstPtr& cloudMsg, const CloudPlanes::ConstPtr& planeMsg){

        pcl::fromROSMsg(*cloudMsg, *cloud);

        planeModels = std::vector<Eigen::VectorXf>(planeMsg->planes.size());
        for(size_t i = 0; i < planeMsg->planes.size(); i++){
            Eigen::VectorXf modelVec(4);
            modelVec[0] = planeMsg->planes[i].xParam;
            modelVec[1] = planeMsg->planes[i].yParam;
            modelVec[2] = planeMsg->planes[i].zParam;
            modelVec[3] = planeMsg->planes[i].dParam;

            planeModels[i] = modelVec;
        }
    }

    void colorPlanes(){

        std::vector<uint32_t> colors(6, 0);
        colors[0] = ((uint32_t)255 << 16 | (uint32_t)0 << 8 | (uint32_t)0);
        colors[1] = ((uint32_t)0 << 16 | (uint32_t)255 << 8 | (uint32_t)0);
        colors[2] = ((uint32_t)0 << 16 | (uint32_t)0 << 8 | (uint32_t)255);
        colors[3] = ((uint32_t)255 << 16 | (uint32_t)255 << 8 | (uint32_t)0);
        colors[4] = ((uint32_t)255 << 16 | (uint32_t)0 << 8 | (uint32_t)255);
        colors[5] = ((uint32_t)0 << 16 | (uint32_t)255 << 8 | (uint32_t)255);

        size_t colorIndex = 0;

        for(size_t i = 0; i < planeModels.size(); i++){

            std::vector<int> planeIndices;

            pcl::SampleConsensusModelPlane<POINTTYPE> scmp(cloud);
            scmp.selectWithinDistance(planeModels[i], 0.02, planeIndices);

            uint32_t rgb = colors[(colorIndex++) % colors.size()];

            //color detected plane
            for(size_t j = 0; j < planeIndices.size(); j++){
                cloud->points[planeIndices[j]].rgb = *reinterpret_cast<float*>(&rgb);
            }
        }

        sensor_msgs::PointCloud2 coloredCloudMsg;
        pcl::toROSMsg(*cloud, coloredCloudMsg);
        pub.publish(coloredCloudMsg);

    }
};
} //namespace PlaneVisualizer

int main(int argc, char **argv){

    ros::init(argc, argv, "plane_visualizer_node");

    PlaneVisualizer::PlaneVisualizer visualizer;

    ros::Rate loop_rate(10);

    while(ros::ok()){

        ros::spinOnce();

        visualizer.colorPlanes();

        loop_rate.sleep();
    }
    return 0;
}
