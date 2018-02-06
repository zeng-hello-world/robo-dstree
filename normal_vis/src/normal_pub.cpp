//
// Created by zzy on 2/5/18.
//

#include "ros/ros.h"
#include "normal_vis.h"


static Robosense::normal_vis *test_cloud = new Robosense::normal_vis;
static Robosense::PointCloudNormal::Ptr temp_show_normal_ptr ( new Robosense::PointCloudNormal );
static ros::Publisher g_normal_vis_pub;
static visualization_msgs::MarkerArray g_normal_arrow_array;
static visualization_msgs::Marker g_normal_arrow;
const float g_neighbor_radius = 0.5;

void NormalVis( Robosense::PointCloudXYZ::Ptr in_cloud_ptr  )
{
    in_cloud_ptr->header.frame_id = "rslidar";
    test_cloud->setInputCloud( in_cloud_ptr );
    test_cloud->getPointCloudNormal( g_neighbor_radius, g_normal_arrow_array );

    std::cout << g_normal_arrow_array.markers.size() << std::endl;

    g_normal_vis_pub.publish( g_normal_arrow_array );
    g_normal_arrow_array.markers.clear();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "normal_pub");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe( "rslidar_points", 1, NormalVis );
    g_normal_vis_pub = n.advertise< visualization_msgs::MarkerArray >("chatter2", 2);
    ros::spin();

    return 0;
}


