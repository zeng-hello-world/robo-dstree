//
// Created by zzy on 2/5/18.
//

#ifndef NORMAL_VIS_NORMAL_VIS_H
#define NORMAL_VIS_NORMAL_VIS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace Robosense
{
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;
typedef pcl::search::KdTree<pcl::PointXYZ> KdTreeSearchMethod;

class normal_vis
{
public:
    explicit normal_vis();

    void setInputCloud(const PointCloudXYZ::ConstPtr in_point_cloud_ptr);
    void getPointCloudNormal(const float& in_neighbor_radius, visualization_msgs::MarkerArray& out_arrow_marker);
    void showArrowMarker(visualization_msgs::Marker &out_arrow_marker);

private:
    void clearPrePointCloudData();
    void fixLeftEdgeColNANPts();
    void fixCentralNANPts(const float& in_distance_thre);
    void calculatePointNormal(const float& in_neighbor_radius);



private:
    PointCloudXYZ::Ptr origin_cloud_ptr_;
    PointCloudXYZ::Ptr feature_calculate_cloud_ptr_;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
    KdTreeSearchMethod::Ptr kd_tree_method_ptr_;
    PointCloudNormal::Ptr cloud_normal_ptr_;


};
}

#endif //NORMAL_VIS_NORMAL_VIS_H
