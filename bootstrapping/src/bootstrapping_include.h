//
// Created by zzy on 1/25/18.
//

#ifndef BOOTSTRAPPING_BOOTSTRAAPPING_INCLUDE_H
#define BOOTSTRAPPING_BOOTSTRAAPPING_INCLUDE_H

#include <string.h>
#include <vector>
#include <limits>
#include <string>
#include <math.h>
#include <fstream>
#include <stdio.h>
#include <sstream>
#include <ctime>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include "opencv2/core/core.hpp"
#include "opencv2/ml/ml.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCLoudXYZRGB;

const double Pi = 3.141592654;
const double g_grid_length_type[5] = {0.1, 0.2, 0.25, 0.5, 1.0};
enum g_calculate_grid_cell_method{ differ_height_method = 1, sigma_height_method = 2, average_height_method = 3, RANSAC_plane_estimate = 4 };
enum g_grid_cell_property{ unknown_cell = -1, ground_cell = 0, obstacle_cell = 1 };
enum g_point_property{ ground_point = 0, obstacle_point = 1 };

struct LidarParamter
{
    double height;
    double row_size;
    double column_size;
    double visibility_range;
};

struct PointFlag
{
    unsigned int x_grid_num;
    unsigned int y_grid_num;
    bool point_property;
};

struct PointFeatureStruct
{
public:
    bool isEmpty()
        {  if ( self_range == 0 || isnan( self_range ) || self_range > 200)  return true; }
    unsigned int feature_size()
        { return 8; }

public:
    float self_range;               //beam_range_
    float remission;                //beam_remission_
    float left_minus_self;          //left_beam_range_minus_beam_range_
    float right_minus_self;         //right_beam_range_minus_beam_range_
    float top_minus_self;           //top_beam_range_minus_beam_range_
    float bottom_minus_self;        //bottom_beam_range_minus_beam_range_
    float lidar_height;             //distance_below_sensor_
    float height_above_lowest;      //height_above_lowest_measurement_in_grid_cell_
};

struct GridPointsPropertyContainer
{
    std::vector<pcl::PointXYZ> ground_points;
    std::vector<pcl::PointXYZ> obstacle_points;
};

struct GridStatisticFeatureStruct
{
public:
    bool unReachable() {  if ( delta_g == -1 && delta_o == -1 )  return true; }
    bool noGroundPoint() { if ( delta_g == -1 && delta_o != -1 ) return true; }
    bool noObstaclePoint() { if ( delta_g != -1 && delta_o == -1 ) return true; }

public:
    double mu_g;     //average height of ground hits in global coordinates
    double delta_g;  //variance of ground hit heights
    double mu_o ;    //average of obstacle hit in global coordinates
    double delta_o;  //variance of obstacle hit heights
    double mu_t;     //ground height as measured by known tire position
    double delta_t;  //variance of tire hit heights
    unsigned int obstacle_point_size;
    unsigned int ground_point_size;
};

#endif //BOOTSTRAPPING_BOOTSTRAAPPING_INCLUDE_H
