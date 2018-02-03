//
// Created by zzy on 2/1/18.
//

#ifndef GROUND_REMOVAL_GRID_CLASS_H
#define GROUND_REMOVAL_GRID_CLASS_H

#include <limits>
#include <fstream>
#include <sstream>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>

#include "opencv2/core/core.hpp"
#include "opencv2/ml/ml.hpp"

#include <pcl/kdtree/kdtree.h>

namespace Robosense
{
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCLoudXYZRGB;

enum g_calculate_grid_cell_method{ differ_height_method = 1, sigma_height_method = 2, average_height_method = 3, RANSAC_plane_estimate = 4 };
enum g_point_property{ unknown_mark = -1, ground_mark = 0, obstacle_mark = 1 };
///----------- point data structure -----------///
struct pointFlag
{
    size_t grid_x;
    size_t grid_y;
    bool point_property;
};

struct pointFeatureStruc
{
    float range;                     //beam_range_
    float remission;                 //beam_remission_
    float left_minus;                //left_beam_range_minus_beam_range_
    float right_minus;               //right_beam_range_minus_beam_range_
    float top_minus;                 //top_beam_range_minus_beam_range_
    float bottom_minus;              //bottom_beam_range_minus_beam_range_
    float lidar_height;              //distance_below_sensor_
    float height_above_lowest;       //height_above_lowest_measurement_in_grid_cell_
};

struct PointInfo
{
public:
    bool isEmpty()
        {  if ( feature.range == 0 || isnan( feature.range ) || feature.range > 200)  return true; }
    size_t feature_size()
        { return 8; }

    pointFlag flag;
    pointFeatureStruc feature;
};

///----------- Grid map data structure ---------///
struct lidarParamter
{
    double height;
    double row_size;
    double col_size;
    double range;
};

struct posIndics
{
    size_t row;
    size_t col;
};

struct statisticInfo
{
    float mu_g;     //average height of ground hits in global coordinates
    float delta_g;  //variance of ground hit heights
    float mu_o ;    //average of obstacle hit in global coordinates
    float delta_o;  //variance of obstacle hit heights
    float mu_t;     //ground height as measured by known tire position
    float delta_t;  //variance of tire hit heights
    size_t obstacle_point_size;
    size_t ground_point_size;
};

struct GridCell
{
    bool isEmpty()
         { if ( ptsContainer.empty() ) return true;  }

    size_t grid_x_num_, grid_y_num_;
    PointCloudXYZ ptsContainer;
    std::vector< posIndics > contain_pts_;
    float lowest_height_;
    bool property;
    statisticInfo stInfo;
};

///--------------------- train data structure ---------------------------
struct trainDataLine
{
    float data[9];
};

struct predictDataStruc
{
    posIndics index;
    pointFeatureStruc feature;
    bool property;
};
///=====================================================================///

class grid_map
{
public:

    explicit grid_map();

    void setInputCloud(const PointCloudXYZ::ConstPtr in_point_cloud_ptr);
    void getGidCell(const size_t& in_index_x, const size_t& in_index_y, GridCell out_grid_cell);
    void setLidarParams(const float& in_lidar_height = 1.8, const float& in_lidar_range = 50.0 );
    void classifyOriginCloudWithWeakClassifier(const size_t& in_classify_method);
    void showCloudInRvizInterface( PointCloudXYZ::Ptr out_show_cloud );
    void generateOpenCVDescionTreeTrainData();
    float evaluateDSTreePredictAccuracy( cv::Mat& predicted, cv::Mat& actual );
    void loadDStreeXMLFile( const char in_load_file[] );
    void useDStreePredictPointCloud( PointCloudXYZ::Ptr out_cloud_ptr );

private:
    void initialDataSize();
    void clearPrePtsCldData();
    void generateOneMeterGridMap();
    void initialGridMapRelatedDataStruc();
    void calculateLowestHeightInGridCell();
    void fixLeftEdgeColNANPts();
    void fixCentralNANPts(const float& in_distance_thre);
    void calculatePtsCloudFeature();
    void calculateGridPropertyInHeightDifferMethod(const float& in_differ_height_thre = 0.2, const float& in_lowest_height_thre = -1.8);
    void classifyCloudToGroundAndObstacleCloud();
    void calculatePointProperty();


private:
    lidarParamter rslidar_;
    float lowest_h_thre_;
    float differ_h_thre;
    PointCloudXYZ::Ptr origin_cloud_ptr_;
    PointCloudXYZ::Ptr feature_calculate_cloud_ptr_;
    std::vector< std::vector< PointInfo > > point_info_vec_;
    bool is_initial_;

    size_t grid_size_; // 100
    float grid_cell_length_;
    std::vector< std::vector<GridCell> > grid_info_vec_; // 100*100
    float diff_height_thre_;
    float lowest_height_thre_;
    PointCloudXYZ::Ptr clsf_ground_cloud_ptr_;
    PointCloudXYZ::Ptr clsf_obstacle_cloud_ptr_;

    CvDTree *ds_tree_;
    std::vector<predictDataStruc> under_predict_point_vec_;
    PointCloudXYZ::Ptr predict_ground_cloud_ptr_;
    PointCloudXYZ::Ptr predict_obstacle_cloud_ptr_;
};
}
#endif //GROUND_REMOVAL_GRID_CLASS_H
