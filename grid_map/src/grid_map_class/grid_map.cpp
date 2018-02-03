//
// Created by zzy on 2/1/18.
//

#include <iostream>
#include "grid_map.h"

Robosense::grid_map::grid_map( )
{
    origin_cloud_ptr_.reset( new PointCloudXYZ );
    feature_calculate_cloud_ptr_.reset( new PointCloudXYZ );
    clsf_ground_cloud_ptr_.reset( new PointCloudXYZ );
    clsf_obstacle_cloud_ptr_.reset( new PointCloudXYZ );

    ds_tree_= new CvDTree;
    predict_ground_cloud_ptr_.reset( new PointCloudXYZ );
    predict_obstacle_cloud_ptr_.reset( new PointCloudXYZ );
}

void Robosense::grid_map::setInputCloud(const PointCloudXYZ::ConstPtr in_point_cloud_ptr)
{
    this->clearPrePtsCldData();

    *origin_cloud_ptr_ = *in_point_cloud_ptr;

    rslidar_.row_size = origin_cloud_ptr_->height;
    rslidar_.col_size = origin_cloud_ptr_->width; // may vary between frames
    this->setLidarParams();
    this->initialDataSize();
}

void Robosense::grid_map::getGidCell(const size_t& in_index_x, const size_t& in_index_y, GridCell out_grid_cell)
{
    if ( in_index_x >= 0 && in_index_x < grid_size_ && in_index_y >= 0 && in_index_y < grid_size_ )
    {
        if (!grid_info_vec_[in_index_x][in_index_y].isEmpty())
            out_grid_cell = grid_info_vec_[in_index_x][in_index_y];
    }
    else
        std::cerr << "Grid index out of range!" << std::endl;
}

void Robosense::grid_map::setLidarParams(const float &in_lidar_height, const float &in_lidar_range)
{
    rslidar_.height = in_lidar_height;
    rslidar_.range = in_lidar_range;
}

void Robosense::grid_map::classifyOriginCloudWithWeakClassifier(const size_t &in_classify_method)
{
    if (!is_initial_)
    { PCL_ERROR("Cloud points is empty! Try again!");  return;  }

    this->generateOneMeterGridMap();
    this->calculateLowestHeightInGridCell();
    this->calculatePtsCloudFeature();
    if ( in_classify_method == differ_height_method )
    {
        this->calculateGridPropertyInHeightDifferMethod( 0.2, -rslidar_.height + 0.3 );
    }

    this->classifyCloudToGroundAndObstacleCloud();
    this->calculatePointProperty();
}

void Robosense::grid_map::showCloudInRvizInterface(pcl::PointCloud<pcl::PointXYZ>::Ptr out_show_cloud)
{
    *out_show_cloud = *clsf_ground_cloud_ptr_;
}

void Robosense::grid_map::generateOpenCVDescionTreeTrainData()
{
    predictDataStruc temp_predict_point;
    for (int i = 0; i < point_info_vec_.size(); ++i)
    {
        for (int j = 0; j < point_info_vec_[i].size(); ++j)
        {
            if (!point_info_vec_[i][j].isEmpty())
            {
                temp_predict_point.feature = point_info_vec_[i][j].feature;
                temp_predict_point.index.row = i;
                temp_predict_point.index.col = j;
                under_predict_point_vec_.push_back( temp_predict_point );
            }
        }
    }
}

float Robosense::grid_map::evaluateDSTreePredictAccuracy(cv::Mat &predicted, cv::Mat &actual)
{
    assert(predicted.rows == actual.rows);
    int t = 0;
    int f = 0;
    for(int i = 0; i < actual.rows; i++) {
        float p = predicted.at<float>(i,0);
        float a = actual.at<float>(i,0);
        if((p == 0.0 && a == 0.0) || (p == 1.0 &&  a == 1.0))
        {
            t++;
        }
        else
        {
            f++;
        }
    }
    return (t * 1.0) / (t + f);
}

void Robosense::grid_map::loadDStreeXMLFile( const char in_load_file[] )
{
     ds_tree_->load( in_load_file ) ;
}

void Robosense::grid_map::useDStreePredictPointCloud(PointCloudXYZ::Ptr out_cloud_ptr)
{
    if (!is_initial_)
    {
        PCL_ERROR("Cloud points is empty! Try again!");
        return;
    }

    this->generateOneMeterGridMap();
    this->calculateLowestHeightInGridCell();
    this->calculatePtsCloudFeature();
    this->generateOpenCVDescionTreeTrainData();

    // 2.trans to cv::Mat
    float under_predict_feature_mat[ under_predict_point_vec_.size() ][8];
    unsigned int under_predict_xy_mat[ under_predict_point_vec_.size() ][2];
    for (int i = 0; i < under_predict_point_vec_.size(); ++i)
    {
        under_predict_feature_mat[i][0] = under_predict_point_vec_[i].feature.range;
        under_predict_feature_mat[i][1] = under_predict_point_vec_[i].feature.remission;
        under_predict_feature_mat[i][2] = under_predict_point_vec_[i].feature.left_minus;
        under_predict_feature_mat[i][3] = under_predict_point_vec_[i].feature.right_minus;
        under_predict_feature_mat[i][4] = under_predict_point_vec_[i].feature.top_minus;
        under_predict_feature_mat[i][5] = under_predict_point_vec_[i].feature.bottom_minus;
        under_predict_feature_mat[i][6] = under_predict_point_vec_[i].feature.lidar_height;
        under_predict_feature_mat[i][7] = under_predict_point_vec_[i].feature.height_above_lowest;

        under_predict_xy_mat[i][0] = under_predict_point_vec_[i].index.row;
        under_predict_xy_mat[i][1] = under_predict_point_vec_[i].index.col;
    }
    cv::Mat under_predict_feature_cvmat( under_predict_point_vec_.size(), 8, CV_32F, under_predict_feature_mat );
    cv::Mat predicted_value( under_predict_point_vec_.size(), 1, CV_32F );
    for (int k = 0; k < under_predict_point_vec_.size(); ++k)
    {
        const cv::Mat sample = under_predict_feature_cvmat.row(k);
        CvDTreeNode* prediction = ds_tree_->predict( sample );
        if ( prediction->value == obstacle_mark )
            predict_obstacle_cloud_ptr_->push_back( origin_cloud_ptr_->at( under_predict_xy_mat[k][1],under_predict_xy_mat[k][0] ) );
        else if ( prediction->value == ground_mark )
            predict_ground_cloud_ptr_->push_back( origin_cloud_ptr_->at( under_predict_xy_mat[k][1],under_predict_xy_mat[k][0] ) );
    }

    *out_cloud_ptr = *predict_ground_cloud_ptr_;
}

///----------------------------------------------------------------------------------------
///---------------------------------- private member func ---------------------------------
///----------------------------------------------------------------------------------------

void Robosense::grid_map::initialDataSize()
{
    point_info_vec_.resize( origin_cloud_ptr_->height );
    for (int i = 0; i < point_info_vec_.size(); ++i)
    {
        point_info_vec_[i].resize( origin_cloud_ptr_->width );
    }

    is_initial_ = true;
}



void Robosense::grid_map::clearPrePtsCldData()
{
    origin_cloud_ptr_->clear();
    feature_calculate_cloud_ptr_->clear();
    grid_info_vec_.clear();
    clsf_ground_cloud_ptr_->clear();
    clsf_obstacle_cloud_ptr_->clear();

    predict_ground_cloud_ptr_->clear();
    predict_obstacle_cloud_ptr_->clear();
    under_predict_point_vec_.clear();
}

void Robosense::grid_map::generateOneMeterGridMap()
{
    grid_cell_length_ = 1.0;//one meter grid cell length
    grid_size_ = 2 * rslidar_.range / grid_cell_length_;
    this->initialGridMapRelatedDataStruc();
    size_t temp_point_x_vec_No;
    size_t temp_point_y_vec_No;
    for (int i = 0; i < origin_cloud_ptr_->height; ++i)
    {
        for (int j = 0; j < origin_cloud_ptr_->width; ++j)
        {
            if ( !isnan( origin_cloud_ptr_->at( j,i ).x ) )
            {
                temp_point_x_vec_No = static_cast<size_t >( origin_cloud_ptr_->at(j, i).x + grid_size_ / 2);
                temp_point_y_vec_No = static_cast<size_t >( origin_cloud_ptr_->at(j, i).y + grid_size_ / 2);
                if (temp_point_x_vec_No >= 0 && temp_point_x_vec_No < grid_size_ &&
                    temp_point_y_vec_No >= 0 && temp_point_y_vec_No < grid_size_)
                {
                    grid_info_vec_[temp_point_x_vec_No][temp_point_y_vec_No].ptsContainer.push_back( origin_cloud_ptr_->at( j,i ) );
                    point_info_vec_[i][j].flag.grid_x = temp_point_x_vec_No;
                    point_info_vec_[i][j].flag.grid_y = temp_point_y_vec_No;
                }
            }
        }
    }
}

void Robosense::grid_map::initialGridMapRelatedDataStruc()
{
    grid_info_vec_.resize( grid_size_ );
    for (int i = 0; i < grid_size_; ++i)
    {
        grid_info_vec_[i].resize( grid_size_ );
        for (int j = 0; j < grid_size_; ++j)
        {
            grid_info_vec_[i][j].property = unknown_mark;
        }
    }
}

void Robosense::grid_map::calculateLowestHeightInGridCell()
{
    for (int i = 0; i < grid_info_vec_.size(); ++i)
    {
        for (int j = 0; j < grid_info_vec_[i].size(); ++j)
        {
            grid_info_vec_[i][j].lowest_height_ = std::numeric_limits<float>::max();
            for (int k = 0; k < grid_info_vec_[i][j].ptsContainer.size(); ++k)
            {
                if ( grid_info_vec_[i][j].lowest_height_ > grid_info_vec_[i][j].ptsContainer.at(k).z )
                {
                    grid_info_vec_[i][j].lowest_height_ = grid_info_vec_[i][j].ptsContainer.at(k).z;
                }
            }
        }
    }
}

void Robosense::grid_map::fixLeftEdgeColNANPts()
{
    for (int i = 0; i < feature_calculate_cloud_ptr_->height; ++i)
    {
        if (isnan(feature_calculate_cloud_ptr_->at(0, i).x))
        {
            int temp_num = 1;
            while (isnan(feature_calculate_cloud_ptr_->at(temp_num, i).x))
                ++temp_num;

            for (int j = temp_num; j > 1; --j)
                feature_calculate_cloud_ptr_->at(j-1, i) = feature_calculate_cloud_ptr_->at(j, i);
        }
    }
}


void Robosense::grid_map::fixCentralNANPts( const float& in_distance_thre )
{
    std::vector<pcl::PointXYZ> temp_points_template;
    double temp_distance;
    for (int i = 0; i < feature_calculate_cloud_ptr_->height; ++i)
    {
        for (int j = 1; j < feature_calculate_cloud_ptr_->width-1; ++j)
        {
            if ( isnan(feature_calculate_cloud_ptr_->at(j, i).x) )
            {

                temp_points_template.push_back(feature_calculate_cloud_ptr_->at(j - 1, i));
                if (!isnan(feature_calculate_cloud_ptr_->at(j + 1, i).x))
                {
                    temp_points_template.push_back(feature_calculate_cloud_ptr_->at(j + 1, i));
                    temp_distance = sqrt( pow( temp_points_template[0].x - temp_points_template[1].x, 2 )
                                          + pow( temp_points_template[0].y - temp_points_template[1].y, 2 )
                                          + pow( temp_points_template[0].z - temp_points_template[1].z, 2 ) );
                    if ( temp_distance > in_distance_thre )
                    {
                        feature_calculate_cloud_ptr_->at(j, i).x = temp_points_template[0].x;
                        feature_calculate_cloud_ptr_->at(j, i).y = temp_points_template[0].y;
                        feature_calculate_cloud_ptr_->at(j, i).z = temp_points_template[0].z;
                    }
                    else
                    {
                        feature_calculate_cloud_ptr_->at(j, i).x = (temp_points_template[0].x + temp_points_template[1].x) / 2;
                        feature_calculate_cloud_ptr_->at(j, i).y = (temp_points_template[0].y + temp_points_template[1].y) / 2;
                        feature_calculate_cloud_ptr_->at(j, i).z = (temp_points_template[0].z + temp_points_template[1].z) / 2;
                    }
                }
                else
                {
                    feature_calculate_cloud_ptr_->at(j, i).x = temp_points_template[0].x;
                    feature_calculate_cloud_ptr_->at(j, i).y = temp_points_template[0].y;
                    feature_calculate_cloud_ptr_->at(j, i).z = temp_points_template[0].z;
                }
            }

            temp_points_template.clear();
        }
    }
}


void Robosense::grid_map::calculatePtsCloudFeature()
{
    *feature_calculate_cloud_ptr_ = *origin_cloud_ptr_;
    this->fixLeftEdgeColNANPts();
    this->fixCentralNANPts( 0.5 );

    float temp_range[5];
    pointFeatureStruc temp_point_feature;
    size_t temp_point_x_vec_No;
    size_t temp_point_y_vec_No;
    for (int i = 0; i < origin_cloud_ptr_->height; ++i)
    {
        for (int j = 0; j < origin_cloud_ptr_->width; ++j) //except 1st and last column
        {
            if ( !isnan( origin_cloud_ptr_->at( j,i ).x ) )
            {
                temp_point_x_vec_No = static_cast< size_t > (origin_cloud_ptr_->at( j,i ).x + grid_size_/2);
                temp_point_y_vec_No = static_cast< size_t > (origin_cloud_ptr_->at( j,i ).y + grid_size_/2);
                if ( temp_point_x_vec_No >= 0 && temp_point_x_vec_No < grid_size_ &&
                     temp_point_y_vec_No >= 0 && temp_point_y_vec_No < grid_size_ )
                {
                    ///--- 1. calculate range for each position ---///
                    temp_range[0] = sqrt( pow( origin_cloud_ptr_->at(j,i).x, 2 ) + pow( origin_cloud_ptr_->at(j,i).y, 2 ) + pow( origin_cloud_ptr_->at(j,i).z, 2 ) );
                    // left beam range
                    if ( j == 0 || isnan(origin_cloud_ptr_->at(j-1,i).x) )
                        temp_range[1] = temp_range[0];
                    else
                        temp_range[1] = sqrt( pow( origin_cloud_ptr_->at(j-1,i).x, 2 ) + pow( origin_cloud_ptr_->at(j-1,i).y, 2 ) + pow( origin_cloud_ptr_->at(j-1,i).z, 2 ) );
                    // right beam range
                    if ( j == origin_cloud_ptr_->width-1 || isnan(origin_cloud_ptr_->at(j+1,i).x) )
                        temp_range[2] = temp_range[0];
                    else
                        temp_range[2] = sqrt( pow( origin_cloud_ptr_->at(j+1,i).x, 2 ) + pow( origin_cloud_ptr_->at(j+1,i).y, 2 ) + pow( origin_cloud_ptr_->at(j+1,i).z, 2 ) );
                    // top beam range
                    if ( i == 0 || isnan(origin_cloud_ptr_->at(j,i-1).x) )
                        temp_range[3] = temp_range[0];
                    else
                        temp_range[3] = sqrt( pow( origin_cloud_ptr_->at(j,i-1).x, 2 ) + pow( origin_cloud_ptr_->at(j,i-1).y, 2 ) + pow( origin_cloud_ptr_->at(j,i-1).z, 2 ) );
                    // bottom beam range
                    if ( i == origin_cloud_ptr_->height-1 || isnan(origin_cloud_ptr_->at(j,i+1).x) )
                        temp_range[4] = temp_range[0];
                    else
                        temp_range[4] = sqrt( pow( origin_cloud_ptr_->at(j,i+1).x, 2 ) + pow( origin_cloud_ptr_->at(j,i+1).y, 2 ) + pow( origin_cloud_ptr_->at(j,i+1).z, 2 ) );
                    ///--- 2. copy to feature type item ---///
                    temp_point_feature.range = temp_range[0];
                    temp_point_feature.remission = feature_calculate_cloud_ptr_->at( j,i ).data[3];
                    temp_point_feature.left_minus = temp_range[1] - temp_range[0];
                    temp_point_feature.right_minus = temp_range[2] - temp_range[0];
                    temp_point_feature.top_minus = temp_range[3] - temp_range[0];
                    temp_point_feature.bottom_minus = temp_range[4] - temp_range[0];
                    temp_point_feature.lidar_height = rslidar_.height;
                    temp_point_feature.height_above_lowest = feature_calculate_cloud_ptr_->at(j,i).z - grid_info_vec_[temp_point_x_vec_No][temp_point_y_vec_No].lowest_height_;

                    point_info_vec_[i][j].feature = temp_point_feature;
                }
            }
        }
    }
}

void Robosense::grid_map::calculateGridPropertyInHeightDifferMethod(const float& in_differ_height_thre, const float& in_lowest_height_thre)
{
    diff_height_thre_ = in_differ_height_thre;
    lowest_height_thre_ = in_lowest_height_thre;

    float temp_point_z = 0;
    float_t temp_height_difference = 0;
    size_t temp_grid_cell_points_num;
    float temp_highest_height;
    float temp_lowest_height;
    for (int i = 0; i < grid_size_; ++i)
    {
        for (int j = 0; j < grid_size_; ++j)
        {
            if ( grid_info_vec_[i][j].isEmpty() )
                continue;

            temp_grid_cell_points_num = static_cast<int>( grid_info_vec_[i][j].ptsContainer.size() );
            temp_highest_height =  - std::numeric_limits<float>::max();//
            temp_lowest_height = std::numeric_limits<float>::max();
            for (int k = 0; k < temp_grid_cell_points_num; ++k)
            {
                temp_point_z = grid_info_vec_[i][j].ptsContainer.at(k).z;
                if ( temp_lowest_height > temp_point_z )
                    temp_lowest_height = temp_point_z;

                if ( temp_highest_height < temp_point_z )
                    temp_highest_height = temp_point_z;
            }
            temp_height_difference = fabs( temp_highest_height - temp_lowest_height );
            if ( temp_height_difference > diff_height_thre_ || temp_lowest_height > lowest_height_thre_ )
                grid_info_vec_[i][j].property = obstacle_mark;
            else
                grid_info_vec_[i][j].property = ground_mark;
        }
    }
}

void Robosense::grid_map::classifyCloudToGroundAndObstacleCloud()
{
    size_t temp_points_num;
    for (int i = 0; i < grid_size_; ++i)
    {
        for (int j = 0; j < grid_size_; ++j)
        {
            if ( grid_info_vec_[i][j].property == ground_mark )
            {
                temp_points_num = static_cast<size_t> ( grid_info_vec_[i][j].ptsContainer.size() );
                for (int k = 0; k < temp_points_num; ++k)
                    clsf_ground_cloud_ptr_->push_back(grid_info_vec_[i][j].ptsContainer.at(k) );
            }
            else if( grid_info_vec_[i][j].property == obstacle_mark )
            {
                temp_points_num = static_cast<size_t> ( grid_info_vec_[i][j].ptsContainer.size() );
                for (int k = 0; k < temp_points_num; ++k)
                    clsf_obstacle_cloud_ptr_->push_back( grid_info_vec_[i][j].ptsContainer.at(k) );
            }
        }
    }
}

void Robosense::grid_map::calculatePointProperty()
{
    size_t temp_x_grid_num;
    size_t temp_y_grid_num;
    for (int i = 0; i < point_info_vec_.size(); ++i)
    {
        for (int j = 0; j < point_info_vec_[i].size(); ++j)
        {
            temp_x_grid_num = point_info_vec_[i][j].flag.grid_x;
            temp_y_grid_num = point_info_vec_[i][j].flag.grid_y;
            if ( grid_info_vec_[temp_x_grid_num][temp_y_grid_num].property == ground_mark )
                point_info_vec_[i][j].flag.point_property = ground_mark;
            else if ( grid_info_vec_[temp_x_grid_num][temp_y_grid_num].property == obstacle_mark )
                point_info_vec_[i][j].flag.point_property = obstacle_mark;
        }
    }
}














