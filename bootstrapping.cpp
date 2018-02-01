//
// Created by zzy on 1/25/18.
//

#include "bootstrapping.h"

BootstrappingType::BootstrappingType()
{
    origin_cloud_ptr_.reset( new PointCloudXYZ );
    feature_calculate_cloud_ptr_.reset( new PointCloudXYZ );
    classified_ground_cloud_ptr_.reset( new PointCloudXYZ );
    classified_obstacle_cloud_ptr_.reset( new PointCloudXYZ );
    pretreated_cloud_ptr_.reset( new PointCloudXYZ );
    predict_ground_cloud_ptr_.reset( new PointCloudXYZ );
    predict_obstacle_cloud_ptr_.reset( new PointCloudXYZ );
    ds_tree_ptr_ = new CvDTree;
    kd_tree_.reset( new pcl::search::KdTree<pcl::PointXYZ> );
    cluster_cloud_2d_.reset( new pcl::PointCloud<pcl::PointXYZ> );

    cluster_min_size_ = 20;
    cluster_max_size_ = 10000;
    cluster_distance_threshold_ = 0.5;
    is_initial_ = false;
}

void BootstrappingType::SetLidarParameter(const double& in_lidar_height, const double& in_visibility_range)
{
    RS_LiDAR_.height = in_lidar_height;
    RS_LiDAR_.visibility_range = in_visibility_range;
}

void BootstrappingType::ClassifyOriginCloudToGroundAndObstacle(const unsigned int& in_property_method)
{
    if (!is_initial_)
    {
        PCL_ERROR("Cloud points is empty! Try again!");
        return;
    }

    this->generateOneMeterGridMap();
    this->calculateLowestHeightInGridCell();
    this->calculateCloudPointsFeatures( );
    if ( in_property_method == differ_height_method )
    {
        this->CalculateGridPropertyHeightDifferenceMethod( 0.2, -RS_LiDAR_.height + 0.3 );
    }
//    else if ( in_property_method == sigma_height_method )
//    {
//        this->gridPropertyHeightSigmaFunc( );
//    }
//    else if ( in_property_method == average_height_method )
//    {
//        this->gridPropertyAverageFunc( );
//    }
//    else if ( in_property_method == RANSAC_plane_estimate )
//    {
//        this->gridPropertyRANSACFunc( );
//    }
    this->classifyCloudToGroundAndObstacleCloud();
    this->calculatePointFlag();
}

void BootstrappingType::ShowCloudInRvizInterface( PointCloudXYZ::Ptr out_show_cloud )
{
    *out_show_cloud = *classified_ground_cloud_ptr_;
}

void BootstrappingType::SetInputCloud( const PointCloudXYZ::ConstPtr in_cloud )
{
    this->clearOldPointCloudData();

    *origin_cloud_ptr_ = *in_cloud;
    RS_LiDAR_.row_size = in_cloud->height;
    RS_LiDAR_.column_size = in_cloud->width;
    this->initialDataSize( );
    this->SetLidarParameter( 1.8, 75.0 );

    lowest_height_threshold_ = -RS_LiDAR_.height + (-0.05);
    difference_height_threshold_ = 0.2;
}

void BootstrappingType::LoadDStreeXML( const char in_load_file[] )
{
    ds_tree_ptr_->load( in_load_file );
}

void BootstrappingType::initialDataSize( )
{
    point_cloud_flag_.resize( origin_cloud_ptr_->height );
    cloud_point_feature_vec_.resize( origin_cloud_ptr_->height );
    for (int i = 0; i < origin_cloud_ptr_->height; ++i)
    {
        point_cloud_flag_[i].resize( origin_cloud_ptr_->width );
        cloud_point_feature_vec_[i].resize( origin_cloud_ptr_->width );
    }

    ///--- initial point_cloud_grid_flag_[i][j].point_property ---///
    for (int i = 0; i < point_cloud_flag_.size(); ++i)
    {
        for (int j = 0; j < point_cloud_flag_[i].size(); ++j)
        {
            point_cloud_flag_[i][j].point_property = obstacle_point;
        }
    }

    is_initial_ = true;
}


void BootstrappingType::clearOldPointCloudData()
{
    origin_cloud_ptr_->clear();
    pretreated_cloud_ptr_->clear();
    grid_map_property_vec_.clear();
    classified_ground_cloud_ptr_->clear();
    classified_obstacle_cloud_ptr_->clear();
    feature_calculate_cloud_ptr_->clear();
    under_predict_point_vec_.clear();
    under_predict_point_num_vec_.clear();
    predict_ground_cloud_ptr_->clear();
    predict_obstacle_cloud_ptr_->clear();
    point_cloud_flag_.clear();
    cloud_point_feature_vec_.clear();
    grid_cell_lowest_height_vec_.clear();
    grid_map_vec_.clear();
    grid_cell_points_property_.clear();
    grid_statistic_feature_vec_.clear();
    terrain_height_estimation_vec_.clear();
    cluster_cloud_2d_->clear();
    cluster_indices_.clear();
    colored_cluster_cloud_vec_.clear();

    is_initial_= false;
}


void BootstrappingType::fixSingleLeftEdgeColumnNANPoints( )
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

void BootstrappingType::fixCentralNANPoints( const double& in_distance_threshold )
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
                    if ( temp_distance > in_distance_threshold )
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

void BootstrappingType::pretreatOriginCloudWithHeightLimite(const double &in_lowest_height,
                                                            const double &in_highest_height)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud ( origin_cloud_ptr_ );
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (in_lowest_height, in_highest_height);// lidar height is 1.8m
    pass.filter ( *pretreated_cloud_ptr_ );
}

void BootstrappingType::generateOneMeterGridMap( )
{
    grid_cell_length_ = 1.0;//one meter grid cell length
    grid_size_ = 2 * RS_LiDAR_.visibility_range / grid_cell_length_;
    this->initialGridMapSizeRelatedDataSize();
    unsigned int temp_point_x_vec_No;
    unsigned int temp_point_y_vec_No;
    for (int i = 0; i < origin_cloud_ptr_->height; ++i)
    {
        for (int j = 0; j < origin_cloud_ptr_->width; ++j)
        {
            if ( !isnan( origin_cloud_ptr_->at( j,i ).x ) )
            {
                temp_point_x_vec_No = (int) origin_cloud_ptr_->at(j, i).x + grid_size_ / 2;
                temp_point_y_vec_No = (int) origin_cloud_ptr_->at(j, i).y + grid_size_ / 2;
                if (temp_point_x_vec_No >= 0 && temp_point_x_vec_No < grid_size_ &&
                    temp_point_y_vec_No >= 0 && temp_point_y_vec_No < grid_size_)
                {
                    grid_map_vec_[temp_point_x_vec_No][temp_point_y_vec_No].push_back(origin_cloud_ptr_->at(j, i));
                    point_cloud_flag_[i][j].x_grid_num = temp_point_x_vec_No;
                    point_cloud_flag_[i][j].y_grid_num = temp_point_y_vec_No;
                }
            }
        }
    }
}

void BootstrappingType::CalculateGridPropertyHeightDifferenceMethod( const double & input_difference_height_threshold, const double &input_lowest_height_threshold )
{
    difference_height_threshold_ = input_difference_height_threshold;
    lowest_height_threshold_ = input_lowest_height_threshold;

    double temp_point_z = 0;
    double temp_height_difference = 0;
    unsigned int temp_grid_cell_points_num;
    for (int i = 0; i < grid_size_; ++i)
    {
        for (int j = 0; j < grid_size_; ++j)
        {
            if (grid_map_vec_[i][j].empty())
                continue;

            temp_grid_cell_points_num = static_cast<int>(grid_map_vec_[i][j].size());
            double temp_highest_height =  - std::numeric_limits<double >::max();//
            double temp_lowest_height = std::numeric_limits<double >::max();
            for (int k = 0; k < temp_grid_cell_points_num; ++k)
            {
                temp_point_z = grid_map_vec_[i][j][k].z;

                if ( temp_lowest_height > temp_point_z )
                {
                    temp_lowest_height = temp_point_z;
                }
                if ( temp_highest_height < temp_point_z )
                {
                    temp_highest_height = temp_point_z;
                }
            }
            temp_height_difference = fabs( temp_highest_height - temp_lowest_height );
            if ( temp_height_difference > difference_height_threshold_ || temp_lowest_height > input_lowest_height_threshold )
            {
                grid_map_property_vec_[i][j] = obstacle_cell;
            }
            else
            {
                grid_map_property_vec_[i][j] = ground_cell;
            }
        }
    }
}

void BootstrappingType::classifyCloudToGroundAndObstacleCloud( )
{
    unsigned int temp_points_num;
    for (int i = 0; i < grid_size_; ++i)
    {
        for (int j = 0; j < grid_size_; ++j)
        {
            if ( grid_map_property_vec_[i][j] == ground_cell )
            {
                temp_points_num = (unsigned int) grid_map_vec_[i][j].size();
                for (int k = 0; k < temp_points_num; ++k)
                {
                    classified_ground_cloud_ptr_->push_back( grid_map_vec_[i][j][k] );
                }
            }
            else
            {
                temp_points_num = (unsigned int) grid_map_vec_[i][j].size();
                for (int k = 0; k < temp_points_num; ++k)
                {
                    classified_obstacle_cloud_ptr_->push_back( grid_map_vec_[i][j][k] );
                }
            }
        }
    }
}

void BootstrappingType::GenerateSetValueGridMap( const double &in_grid_length )
{
    grid_cell_length_ = in_grid_length;
    //---todo

}

void BootstrappingType::initialGridMapSizeRelatedDataSize( )
{
    grid_cell_lowest_height_vec_.resize( grid_size_ );
    grid_map_vec_.resize( grid_size_ );
    grid_map_property_vec_.resize( grid_size_ );
    grid_cell_points_property_.resize( grid_size_ );
    grid_statistic_feature_vec_.resize( grid_size_ );
    terrain_height_estimation_vec_.resize( grid_size_ );
    for (int i = 0; i < grid_size_; ++i)
    {
        grid_cell_lowest_height_vec_[i].resize( grid_size_ );
        grid_map_vec_[i].resize( grid_size_ );
        grid_map_property_vec_[i].resize( grid_size_ );
        grid_cell_points_property_[i].resize( grid_size_ );
        grid_statistic_feature_vec_[i].resize( grid_size_ );
        terrain_height_estimation_vec_[i].resize( grid_size_ );
    }
}

void BootstrappingType::calculateLowestHeightInGridCell( )
{
    for (int i = 0; i < grid_map_vec_.size(); ++i)
    {
        for (int j = 0; j < grid_map_vec_[i].size(); ++j)
        {
            grid_cell_lowest_height_vec_[i][j] = std::numeric_limits<double >::max();
            for (int k = 0; k < grid_map_vec_[i][j].size(); ++k)
            {
                if ( grid_cell_lowest_height_vec_[i][j] > grid_map_vec_[i][j][k].z )
                {
                    grid_cell_lowest_height_vec_[i][j] = grid_map_vec_[i][j][k].z;
                }
            }
        }
    }
}

void BootstrappingType::calculateCloudPointsFeatures( )
{
    *feature_calculate_cloud_ptr_ = *origin_cloud_ptr_;
    this->fixSingleLeftEdgeColumnNANPoints();
    this->fixCentralNANPoints( 0.5 );

    double temp_range[5];
    PointFeatureStruct temp_point_feature;
    int temp_point_x_vec_No;
    int temp_point_y_vec_No;
    for (int i = 0; i < origin_cloud_ptr_->height; ++i)
    {
        for (int j = 0; j < origin_cloud_ptr_->width; ++j) //except 1st and last column
        {
            if ( !isnan( origin_cloud_ptr_->at( j,i ).x ) )
            {
                temp_point_x_vec_No = (int)origin_cloud_ptr_->at( j,i ).x + grid_size_/2;
                temp_point_y_vec_No = (int)origin_cloud_ptr_->at( j,i ).y + grid_size_/2;
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
                    temp_point_feature.self_range = temp_range[0];
                    temp_point_feature.remission = feature_calculate_cloud_ptr_->at( j,i ).data[3];
                    temp_point_feature.left_minus_self = temp_range[1] - temp_range[0];
                    temp_point_feature.right_minus_self = temp_range[2] - temp_range[0];
                    temp_point_feature.top_minus_self = temp_range[3] - temp_range[0];
                    temp_point_feature.bottom_minus_self = temp_range[4] - temp_range[0];
                    temp_point_feature.lidar_height = RS_LiDAR_.height;
                    temp_point_feature.height_above_lowest = feature_calculate_cloud_ptr_->at(j,i).z - grid_cell_lowest_height_vec_[temp_point_x_vec_No][temp_point_y_vec_No];

                    cloud_point_feature_vec_[i][j] = temp_point_feature;
                }
            }
        }
    }
}


void BootstrappingType::calculatePointFlag()
{
    unsigned int temp_x_grid_num;
    unsigned int temp_y_grid_num;
    for (int i = 0; i < point_cloud_flag_.size(); ++i)
    {
        for (int j = 0; j < point_cloud_flag_[i].size(); ++j)
        {
            temp_x_grid_num = point_cloud_flag_[i][j].x_grid_num;
            temp_y_grid_num = point_cloud_flag_[i][j].y_grid_num;
            if ( grid_map_property_vec_[temp_x_grid_num][temp_y_grid_num] == ground_cell )
            {
                point_cloud_flag_[i][j].point_property = ground_point;
            }
            else if ( grid_map_property_vec_[temp_x_grid_num][temp_y_grid_num] == obstacle_cell )
            {
                point_cloud_flag_[i][j].point_property = obstacle_point;
            }
        }
    }
}


void BootstrappingType::ClassifyPointFeaturesThroughWeakClassifier(const double &height_above_lowest_thre, const double &bottom_minus_self_thre)
{
    *feature_calculate_cloud_ptr_ = *origin_cloud_ptr_;
    ///--- pretreat data ---
    this->generateOneMeterGridMap();
    this->calculateLowestHeightInGridCell();
    this->calculateCloudPointsFeatures( );
    ///--- classify ---///
    PointFeatureStruct temp_point_feature;
    for (int i = 0; i < cloud_point_feature_vec_.size(); ++i)
    {
        for (int j = 0; j < cloud_point_feature_vec_[i].size(); ++j)
        {
            if ( !cloud_point_feature_vec_[i][j].isEmpty() )
            {
                temp_point_feature = cloud_point_feature_vec_[i][j];
                if (temp_point_feature.height_above_lowest < height_above_lowest_thre)
                    point_cloud_flag_[i][j].point_property = ground_point;
                if (temp_point_feature.bottom_minus_self < bottom_minus_self_thre)
                    point_cloud_flag_[i][j].point_property = ground_point;
            }
        }
    }

    this->calculateGridCellStatisticProperty();
}

void BootstrappingType::calculateGridCellStatisticProperty()
{
    PointFlag temp_point_flag;
    for (int i = 0; i < feature_calculate_cloud_ptr_->height; ++i)
    {
        for (int j = 0; j < feature_calculate_cloud_ptr_->width; ++j)
        {
            // according the point flag, classify each point to grid cell.
            temp_point_flag = point_cloud_flag_[i][j];
            if ( temp_point_flag.point_property == ground_point )
                grid_cell_points_property_[temp_point_flag.x_grid_num][temp_point_flag.y_grid_num].ground_points.push_back( feature_calculate_cloud_ptr_->at( j,i ) );
            else if ( temp_point_flag.point_property == obstacle_point )
                grid_cell_points_property_[temp_point_flag.x_grid_num][temp_point_flag.y_grid_num].obstacle_points.push_back( feature_calculate_cloud_ptr_->at( j,i ) );
        }
    }

    // after last step done, calculate the grid statistic feature.
    for (int i = 0; i < grid_cell_points_property_.size(); ++i)
    {
        for (int j = 0; j < grid_cell_points_property_[i].size(); ++j)
        {
            double temp_ground_average_height;
            double temp_ground_highest_height = - std::numeric_limits<double >::max();
            double temp_ground_lowest_height = std::numeric_limits<double >::max();
            double temp_obstacle_average_height;
            double temp_obstacle_highest_height = - std::numeric_limits<double >::max();
            double temp_obstacle_lowest_height = std::numeric_limits<double >::max();

            if ( !grid_cell_points_property_[i][j].ground_points.empty() )
            {
                for (int k = 0; k < grid_cell_points_property_[i][j].ground_points.size(); ++k)
                {
                    temp_ground_average_height += grid_cell_points_property_[i][j].ground_points[k].z;

                    if (temp_ground_highest_height < grid_cell_points_property_[i][j].ground_points[k].z)
                        temp_ground_highest_height = grid_cell_points_property_[i][j].ground_points[k].z;
                    if (temp_ground_lowest_height > grid_cell_points_property_[i][j].ground_points[k].z)
                        temp_ground_lowest_height = grid_cell_points_property_[i][j].ground_points[k].z;
                }
                temp_ground_average_height = temp_ground_average_height / grid_cell_points_property_[i][j].ground_points.size();

                grid_statistic_feature_vec_[i][j].mu_g = temp_ground_average_height;
                grid_statistic_feature_vec_[i][j].delta_g = temp_ground_highest_height - temp_ground_lowest_height;
            }
            else
            {
                grid_statistic_feature_vec_[i][j].mu_g = 0;
                grid_statistic_feature_vec_[i][j].delta_g = -1;
            }

            if ( !grid_cell_points_property_[i][j].obstacle_points.empty() )
            {
                for (int k = 0; k < grid_cell_points_property_[i][j].obstacle_points.size(); ++k)
                {
                    temp_obstacle_average_height += grid_cell_points_property_[i][j].obstacle_points[k].z;

                    if (temp_obstacle_highest_height < grid_cell_points_property_[i][j].obstacle_points[k].z)
                        temp_obstacle_highest_height = grid_cell_points_property_[i][j].obstacle_points[k].z;
                    if (temp_obstacle_lowest_height > grid_cell_points_property_[i][j].obstacle_points[k].z)
                        temp_obstacle_lowest_height = grid_cell_points_property_[i][j].obstacle_points[k].z;
                }
                temp_obstacle_average_height = temp_obstacle_average_height / grid_cell_points_property_[i][j].obstacle_points.size();

                grid_statistic_feature_vec_[i][j].mu_o = temp_obstacle_average_height;
                grid_statistic_feature_vec_[i][j].delta_o = temp_obstacle_highest_height - temp_obstacle_lowest_height;
            }
            else
            {
                grid_statistic_feature_vec_[i][j].mu_o = 0;
                grid_statistic_feature_vec_[i][j].delta_o = -1;
            }

            grid_statistic_feature_vec_[i][j].mu_t = RS_LiDAR_.height;
            grid_statistic_feature_vec_[i][j].delta_t = 0;
        }
    }
}

void BootstrappingType::calculateCRFModelPsiItem(  const double& in_point_height,
                                                   const double& in_mu_t, const double& in_delta_t,
                                                   const double& in_mu_g, const double& in_delta_g,
                                                   const double& in_mu_o, const double& in_delta_o)
{
    double temp_psi_1 = 0;
    double temp_psi_2 = 0;
    double temp_psi_3 = 0;
    temp_psi_1 = pow( (in_point_height - in_mu_t) / in_delta_t, 2 ) / 2;
    temp_psi_2 = pow( (in_point_height - in_mu_g) / in_delta_g, 2 ) / 2;
    temp_psi_3 = - log( 1 - 1/2 * ( 1 + erf( 0.7071067812 * (in_point_height - in_mu_o) / in_delta_o )) );
}

void BootstrappingType::calculateCRFModelPhiItem( const double in_height_i, std::vector< std::vector<double > > in_points_height )
{
    double temp_phi = 0;
    for (int i = 1; i < in_points_height.size() - 1; ++i)
    {
        for (int j = 1; j < in_points_height[i].size() - 1; ++j)
        {
            temp_phi = 1/2 * ( pow( in_height_i - in_points_height[i-1][j], 2 ) + pow( in_height_i - in_points_height[i+1][j], 2 )
                               + pow( in_height_i - in_points_height[i][j-1], 2 ) + pow( in_height_i - in_points_height[i][j+1], 2 ));

        }
    }

}

void BootstrappingType::conjugateGradientOptimization()
{
    //--todo
}

void BootstrappingType::GridientDesentMethodEstimateTerrain( const double& learning_step, const double& omega_psi, const double& omega_phi )
{
    /// 1. initial height as zero
    for (int i = 0; i < terrain_height_estimation_vec_.size(); ++i)
    {
        for (int j = 0; j < terrain_height_estimation_vec_[i].size(); ++j)
        {
            terrain_height_estimation_vec_[i][j] = 0.0;
        }
    }
    /// 2. calculate minus gradient
    for (int k = 0; k < 1000; ++k)
    {
        //--todo

    }

    std::vector< std::vector< double > > temp_height_vec;
    temp_height_vec = terrain_height_estimation_vec_;
    for (int i = 0; i < grid_statistic_feature_vec_.size(); ++i)
    {
        for (int j = 0; j < grid_statistic_feature_vec_[i].size(); ++j)
        {
            terrain_height_estimation_vec_[i][j] = 0;

            for (int k = 0; k < 1000; ++k) // times of iteration
            {

                //--todo


            }
        }
    }
}

void BootstrappingType::GetOpenCVDescionTreeTrainData( std::vector< PointFeatureStruct >& out_point_feature_vec,  std::vector< float >& out_response_vec )
{
    for (int i = 0; i < cloud_point_feature_vec_.size(); ++i)
    {
        for (int j = 0; j < cloud_point_feature_vec_[i].size(); ++j)
        {
            if( !cloud_point_feature_vec_[i][j].isEmpty() )
            {
                out_point_feature_vec.push_back( cloud_point_feature_vec_[i][j] );
                out_response_vec.push_back( point_cloud_flag_[i][j].point_property );
            }
        }
    }
}

void BootstrappingType::calculatePsiGradient( const double& in_grid_height, GridStatisticFeatureStruct& in_grid_statistic_feature, double out_gradient[] )
{
//    double temp_psi_ground_height_gradient;
//    temp_psi_ground_height_gradient = ( in_point_height - in_grid_statistic_feature.mu_t ) / ( 2 * in_grid_statistic_feature.delta_t*in_grid_statistic_feature.delta_t );

    double temp_psi_ground_gradient;
    temp_psi_ground_gradient = ( in_grid_height - in_grid_statistic_feature.mu_g ) / ( 2 * in_grid_statistic_feature.delta_g*in_grid_statistic_feature.delta_g );
    double temp_psi_obstacle_gradient;
    temp_psi_obstacle_gradient = 1.414213562 * exp( -0.5 * (( in_grid_height - in_grid_statistic_feature.mu_o ) / in_grid_statistic_feature.delta_o)*(( in_grid_height - in_grid_statistic_feature.mu_o ) / in_grid_statistic_feature.delta_o) )
                           / ( sqrt( Pi*in_grid_statistic_feature.delta_o ) * ( 1 - erf( (in_grid_height - in_grid_statistic_feature.mu_o) / ( 1.414213562 * in_grid_statistic_feature.delta_o) ) ));

    out_gradient[0] = in_grid_height;
    out_gradient[1] = temp_psi_ground_gradient + temp_psi_obstacle_gradient;
}

void BootstrappingType::calculatePhiGradient(  )
{
    // 4-neighborhood

    //--todo


}

void BootstrappingType::generateOpenCVDSTreePredictDataType( )
{
    std::vector<unsigned int> temp_num;
    temp_num.resize(2);
    for (int i = 0; i < cloud_point_feature_vec_.size(); ++i)
    {
        for (int j = 0; j < cloud_point_feature_vec_[i].size(); ++j)
        {
            if ( !cloud_point_feature_vec_[i][j].isEmpty() )
            {
                under_predict_point_vec_.push_back( cloud_point_feature_vec_[i][j] );
                temp_num[0] = i;  temp_num[1] = j;
                under_predict_point_num_vec_.push_back( temp_num );
            }
        }
    }
}

void BootstrappingType::WriteDataToTXTFile( const std::string in_file_rout, const std::vector<std::vector<double> > in_data_vec )
{
    char temp_rout[50];
    strcpy( temp_rout, in_file_rout.c_str() );
    std::ofstream temp_outfile;
    temp_outfile.open( temp_rout );
    if(!temp_outfile)
        std::cerr << "Fail to open txt file!" << std::endl;
}

void BootstrappingType::UseDSTreePredictCloud( PointCloudXYZ::Ptr out_cloud_ptr )
{
    if (!is_initial_)
    {
        PCL_ERROR("Cloud points is empty! Try again!");
        return;
    }

    this->generateOneMeterGridMap();
    this->calculateLowestHeightInGridCell();
    this->calculateCloudPointsFeatures( );
    this->generateOpenCVDSTreePredictDataType();

    // 2.trans to cv::Mat
    float under_predict_feature_mat[ under_predict_point_vec_.size() ][8];
    unsigned int under_predict_xy_mat[ under_predict_point_vec_.size() ][2];
    for (int i = 0; i < under_predict_point_vec_.size(); ++i)
    {
        under_predict_feature_mat[i][0] = under_predict_point_vec_[i].self_range;
        under_predict_feature_mat[i][1] = under_predict_point_vec_[i].remission;
        under_predict_feature_mat[i][2] = under_predict_point_vec_[i].left_minus_self;
        under_predict_feature_mat[i][3] = under_predict_point_vec_[i].right_minus_self;
        under_predict_feature_mat[i][4] = under_predict_point_vec_[i].top_minus_self;
        under_predict_feature_mat[i][5] = under_predict_point_vec_[i].bottom_minus_self;
        under_predict_feature_mat[i][6] = under_predict_point_vec_[i].lidar_height;
        under_predict_feature_mat[i][7] = under_predict_point_vec_[i].height_above_lowest;

        under_predict_xy_mat[i][0] = under_predict_point_num_vec_[i][0];
        under_predict_xy_mat[i][1] = under_predict_point_num_vec_[i][1];
    }
    cv::Mat under_predict_feature_cvmat( under_predict_point_vec_.size(), 8, CV_32F, under_predict_feature_mat );
    cv::Mat predicted_value( under_predict_point_vec_.size(), 1, CV_32F );
    for (int k = 0; k < under_predict_point_vec_.size(); ++k)
    {
        const cv::Mat sample = under_predict_feature_cvmat.row(k);
        CvDTreeNode* prediction = ds_tree_ptr_->predict( sample );

        if ( prediction->value == obstacle_point )
        {
            predict_obstacle_cloud_ptr_->push_back( feature_calculate_cloud_ptr_->at( under_predict_xy_mat[k][1],under_predict_xy_mat[k][0] ) );
        }
        else if ( prediction->value == ground_point )
        {
            predict_ground_cloud_ptr_->push_back( feature_calculate_cloud_ptr_->at( under_predict_xy_mat[k][1],under_predict_xy_mat[k][0] ) );
        }
    }

    *out_cloud_ptr = *predict_ground_cloud_ptr_;

}

float BootstrappingType::EvaluateDSTreePredictAccuracy( cv::Mat& predicted, cv::Mat& actual )
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

void BootstrappingType::EuclideanCluster( PointCLoudXYZRGB::Ptr output_cloud_ptr,
                                          const double in_cluster_distance_thre,
                                          const unsigned int in_cluster_min_size,
                                          const unsigned int in_cluster_max_size)
{
    cluster_min_size_ = in_cluster_min_size;
    cluster_max_size_ = in_cluster_max_size;
    cluster_distance_threshold_ = in_cluster_distance_thre;

    // make point cloud flat
    pcl::copyPointCloud( *predict_obstacle_cloud_ptr_, *cluster_cloud_2d_ );
    for (size_t i = 0; i < cluster_cloud_2d_->points.size(); i++)
        cluster_cloud_2d_->points[i].z = 0;


    std::vector<int> temp_indicates;
    pcl::removeNaNFromPointCloud( *cluster_cloud_2d_, *cluster_cloud_2d_, temp_indicates );
    if ( cluster_cloud_2d_->points.size() > 0 )
        kd_tree_->setInputCloud( cluster_cloud_2d_ );

    // perform clustering on 2d cloud
    euclidean_cluster_extrator_.setClusterTolerance ( cluster_distance_threshold_ );
    euclidean_cluster_extrator_.setMinClusterSize ( cluster_min_size_ );
    euclidean_cluster_extrator_.setMaxClusterSize ( cluster_max_size_ );
    euclidean_cluster_extrator_.setSearchMethod( kd_tree_ );
    euclidean_cluster_extrator_.setInputCloud( cluster_cloud_2d_ );
    euclidean_cluster_extrator_.extract ( cluster_indices_ );
    // colour clustered points
    unsigned int k = 0;
    colored_cluster_cloud_vec_.resize( cluster_indices_.size() );
    unsigned int temp_single_cluster_points = 0;
    for(std::vector<pcl::PointIndices>::iterator iter = cluster_indices_.begin(); iter != cluster_indices_.end(); ++iter )
    {
        unsigned int cloud_color_rgb[3];
        cloud_color_rgb[0] = rand()%255;
        cloud_color_rgb[1] = rand()%255;
        cloud_color_rgb[2] = rand()%255;
        for (std::vector<int>::iterator i = iter->indices.begin(); i != iter->indices.end() ; ++i)
        {
            pcl::PointXYZRGB p;
            p.x = predict_obstacle_cloud_ptr_->points[*i].x;
            p.y = predict_obstacle_cloud_ptr_->points[*i].y;
            p.z = predict_obstacle_cloud_ptr_->points[*i].z;
            p.r = cloud_color_rgb[0];
            p.g = cloud_color_rgb[1];
            p.b = cloud_color_rgb[2];

            output_cloud_ptr->push_back( p );
        }
    }
}



































































