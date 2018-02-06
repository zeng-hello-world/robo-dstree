//
// Created by zzy on 2/5/18.
//

#include "normal_vis.h"


Robosense::normal_vis::normal_vis()
{
    origin_cloud_ptr_.reset( new PointCloudXYZ );
    feature_calculate_cloud_ptr_.reset( new PointCloudXYZ );
    kd_tree_method_ptr_.reset( new KdTreeSearchMethod );
    cloud_normal_ptr_.reset( new PointCloudNormal );
}

void Robosense::normal_vis::setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr in_point_cloud_ptr)
{
    this->clearPrePointCloudData();

    *origin_cloud_ptr_ = *in_point_cloud_ptr;
}

void Robosense::normal_vis::getPointCloudNormal(const float &in_neighbor_radius, visualization_msgs::MarkerArray &out_arrow_marker)
{
    *feature_calculate_cloud_ptr_ = *origin_cloud_ptr_;
    this->fixLeftEdgeColNANPts();
    this->fixCentralNANPts( 0.5 );
    this->calculatePointNormal( in_neighbor_radius );

    visualization_msgs::Marker temp_marker;
    temp_marker.type = visualization_msgs::Marker::ARROW;
    temp_marker.header.frame_id = "/rslidar";
    temp_marker.header.stamp = ros::Time();
    temp_marker.ns = "arrows";
    temp_marker.id = 0;
    temp_marker.action = visualization_msgs::Marker::ADD;
    temp_marker.lifetime = ros::Duration();
    temp_marker.scale.x = 0.05; temp_marker.scale.y = 0.1; temp_marker.scale.z = 0.1;
    temp_marker.color.a = 1.0; temp_marker.color.r = 0.0; temp_marker.color.g = 1.0; temp_marker.color.b = 0.0;
    geometry_msgs::Point p_start;
    geometry_msgs::Point p_end;
    for (int i = 0; i < origin_cloud_ptr_->height; ++i)
    {
        for (int j = 0; j < origin_cloud_ptr_->width; ++j)
        {
            if ( !isnan( origin_cloud_ptr_->at( j,i ).x ) )
            {
                if ( !isnan( cloud_normal_ptr_->at( j,i ).normal_x ) )
                {
                    temp_marker.id ++;
                    p_start.x = origin_cloud_ptr_->at(j, i).x;
                    p_start.y = origin_cloud_ptr_->at(j, i).y;
                    p_start.z = origin_cloud_ptr_->at(j, i).z;
                    p_end.x = p_start.x + cloud_normal_ptr_->at(j, i).normal_x;
                    p_end.y = p_start.y + cloud_normal_ptr_->at(j, i).normal_y;
                    p_end.z = p_start.z + cloud_normal_ptr_->at(j, i).normal_z;
                    temp_marker.points.push_back(p_start);
                    temp_marker.points.push_back(p_end);

                    out_arrow_marker.markers.push_back( temp_marker );
                    temp_marker.points.clear();
                }
            }
        }
    }
}

void Robosense::normal_vis::showArrowMarker(visualization_msgs::Marker &out_arrow_marker)
{
    out_arrow_marker.type = visualization_msgs::Marker::ARROW;
    out_arrow_marker.header.frame_id = "/rslidar";
    out_arrow_marker.header.stamp = ros::Time();
    out_arrow_marker.ns = "arrows";
    out_arrow_marker.id = 0;
    out_arrow_marker.action = visualization_msgs::Marker::ADD;
    out_arrow_marker.lifetime = ros::Duration();
    out_arrow_marker.scale.x = 0.05; out_arrow_marker.scale.y = 0.1; out_arrow_marker.scale.z = 0.1;
    out_arrow_marker.color.a = 1.0; out_arrow_marker.color.r = 0.0; out_arrow_marker.color.g = 1.0; out_arrow_marker.color.b = 0.0;
    geometry_msgs::Point p_start;
    p_start.x = 0; p_start.y = 0; p_start.z = 0;
    geometry_msgs::Point p_end;
    p_end.x = 1; p_end.y = 1; p_end.z = 1;
    out_arrow_marker.points.push_back( p_start );
    out_arrow_marker.points.push_back( p_end );
}

///+++++++++++++++++++++++++++++ private member function ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++///

void Robosense::normal_vis::clearPrePointCloudData()
{
    origin_cloud_ptr_->clear();
    feature_calculate_cloud_ptr_->clear();
    cloud_normal_ptr_->clear();
}

void Robosense::normal_vis::fixLeftEdgeColNANPts()
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

void Robosense::normal_vis::fixCentralNANPts(const float& in_distance_thre)
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

void Robosense::normal_vis::calculatePointNormal(const float &in_neighbor_radius)
{
    clock_t time_start, time_finish;
    time_start = clock();
    normalEstimation.setInputCloud( feature_calculate_cloud_ptr_ );
    normalEstimation.setSearchMethod( kd_tree_method_ptr_ );
    normalEstimation.setNumberOfThreads( 2); // 8 core cpu
    normalEstimation.setRadiusSearch( in_neighbor_radius );
    normalEstimation.compute( *cloud_normal_ptr_ );
    time_finish=clock();
    std::cout << (float)(time_finish - time_start) / 1000  << "(ms)"<< std::endl;
}




