//
// Created by zzy on 1/25/18.
//

#ifndef BOOTSTRAPPING_BOOTSTRAPPING_H
#define BOOTSTRAPPING_BOOTSTRAPPING_H

#include "bootstrapping_include.h"

class BootstrappingType
{
public:
    explicit BootstrappingType();
    ~BootstrappingType() {}

    void SetLidarParameter( const double& in_lidar_height, const double& in_visibility_range );
    void SetInputCloud( const PointCloudXYZ::ConstPtr input_cloud );
    void LoadDStreeXML( const char in_load_file[] );
    void GenerateSetValueGridMap( const double &in_grid_length );
    void CalculateGridPropertyHeightDifferenceMethod( const double & input_difference_height_threshold, const double &input_lowest_height_threshold );
    void ClassifyOriginCloudToGroundAndObstacle( const unsigned int& in_property_method );
    void ShowCloudInRvizInterface( PointCloudXYZ::Ptr out_show_cloud );
    void ClassifyPointFeaturesThroughWeakClassifier( const double &height_above_lowest_thre, const double &bottom_minus_self_thre );
    void GridientDesentMethodEstimateTerrain( const double& learning_step, const double& omega_psi, const double& omega_phi );
    void GetOpenCVDescionTreeTrainData( std::vector< PointFeatureStruct >& out_point_feature_vec,  std::vector< float >& out_response_vec );
    void WriteDataToTXTFile( const std::string in_file_rout, const std::vector<std::vector<double> > in_data_vec );
    void UseDSTreePredictCloud( PointCloudXYZ::Ptr out_obstacle_cloud );
    float EvaluateDSTreePredictAccuracy( cv::Mat& predicted, cv::Mat& actual );



private:
    void initialDataSize( );
    void clearOldPointCloudData( );
    void fixSingleLeftEdgeColumnNANPoints( );
    void fixCentralNANPoints( const double& in_distance_threshold );
    void pretreatOriginCloudWithHeightLimite( const double& in_lowest_height, const double& in_highest_height ); // ( -1.5, 3.0 + )
    void generateOneMeterGridMap( );
    void classifyCloudToGroundAndObstacleCloud( );
    void initialGridMapSizeRelatedDataSize( );
    void calculateLowestHeightInGridCell( );
    void calculateCloudPointsFeatures( );
    void calculatePointFlag();
    void calculateGridCellStatisticProperty( );
    void calculateCRFModelPsiItem( const double& in_point_height, const double& in_mu_t, const double& in_delta_t, const double& in_mu_g, const double& in_delta_g, const double& in_mu_o, const double& in_delta_o );//conditional random field in Gibbsian form
    void calculateCRFModelPhiItem( const double in_height_i, std::vector< std::vector<double > > in_points_height );
    void conjugateGradientOptimization( );
    void calculatePsiGradient( const double& in_point_height, GridStatisticFeatureStruct& in_grid_statistic_feature, double out_gradient[] );
    void calculatePhiGradient( );
    void generateOpenCVDSTreePredictDataType( );


private:
    LidarParamter RS_LiDAR_;
    double grid_cell_length_;
    unsigned int grid_size_;

    PointCloudXYZ::Ptr origin_cloud_ptr_;
    bool is_initial_;
    PointCloudXYZ::Ptr pretreated_cloud_ptr_;
    std::vector< std::vector <int> > grid_map_property_vec_;//use enum g_grid_cell_property
    PointCloudXYZ::Ptr classified_ground_cloud_ptr_;
    PointCloudXYZ::Ptr classified_obstacle_cloud_ptr_;
    PointCloudXYZ::Ptr feature_calculate_cloud_ptr_;
    CvDTree *ds_tree_ptr_;
    std::vector< PointFeatureStruct > under_predict_point_vec_;
    PointCloudXYZ::Ptr predict_ground_cloud_ptr_;
    PointCloudXYZ::Ptr predict_obstacle_cloud_ptr_;
    std::vector< std::vector< PointFlag > > point_cloud_flag_;// 16*2016
    std::vector< std::vector< PointFeatureStruct > > cloud_point_feature_vec_;// 16*2016
    std::vector< std::vector<double> > grid_cell_lowest_height_vec_;// 150*150
    std::vector< std::vector< std::vector<pcl::PointXYZ> > > grid_map_vec_; // 150*150
    std::vector< std::vector< GridPointsPropertyContainer > > grid_cell_points_property_; // 150*150
    std::vector< std::vector< GridStatisticFeatureStruct > > grid_statistic_feature_vec_; // 150*150
    std::vector< std::vector< double > > terrain_height_estimation_vec_; // 150*150
    double difference_height_threshold_;
    double lowest_height_threshold_;
};
#endif //BOOTSTRAPPING_BOOTSTRAPPING_H
