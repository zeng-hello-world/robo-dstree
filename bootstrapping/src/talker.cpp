//
// Created by zzy on 1/25/18.
//
#include "bootstrapping.h"
#include "ros/ros.h"
///++++++++++++++++++++++++++++++++++ Show in result rviz +++++++++++++++++++++++++++++++++++++++++///

//static ros::Publisher g_chatter_pub;
//static BootstrappingType *test_cloud = new BootstrappingType;
//static PointCloudXYZ::Ptr temp_show_cloud_ptr ( new PointCloudXYZ );
//
//void ShowResultInRviz( PointCloudXYZ::Ptr input_cloud_ptr )
//{
//    input_cloud_ptr->header.frame_id = "rslidar";
//
//    test_cloud->SetInputCloud( input_cloud_ptr );
//    test_cloud->ClassifyOriginCloudToGroundAndObstacle( differ_height_method ); // change show cloud here
//    test_cloud->ShowCloudInRvizInterface( temp_show_cloud_ptr );
//    temp_show_cloud_ptr->header.frame_id = "rslidar";
//    g_chatter_pub.publish( temp_show_cloud_ptr );
//    temp_show_cloud_ptr->clear();
//}
//
//int main(int argc, char **argv)
//{
//    ros::init(argc, argv, "talker");
//    ros::NodeHandle n;
//    ros::Subscriber sub = n.subscribe( "rslidar_points", 1, ShowResultInRviz );
//    g_chatter_pub = n.advertise< pcl::PointCloud< pcl::PointXYZ> >("chatter", 10);
//    ros::spin();
//
//    return 0;
//}



///++++++++++++++++++++++++++++++++++ Generate train data ++++++++++++++++++++++++++++++++++++++++++++++///

//static std::ofstream outfile;
//static ros::Publisher g_chatter_pub;
//static BootstrappingType *test_cloud = new BootstrappingType;
//static PointCloudXYZ::Ptr temp_show_cloud_ptr ( new PointCloudXYZ );
//static std::vector< PointFeatureStruct > point_feature_vec;
//static std::vector< float > response_vec;
//
//void GenerateTrainData( PointCloudXYZ::Ptr input_cloud_ptr )
//{
//    ///--- 1. pretreat point cloud data ---///
//    input_cloud_ptr->header.frame_id = "rslidar";
//    test_cloud->SetInputCloud( input_cloud_ptr );
//    test_cloud->ClassifyOriginCloudToGroundAndObstacle( differ_height_method );
//    test_cloud->ShowCloudInRvizInterface( temp_show_cloud_ptr );
//    temp_show_cloud_ptr->header.frame_id = "rslidar";
//    g_chatter_pub.publish( temp_show_cloud_ptr );
//    ///--- 2. write train data to txt file ---///
//    test_cloud->GetOpenCVDescionTreeTrainData( point_feature_vec, response_vec );
//    for (int i = 0; i < point_feature_vec.size(); ++i)
//    {
//        outfile << point_feature_vec[i].self_range << "," << point_feature_vec[i].remission << "," << point_feature_vec[i].left_minus_self << ","
//                << point_feature_vec[i].right_minus_self << "," << point_feature_vec[i].top_minus_self << "," << point_feature_vec[i].bottom_minus_self << ","
//                << point_feature_vec[i].lidar_height << "," << point_feature_vec[i].height_above_lowest << "," << response_vec[i] << std::endl;
//    }
//
//    temp_show_cloud_ptr->clear();
//    point_feature_vec.clear();
//    response_vec.clear();
//}
//
//int main(int argc, char **argv)
//{
//    outfile.open( "training data.txt", std::ios::app);
//    if(!outfile)
//        std::cerr << "Fail to open txt file!" << std::endl;
//
//    ros::init(argc, argv, "talker");
//    ros::NodeHandle n;
//    ros::Subscriber sub = n.subscribe( "rslidar_points", 1, GenerateTrainData );
//    ros::Rate loop_rate( 1 );
//    g_chatter_pub = n.advertise< pcl::PointCloud< pcl::PointXYZ> >("chatter", 2);
////    ros::spin();
//
//    int temp_count = 0;
//    while ( temp_count != 100 )
//    {
//        ros::spinOnce();
//        temp_count++;
//        loop_rate.sleep();
//    }
//
//    outfile.close();
//
//    return 0;
//}

///+++++++++++++++++++++++++++++++++++++++++++ Train decision tree +++++++++++++++++++++++++++++++++++++++++++++++++++++///
//const unsigned int g_train_data_size = 3000000;
//const unsigned int g_test_data_size = 1000000;
//static CvDTree *DS_Tree = new CvDTree;
//static BootstrappingType *test_cloud = new BootstrappingType;
//static float g_train_feature_data[g_train_data_size][8];
//static float g_train_response_data[g_train_data_size];
//static std::vector< std::vector< float > > train_data_vec;
//static std::vector<float> temp_single_train_data;
//unsigned int temp_count = 0;
//float train_data_0, train_data_1, train_data_2, train_data_3, train_data_4, train_data_5, train_data_6, train_data_7, train_data_8;
//
//void TrainDecisionTree()
//{
//    ///--- 1. read data from txt file ---///
//    FILE *in_file = fopen( "training data.txt", "r" );
//    if( in_file == NULL )
//        std::cerr << "Fail to open txt file!" << std::endl;
//
//    temp_single_train_data.resize( 9 );
//
//    while( temp_count != g_train_data_size )  // ! feof (in_file)
//    {
//        fscanf( in_file, "%f,%f,%f,%f,%f,%f,%f,%f,%f", &train_data_0, &train_data_1, &train_data_2,
//                                                       &train_data_3, &train_data_4, &train_data_5,
//                                                       &train_data_6, &train_data_7, &train_data_8 );
//        temp_single_train_data[0] = train_data_0;
//        temp_single_train_data[1] = train_data_1;
//        temp_single_train_data[2] = train_data_2;
//        temp_single_train_data[3] = train_data_3;
//        temp_single_train_data[4] = train_data_4;
//        temp_single_train_data[5] = train_data_5;
//        temp_single_train_data[6] = train_data_6;
//        temp_single_train_data[7] = train_data_7;
//        temp_single_train_data[8] = train_data_8;
//
//        train_data_vec.push_back( temp_single_train_data );
//        temp_count ++;
//    }
//    ///--- 2. trans data to OpenCv type ---
//    for (unsigned int i = 0; i < train_data_vec.size(); ++i)
//    {
//        for (int j = 0; j < 8; ++j)
//        {
//            g_train_feature_data[i][j] = train_data_vec[i][j];
//        }
//        g_train_response_data[i] = train_data_vec[i][8];
//    }
//    ///--- 3. set OpenCV decision tree parameters ---
//    cv::Mat feature_train_data( train_data_vec.size() - g_test_data_size, 8, CV_32F, g_train_feature_data );
//    cv::Mat response_train_data( train_data_vec.size() - g_test_data_size, 1, CV_32F, g_train_response_data );
//    float temp_cv_priors[] = { 1, 1 };
//    CvDTreeParams temp_cv_params( 20,       // the maximum possible depth of the tree
//                                  10,        // min sample count
//                                  0,        // regression accuracy: N/A here
//                                  true,     // compute surrogate split
//                                  15,       // max categories
//                                  0,        // cv folds
//                                  true,     // use 1se rule, smaller true, cut tree
//                                  true,     // throw away the pruned tree brunches
//                                  temp_cv_priors );// the priors array, the bigger p_weight, the more attention to the p_weight
//    ///--- 4. train decision tree model and calculate accuracy ---///
//    DS_Tree->train( feature_train_data, CV_ROW_SAMPLE, response_train_data, cv::Mat(), cv::Mat(),cv::Mat(),cv::Mat(), temp_cv_params );
//
//    cv::Mat feature_test_data( g_test_data_size, 8, CV_32F, g_train_feature_data + g_test_data_size );
//    cv::Mat response_test_data( g_test_data_size, 1, CV_32F, g_train_response_data + g_test_data_size );
//    cv::Mat predicted( g_test_data_size, 1, CV_32F );
//    for (int k = 0; k < g_test_data_size; ++k)
//    {
//        const cv::Mat sample = feature_test_data.row(k);
//        CvDTreeNode* prediction = DS_Tree->predict( sample );
//        predicted.at<float> (k, 0) = prediction->value;
//    }
//    std::cout << test_cloud->EvaluateDSTreePredictAccuracy( predicted, response_test_data ) << std::endl ;
//    train_data_vec.clear();
//    temp_single_train_data.clear();
//    ///--- 5. save to .xml file ---///
//    DS_Tree->save( "decision_tree_model.xml" );
//    std::cout << "Training done!" << std::endl;
//}
//
//int main(int argc, char **argv)
//{
//    TrainDecisionTree();
//
//    return 0;
//}

///+++++++++++++++++++++++++++++++++++++++++++ Use decision tree classify points +++++++++++++++++++++++++++++++++++++++++++++++++++++///

static ros::Publisher g_chatter_pub;
static BootstrappingType *test_cloud = new BootstrappingType;
static PointCloudXYZ::Ptr out_show_cloud_ptr ( new PointCloudXYZ );
static PointCLoudXYZRGB::Ptr out_color_cloud_ptr( new PointCLoudXYZRGB );
static clock_t time_start, time_finish;

void ClassifyCloudPointInDStreeModel( PointCloudXYZ::Ptr input_cloud_ptr )
{
    time_start = clock();
    input_cloud_ptr->header.frame_id = "rslidar";
    test_cloud->SetInputCloud( input_cloud_ptr );
    test_cloud->UseDSTreePredictCloud( out_show_cloud_ptr );
    out_show_cloud_ptr->header.frame_id = "rslidar";
    g_chatter_pub.publish( out_show_cloud_ptr );

//    test_cloud->EuclideanCluster( 0.6, 20, 10000, out_color_cloud_ptr );
//    out_color_cloud_ptr->header.frame_id = "rslidar";
//    g_chatter_pub.publish( out_color_cloud_ptr );

    out_show_cloud_ptr->clear();
    out_color_cloud_ptr->clear();

    time_finish=clock();
    std::cout << (float)(time_finish - time_start) / 1000  << " (ms) "<< std::endl;
}

int main(int argc, char **argv)
{
    test_cloud->LoadDStreeXML( "decision_tree_model.xml"  );

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe( "rslidar_points", 1, ClassifyCloudPointInDStreeModel );
    g_chatter_pub = n.advertise< pcl::PointCloud< pcl::PointXYZ> >("chatter", 10);
    ros::spin();

    return 0;
}
