#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/region_growing_rgb.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <visualization_msgs/Marker.h>


ros::Publisher pub;
ros::Publisher vis_pub;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

void image_cb (const sensor_msgs::ImageConstPtr& input) {
    cv_bridge::CvImageConstPtr image_ptr = cv_bridge::toCvShare(input);
    cv::Mat image = image_ptr->image;

    cv::imshow("Window", image);
    cv::waitKey(10);
}

bool isSkin(const cv::Scalar& color) {
    cv::Mat input = cv::Mat(cv::Size(1, 1), CV_8UC3, color);
    cv::Mat output;

    cv::cvtColor(input, output, CV_RGB2YCrCb);
    cv::Mat skinCrCbHist = cv::Mat::zeros(cv::Size(256, 256), CV_8UC1);
    cv::ellipse(skinCrCbHist, cv::Point(113, 155.6), cv::Size(23.4, 15.2), 43.0, 0.0, 360.0, cv::Scalar(255, 255, 255), -1);

    cv::Vec3b ycrcb = output.at<cv::Vec3b>(0, 0);
    return ((skinCrCbHist.at<uchar>(ycrcb[1], ycrcb[2]) > 0));
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    ROS_INFO_STREAM_ONCE("First point cloud arrived");
    // Create a container for the data.
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 temp_cloud;
    pcl::IndicesPtr indices1 (new std::vector <int>), indices2 (new std::vector <int>);
    pcl_conversions::toPCL(*input, temp_cloud);
    PointCloudPtr pc(new PointCloud);
    pcl::fromPCLPointCloud2(temp_cloud, *pc);

    pcl::search::Search <PointType>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointType> > (new pcl::search::KdTree<PointType>);


    PointCloudPtr pc_filtered(new PointCloud);
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(pc);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-1.5,0.7);
    pass.filter(*indices1);

    pass.setIndices(indices1);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.4,0.3);
    pass.filter(*indices2);

    pass.setIndices(indices2);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.95,1.36);
    pass.filter(*indices1);

    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(pc);
    extract.setIndices(indices1);
    extract.filter(*pc_filtered);

    pcl::RegionGrowingRGB<PointType> reg;
    reg.setInputCloud(pc_filtered);
    //reg.setIndices(indices1);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(10);
    reg.setPointColorThreshold(6);
    reg.setRegionColorThreshold(5);
    reg.setMinClusterSize(150);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    ROS_INFO_STREAM(clusters.size());
    PointCloudPtr colored_cloud = reg.getColoredCloud ();

    extract.setInputCloud(pc_filtered);
    PointCloudPtr segment(new PointCloud);
    ROS_INFO_STREAM(clusters.size());
    int id = 0;
    for (int i = 0; i < clusters.size(); ++i) {
        //pcl::PointIndicesConstPtr test();
        pcl::PointIndices::Ptr tmp_cluster(new pcl::PointIndices(clusters[i]));
        extract.setIndices(tmp_cluster);
        extract.setNegative(false);
        extract.filter(*segment);
        int counter = 0;
        Eigen::Vector3i color_mean;
        PointType tl = segment->points[0];
        for (int i = 0; i < segment->size(); i += 10) {
            ++counter;
            color_mean += segment->points[i].getRGBVector3i();
            if (segment->points[i].y < tl.y) {
                tl = segment->points[i];
            }
        }
        color_mean /= float(counter);

        ROS_INFO_STREAM("false");
        if(isSkin(cv::Scalar(color_mean[0], color_mean[1], color_mean[2]))) {
            pcl::toROSMsg(*segment, output);
            pub.publish(output);
            ROS_INFO_STREAM("true");
            visualization_msgs::Marker marker;
            marker.header.frame_id = "kinect2_ir_optical_frame";
            marker.header.stamp = ros::Time();
            marker.ns = "my_namespace";
            marker.id = id;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = tl.x;
            marker.pose.position.y = tl.y;
            marker.pose.position.z = tl.z;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.03;
            marker.scale.y = 0.03;
            marker.scale.z = 0.03;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            //only if using a MESH_RESOURCE marker type:
            marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
            vis_pub.publish( marker );
            ++id;
        }

    }


    ROS_INFO_STREAM("size: " << pc->size() << "  filtered: " << indices1->size());


    //pcl::toROSMsg(*pc_filtered, output);
    //pub.publish(output);
    ROS_INFO_STREAM(pc->size());

}



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "art_simple_finger_point_detector_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/kinect_pcd", 1, cloud_cb);
  //ros::Subscriber sub_image = nh.subscribe("/kinect_image", 1, image_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/output", 1);
  vis_pub = nh.advertise<visualization_msgs::Marker> ("/visualization", 1);

  // Spin
  ros::spin();

}
