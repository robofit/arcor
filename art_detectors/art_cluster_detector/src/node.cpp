#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <tf/transform_listener.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/common/centroid.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <visualization_msgs/Marker.h>
#include <pcl/segmentation/extract_clusters.h>




typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

class ClusterDetector {
public:
    ClusterDetector() {
        sub = nh.subscribe ("/kinect_pcd", 1, &ClusterDetector::cloud_cb, this);

        // Create a ROS publisher for the output point cloud
        pub = nh.advertise<sensor_msgs::PointCloud2> ("/output", 1);
        vis_pub = nh.advertise<visualization_msgs::Marker> ("/visualization", 1);
    }

private:
    ros::Publisher pub;
    ros::Publisher vis_pub;
    ros::Subscriber sub;
    tf::TransformListener listener_;
    ros::NodeHandle nh;



    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
    {

        ROS_INFO_STREAM_ONCE("First point cloud arrived");
        // Create a container for the data.

        sensor_msgs::PointCloud2 output;
        pcl::PCLPointCloud2 temp_cloud;
        pcl::IndicesPtr indices1 (new std::vector <int>), indices2 (new std::vector <int>);
        pcl_conversions::toPCL(*input, temp_cloud);
        temp_cloud.header.stamp = ros::Time::now().toSec()*1000000;
        PointCloudPtr pc(new PointCloud);
        pcl::fromPCLPointCloud2(temp_cloud, *pc);
        PointCloudPtr pc_transformed(new PointCloud);
        pcl_ros::transformPointCloud("/table", *pc, *pc_transformed, listener_);

        pcl::search::Search <PointType>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointType> > (new pcl::search::KdTree<PointType>);


        PointCloudPtr pc_filtered(new PointCloud);
        pcl::PassThrough<PointType> pass;
        pass.setInputCloud(pc_transformed);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0,0.7);
        pass.filter(*indices1);

        pass.setIndices(indices1);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(0,0.8);
        pass.filter(*indices2);

        pass.setIndices(indices2);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.05,0.2);
        pass.filter(*indices1);

        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud(pc);
        extract.setIndices(indices1);
        extract.filter(*pc_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointType> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (2500);
        ec.setSearchMethod (tree);
        ec.setInputCloud (pc_filtered);
        ec.extract (cluster_indices);

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*pc_filtered, it->indices, centroid);
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/kinect2_ir_optical_frame";
            marker.header.stamp = ros::Time::now();
            marker.ns = "my_namespace";
            marker.id = j;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = centroid[0];
            marker.pose.position.y = centroid[1];
            marker.pose.position.z = centroid[2];
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

            vis_pub.publish( marker );
            j++;

            ROS_INFO_STREAM("x: " << centroid[0] << " y: " << centroid[1] << " z: " << centroid[2]);
         }


        ROS_INFO_STREAM("size: " << pc_filtered->size());
        pcl::toROSMsg(*pc_filtered, output);
        pub.publish(output);

    }

};


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "art_cluster_detector_node");


  ClusterDetector detector;

  // Create a ROS subscriber for the input point cloud


  // Spin
  ros::spin();

}
