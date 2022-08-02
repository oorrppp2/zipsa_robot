#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <iostream>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// darknet_ros_msgs
#include <vision_msgs/SegResult.h>

#include <std_msgs/String.h>


ros::Publisher pointcloud_pub;
std::string remove_switch;	// on : remove pc, off : publich the original pc
void callback(const sensor_msgs::ImageConstPtr& seg_result, const sensor_msgs::PointCloud2ConstPtr& pointcloud)
{
	if(remove_switch == "on") {
		// ROS_INFO("Synchronization successful");
		pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
		// pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
		pcl::fromROSMsg(*pointcloud, pcl_cloud);

		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(seg_result, sensor_msgs::image_encodings::TYPE_8UC1);
		}
		catch(cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv::Mat seg = cv_ptr->image;
		cv::Mat closed_seg;
		cv::morphologyEx(seg, closed_seg, cv::MORPH_CLOSE, cv::Mat());
		uchar* seg_data = closed_seg.data; 
		for (int row = 0; row < closed_seg.rows; row++) 
		{ 
			for (int col = 0; col < closed_seg.cols; col++) 
			{ 
				uchar p = seg_data[row * closed_seg.cols + col];
				if(p > 0) {
					pcl_cloud(col, row).x, pcl_cloud(col, row).y, pcl_cloud(col, row).z = 10;
				}
			}
		}
		// cv::imshow("imGray", closed_seg);
		// cv::waitKey(1);

		pointcloud_pub.publish(pcl_cloud);
	}
}

void remove_switching_callback(const std_msgs::StringConstPtr& msg) {
	remove_switch = msg->data;
}

void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud){
	if(remove_switch == "off") {
		pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
		// pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
		pcl::fromROSMsg(*pointcloud, pcl_cloud);
		pointcloud_pub.publish(pcl_cloud);
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "remove_segmentation_region_node");

	ros::NodeHandle nh_;
	ros::Subscriber remove_switching_sub;
	ros::Subscriber point_cloud_sub;

	remove_switch = "off";
	pointcloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/removed_object_points", 1);
	message_filters::Subscriber<sensor_msgs::Image> seg_result_sub(nh_, "/segmentation_result_Image", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh_, "/camera/depth_registered/points", 1);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), seg_result_sub, pcl_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	remove_switching_sub = nh_.subscribe<std_msgs::String>("/remove_points_switch", 1, &remove_switching_callback);
	point_cloud_sub = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, &point_cloud_callback);


    ros::spin();
    return 0;
}


