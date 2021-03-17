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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <iostream>

#include "convert_2d_to_3d/Result.h"

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include <std_msgs/String.h>

class ConvertBoundingBoxNode
{
    public:
        ConvertBoundingBoxNode()
        {
			pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, &ConvertBoundingBoxNode::pointcloud_callback, this);
			boundingboxes_sub_ = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, &ConvertBoundingBoxNode::boundingbox_callback, this);
			remove_request_sub_ = nh_.subscribe<std_msgs::String>("/remove_points_request", 1, &ConvertBoundingBoxNode::remove_request_callback, this);
			target_id_sub_ = nh_.subscribe<std_msgs::String>("/request_target", 1, &ConvertBoundingBoxNode::target_id_callback, this);
			pause_sub_ = nh_.subscribe<std_msgs::String>("/pause_request", 1, &ConvertBoundingBoxNode::pause_sub_callback, this);
			pub_result_ = nh_.advertise<convert_2d_to_3d::Result>("detected_object", 1);
			pointcloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/removed_object_points", 1);

			last_boundingbox_xmin = 0;
			last_boundingbox_xmax = 0;
			last_boundingbox_ymin = 0;
			last_boundingbox_ymax = 0;

            //cv::namedWindow("result", cv::WINDOW_AUTOSIZE);
            ROS_INFO("[%s] initialized...", ros::this_node::getName().c_str());
        }
        ~ConvertBoundingBoxNode()
        {
            delete pointcloud_sub_;
            delete boundingboxes_sub_;
            delete target_id_sub_;
        }

	void boundingbox_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& boundingbox) {
		gBoundingboxes.bounding_boxes.clear();
		for(int i = 0; i < boundingbox->bounding_boxes.size(); i++) {
		    gBoundingboxes.bounding_boxes.push_back(boundingbox->bounding_boxes[i]);
		}
	}

	void remove_request_callback(const std_msgs::StringConstPtr& msg) {
		remove_request = msg->data;
	}

	void target_id_callback(const std_msgs::StringConstPtr& msg) {
		target_id = msg->data;
	}

	void pause_sub_callback(const std_msgs::StringConstPtr& msg) {
		if(msg->data == "pause") {
			pause_state = true;
		} else if(msg->data == "resume") {
			pause_state = false;
		}
	}

	void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud)
	{
		if(pause_state) {
			pointcloud_pub.publish(paused_pcl_cloud);
			return;
		}
//	    printf("Bounding boxes size : %d\n", gBoundingboxes.bounding_boxes.size());
		pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
		// pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
		pcl::fromROSMsg(*pointcloud, pcl_cloud);
		if(!gBoundingboxes.bounding_boxes.empty()) {

			double x = 0.0;
			double y = 0.0;
			double z = 0.0;
			std::string id = "";
			pcl::PointXYZRGB mClosestPoint;
			bool isFistPoint = true;
			// std::cout << "======================================" << std::endl;

			for(int i = 0; i < gBoundingboxes.bounding_boxes.size(); i++) {
				id = gBoundingboxes.bounding_boxes[i].Class;
				if(id != target_id) {
					continue;
				}
				pcl::PointXYZRGB p = pcl_cloud( (gBoundingboxes.bounding_boxes[i].xmin + gBoundingboxes.bounding_boxes[i].xmax)/2,
												(gBoundingboxes.bounding_boxes[i].ymin + gBoundingboxes.bounding_boxes[i].ymax)/2);
				x = p.x;
				y = p.y;
				z = p.z;
				// std::cout << "id : " << id << " / " << p << std::endl;
				if(isFistPoint) {
					mClosestPoint = p;
					last_boundingbox_xmin = gBoundingboxes.bounding_boxes[i].xmin - remove_padding;
					last_boundingbox_xmax = gBoundingboxes.bounding_boxes[i].xmax + remove_padding;
					last_boundingbox_ymin = gBoundingboxes.bounding_boxes[i].ymin - remove_padding;
					last_boundingbox_ymax = gBoundingboxes.bounding_boxes[i].ymax + remove_padding;
					isFistPoint = false;
				} else if(z < mClosestPoint.z) {
					mClosestPoint = p;
					last_boundingbox_xmin = gBoundingboxes.bounding_boxes[i].xmin - remove_padding;
					last_boundingbox_xmax = gBoundingboxes.bounding_boxes[i].xmax + remove_padding;
					last_boundingbox_ymin = gBoundingboxes.bounding_boxes[i].ymin - remove_padding;
					last_boundingbox_ymax = gBoundingboxes.bounding_boxes[i].ymax + remove_padding;
				}

			}

			if(!isFistPoint) {
				x = mClosestPoint.x;
				y = mClosestPoint.y;
				z = mClosestPoint.z;
				if(remove_request == "remove") {
					for(int x = last_boundingbox_xmin; x < last_boundingbox_xmax; x++) {
						for(int y = last_boundingbox_ymin; y < last_boundingbox_ymax; y++) {
							pcl_cloud(x, y).x, pcl_cloud(x, y).y = INFINITY;
							pcl_cloud(x, y).z = INFINITY;
						}
					}
				}
				// Publish result
				convert_2d_to_3d::Result msg;
				msg.type = "detected_object";
				msg.data = target_id;

				msg.pose.header.stamp = ros::Time::now();
				msg.pose.header.frame_id = "object_coordinate";
				msg.pose.pose.position.x = x;
				msg.pose.pose.position.y = y;
				msg.pose.pose.position.z = z;

				msg.pose.pose.orientation.x = 0;
				msg.pose.pose.orientation.y = 0;
				msg.pose.pose.orientation.z = 0;
				msg.pose.pose.orientation.w = 1;

				pub_result_.publish(msg);
			}

			pointcloud_pub.publish(pcl_cloud);
			paused_pcl_cloud = pcl_cloud;

			ros::Duration(0.01).sleep();
			gBoundingboxes.bounding_boxes.clear();
		}
	}

    private:
        ros::NodeHandle nh_;
        tf2_ros::TransformBroadcaster tfb_;
	ros::Subscriber pointcloud_sub_;
	ros::Subscriber boundingboxes_sub_;
	ros::Subscriber remove_request_sub_;
	ros::Subscriber target_id_sub_;
	ros::Subscriber pause_sub_;
	ros::Publisher pub_result_;
	ros::Publisher pointcloud_pub;
	darknet_ros_msgs::BoundingBoxes gBoundingboxes;
	std::string target_id;
	std::string remove_request;
	pcl::PointCloud<pcl::PointXYZRGB> paused_pcl_cloud;

	int last_boundingbox_xmin;
	int last_boundingbox_xmax;
	int last_boundingbox_ymin;
	int last_boundingbox_ymax;

	int remove_padding = 20;
	bool pause_state = false;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "convert_boundingbox_node");
    ConvertBoundingBoxNode m = ConvertBoundingBoxNode();
    ros::spin();
    return 0;
}


