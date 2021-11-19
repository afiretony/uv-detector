#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>
#include <math.h>
#include <vector>
#include <time.h>
#include <UV_detector.h>
#include <kalman_filter.h>
#include <Eigen/Dense>

using namespace cv; 
using namespace std;
// struct Point{
//	float x;
//	float y;
//}

class my_detector
{  
	public:  
		my_detector()  
		{  
			image_transport::ImageTransport it(nh);
			//Topic subscribed 
			depsub = it.subscribe("/camera/depth/image_rect_raw", 1, &my_detector::run,this);
			// marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
			marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
		}  

    void run(const sensor_msgs::ImageConstPtr& msg)  
		{  
			// image conversion
			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
			cv::Mat depth = cv_ptr->image;
			// detect
			this->v_detector.readdata(depth);
			this->v_detector.detect();
			this->v_detector.track();
			this->v_detector.display_U_map();
			this->v_detector.display_bird_view();
			this->v_detector.display_depth();
			// cout << this->v_detector.bounding_box_B.size() << endl;
			// rviz visualization
			// ---------------

			// for publish
			// vector<Point2f> Points;
			visualization_msgs::Marker marker;
			visualization_msgs::MarkerArray markers;
			
			marker.header.frame_id = "/camera_link";
			// marker.header.stamp = ros::Time();
			// marker.ns = "my_namespace";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::ADD;

			//only if using a MESH_RESOURCE marker type:
			// marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
			// marker_pub.publish(marker);

			//----------------
			double u_r, u_l, d_b, d_t;
			for(int i = 0; i < this->v_detector.bounding_box_B.size(); i++)
			{
				Point2f obs_center = Point2f(this->v_detector.bounding_box_B[i].x + this->v_detector.bounding_box_B[i].width,
											this->v_detector.bounding_box_B[i].y + this->v_detector.bounding_box_B[i].height);
				/*
				 u_r = this->v_detector.bounding_box_U[i].br().x;
				 u_l = this->v_detector.bounding_box_U[i].tl().x;
				 d_b = this->v_detector.bounding_box_U[i].br().y;
				 d_t = this->v_detector.bounding_box_U[i].tl().y;
				 cout << u_r << endl;
				 cout << u_l << endl;
				 cout << d_b << endl;
				 cout << d_t << endl;
				 */
				cout << "object No." << i+1 << endl;
				cout << "x: " << obs_center.x << endl;
				cout << "y: " << obs_center.y << endl;
				// Points.push_back(obs_center);

				marker.lifetime = ros::Duration(0.05);
				marker.pose.position.x = obs_center.x / 100; // cm to m
				marker.pose.position.y = obs_center.y / 100; // cm to m
				marker.pose.position.z = 0.9;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				marker.scale.x = 0.2;
				marker.scale.y = 0.2;
				marker.scale.z = 0.9;
				marker.color.a = 0.7; // Don't forget to set the alpha!
				marker.color.r = abs(sin(i));
				marker.color.g = abs(cos(i));
				marker.color.b = (abs(cos(i)) + abs(sin(i))) / 2;
				markers.markers.push_back(marker);
				marker.id++;
			}
			marker_pub.publish(markers);
			// obspos_pub.publish(Points);

			/*
			markers.header.frame_id = "/camera_link";
			// marker.header.stamp = ros::Time();
			// marker.ns = "my_namespace";
			markers.id = marker.id;
			markers.type = visualization_msgs::Marker::SPHERE;
			markers.action = visualization_msgs::Marker::ADD;
			markers.color.a = 0.7; // Don't forget to set the alpha!
			markers.color.r = 0.6;
			markers.color.g = 0.2;
			markers.color.b = 0.3;
			*/

		}

	private:  
		ros::NodeHandle nh;   		// define node
    	image_transport::Subscriber depsub;		// define subscriber for depth image
		Vdetector v_detector;
		ros::Publisher marker_pub;
		// ros::Publisher obspos_pub;
};








class my_detector_u
{  
	public:  
		my_detector_u()  
		{  
			image_transport::ImageTransport it(nh);
			//Topic subscribed 
			depsub = it.subscribe("/camera/depth/image_rect_raw", 1, &my_detector_u::run,this);
			// marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
			marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
		}  

    void run(const sensor_msgs::ImageConstPtr& msg)  
		{  
			// image conversion
			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
			cv::Mat depth = cv_ptr->image;
			// detect
			this->uv_detector.readdata(depth);
			this->uv_detector.detect();
			this->uv_detector.track();
			this->uv_detector.display_U_map();
			this->uv_detector.display_bird_view();
			this->uv_detector.display_depth();
			// cout << this->uv_detector.bounding_box_B.size() << endl;
			// rviz visualization
			// ---------------

			// for publish
			// vector<Point2f> Points;
			visualization_msgs::Marker marker;
			visualization_msgs::MarkerArray markers;
			
			marker.header.frame_id = "/camera_link";
			// marker.header.stamp = ros::Time();
			// marker.ns = "my_namespace";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::ADD;

			//only if using a MESH_RESOURCE marker type:
			// marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
			// marker_pub.publish(marker);

			//----------------
			double u_r, u_l, d_b, d_t;
			for(int i = 0; i < this->uv_detector.bounding_box_B.size(); i++)
			{
				Point2f obs_center = Point2f(this->uv_detector.bounding_box_B[i].x + this->uv_detector.bounding_box_B[i].width,
											this->uv_detector.bounding_box_B[i].y + this->uv_detector.bounding_box_B[i].height);
				/*
				 u_r = this->uv_detector.bounding_box_U[i].br().x;
				 u_l = this->uv_detector.bounding_box_U[i].tl().x;
				 d_b = this->uv_detector.bounding_box_U[i].br().y;
				 d_t = this->uv_detector.bounding_box_U[i].tl().y;
				 cout << u_r << endl;
				 cout << u_l << endl;
				 cout << d_b << endl;
				 cout << d_t << endl;
				 */
				cout << "object No." << i+1 << endl;
				cout << "x: " << obs_center.x << endl;
				cout << "y: " << obs_center.y << endl;
				// Points.push_back(obs_center);

				marker.lifetime = ros::Duration(0.05);
				marker.pose.position.x = obs_center.x / 100; // cm to m
				marker.pose.position.y = obs_center.y / 100; // cm to m
				marker.pose.position.z = 0.9;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				marker.scale.x = 0.2;
				marker.scale.y = 0.2;
				marker.scale.z = 0.9;
				marker.color.a = 0.7; // Don't forget to set the alpha!
				marker.color.r = abs(sin(i));
				marker.color.g = abs(cos(i));
				marker.color.b = (abs(cos(i)) + abs(sin(i))) / 2;
				markers.markers.push_back(marker);
				marker.id++;
			}
			marker_pub.publish(markers);
			// obspos_pub.publish(Points);

			/*
			markers.header.frame_id = "/camera_link";
			// marker.header.stamp = ros::Time();
			// marker.ns = "my_namespace";
			markers.id = marker.id;
			markers.type = visualization_msgs::Marker::SPHERE;
			markers.action = visualization_msgs::Marker::ADD;
			markers.color.a = 0.7; // Don't forget to set the alpha!
			markers.color.r = 0.6;
			markers.color.g = 0.2;
			markers.color.b = 0.3;
			*/

		}

	private:  
		ros::NodeHandle nh;   		// define node
    	image_transport::Subscriber depsub;		// define subscriber for depth image
		UVdetector uv_detector;
		ros::Publisher marker_pub;
		// ros::Publisher obspos_pub;
};







int main(int argc, char **argv)  
{  
	//Initiate ROS  
	ros::init(argc, argv, "my_realsense_recorder");  

	//Create an object of class SubscribeAndPublish that will take care of everything  
	my_detector SAPObject;
	my_detector_u SAPObject_u;

	ros::spin();  
	return 0;  
} 
