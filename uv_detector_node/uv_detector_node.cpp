#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sstream>
#include <math.h>
#include <vector>
#include <time.h>
#include <UV_detector.h>
#include <kalman_filter.h>
#include <Eigen/Dense>
#include <queue>
#include <librealsense2/rs.hpp>

using namespace cv; 
using namespace std;

class my_detector
{  
	public:  
		my_detector()  
		{  	
			image_transport::ImageTransport it(nh);
			
			//Topic subscribed 
			depsub = it.subscribe("/camera/depth/image_rect_raw", 1, &my_detector::depthCallback,this);
			imgsub = it.subscribe("/camera/color/image_raw", 1, &my_detector::imageCallback,this);
			
			// Topic published
			marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
			// obstacles = n.advertise<std_msgs::Float64MultiArray>("Obstacles", 1000); // working on

		}  
		void imageCallback(const sensor_msgs::ImageConstPtr& msg){

			cv_bridge::CvImagePtr cv_ptr;
			try{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
			}
			catch (cv_bridge::Exception& e){
				ROS_ERROR("Could not convert from '%s' to 'bgr8'.", e.what());
			return;
			}
			cv::Mat RGB = cv_ptr->image;
			this->uv_detector.readrgb(RGB);
		}

		void depthCallback(const sensor_msgs::ImageConstPtr& msg){
			cv_bridge::CvImagePtr cv_ptr;
			try{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1); 
			}
			catch (cv_bridge::Exception& e){
				ROS_ERROR("Could not convert from '%s' to 'TYPE_16UC1'.", e.what());
			return;
			}
			cv::Mat depth = cv_ptr->image;
			if (depthq.size()>2) {
				depthq.pop();
			}

			depthq.push(depth);

			this->uv_detector.readdata(depthq);
			this->uv_detector.detect();
			this->uv_detector.track();
			this->uv_detector.display_U_map();
			this->uv_detector.display_bird_view();
			this->uv_detector.extract_3Dbox();
			this->uv_detector.display_depth();

			visualization_msgs::Marker marker;
			visualization_msgs::MarkerArray markers;
			
			marker.header.frame_id = "/camera_link";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::ADD;

			double u_r, u_l, d_b, d_t;
			for(int i = 0; i < this->uv_detector.box3Ds.size(); i++)
			{
				cout<<"----------------------------"<<endl;
				cout<<"Object "<< i <<": "<<endl;
				cout<<"x: " << uv_detector.box3Ds[i].x<<endl;
				cout<<"y: " <<uv_detector.box3Ds[i].y<<endl;
				cout<<"z: " <<uv_detector.box3Ds[i].z<<endl;

				marker.lifetime = ros::Duration(0.05);
				marker.pose.position.x = uv_detector.box3Ds[i].x / 1000.; // convert from mm to m
				marker.pose.position.y = uv_detector.box3Ds[i].y / 1000.;
				marker.pose.position.z = uv_detector.box3Ds[i].z / 1000.;

				marker.scale.x = uv_detector.box3Ds[i].x_width / 1000.;
				marker.scale.y = uv_detector.box3Ds[i].y_width / 1000.;
				marker.scale.z = uv_detector.box3Ds[i].z_width / 1000.;

				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				
				marker.color.a = 0.7; // Don't forget to set the alpha!
				marker.color.r = abs(sin(i));
				marker.color.g = abs(cos(i));
				marker.color.b = (abs(cos(i)) + abs(sin(i))) / 2;
				markers.markers.push_back(marker);
				marker.id++;
			}
			marker_pub.publish(markers);
		}

	private:  
		queue<cv::Mat> depthq;
		ros::NodeHandle nh;   		// define node
    	image_transport::Subscriber depsub;		// define subscriber for depth image
		image_transport::Subscriber imgsub;
		UVdetector uv_detector;
		ros::Publisher marker_pub;
		// ros::Publisher obstacles; // working on
};

int main(int argc, char **argv)  
{  
	//Initiate ROS  
	ros::init(argc, argv, "my_realsense_recorder");  

	//Create an object of class SubscribeAndPublish that will take care of everything  
	my_detector SAPObject;

	ros::spin();  
	return 0;  
} 
