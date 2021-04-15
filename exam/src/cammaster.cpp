#include "ros/ros.h"
#include "boost/thread.hpp"
#include <stdio.h>
#include <iostream>
#include <string>
#include <math.h>
#include <cmath>
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include "exam/custom1.h"
#include <tf2/utils.h>
#include <kdl/frames.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

class CAMMASTER {

	public:
		CAMMASTER();
		void run();
		void ctrl_loop();
		void sub_cb( exam::custom1 );

	private:
		ros::NodeHandle _nh;

        ros::Publisher _path_pub;
        ros::Publisher _end_pub;
		ros::Subscriber _sub;

		static const int _QR_ELEMENTS = 7;
		static const int _CAM_NUM = 4;
	 	bool found[_QR_ELEMENTS] = {0};
	 	int _found_counter = 0;

		float _x_samples[_QR_ELEMENTS][_CAM_NUM];
		float _y_samples[_QR_ELEMENTS][_CAM_NUM];
		float _yaw_samples[_QR_ELEMENTS][_CAM_NUM];
		int _samp_num[_QR_ELEMENTS] = {0};
};


CAMMASTER::CAMMASTER() {

	string topic_path;
	string topic_detection_end;

	_nh.getParam("/topic_path", topic_path);
	_nh.getParam("/topic_detection_end", topic_detection_end);

	_sub = _nh.subscribe("/new_detection", 10, &CAMMASTER::sub_cb, this);

    _path_pub = _nh.advertise<nav_msgs::Path>(topic_path, 0);
    _end_pub = _nh.advertise<std_msgs::Bool>(topic_detection_end, 0);
}


void CAMMASTER::sub_cb(exam::custom1 msg) {

	//Debug
	cout<<"[CAM: "<<msg.camname.data<<" | ID: "<<int(msg.id.data)<<" | x: "<<msg.spot.x<<" | y: "<<msg.spot.y<<" | yaw: "<<msg.spot.yaw<<"]"<<endl;

	//Data collecting
	_x_samples[int(msg.id.data)-1][_samp_num[int(msg.id.data)-1]] = msg.spot.x;
	_y_samples[int(msg.id.data)-1][_samp_num[int(msg.id.data)-1]] = msg.spot.y;
	_yaw_samples[int(msg.id.data)-1][_samp_num[int(msg.id.data)-1]] = msg.spot.yaw;

	_samp_num[int(msg.id.data)-1]++;

	//Control structure
	if(!found[int(msg.id.data)-1]){

		//Counter update
		found[int(msg.id.data)-1] = 1;
		_found_counter ++;

		//Debug
		cout<<"Found: "<<found[0]<<found[1]<<found[2]<<found[3]<<found[4]<<found[5]<<found[6]<<endl;
	}
}


void CAMMASTER::ctrl_loop() {

	ros::Rate r(100);

	float x_avg[_QR_ELEMENTS] = {0};
	float y_avg[_QR_ELEMENTS] = {0};
	float yaw_avg[_QR_ELEMENTS] = {0};

	tf2::Quaternion facing_quaternion;

	std_msgs::Bool end_detection;
	nav_msgs::Path path;
	path.poses.resize(_QR_ELEMENTS);

	//Wait till every marker is detected
	while(_found_counter!=_QR_ELEMENTS){

		r.sleep();
	}

	for(int i=0 ; i<_QR_ELEMENTS ; i++){

		//Average on cameras
		for(int j=0 ; j < _samp_num[i] ; j++){

			x_avg[i] = x_avg[i] + _x_samples[i][j];
			y_avg[i] = y_avg[i] + _y_samples[i][j];
			yaw_avg[i] = yaw_avg[i] + _yaw_samples[i][j];
		}

		x_avg[i] = x_avg[i] / _samp_num[i];
		y_avg[i] = y_avg[i] / _samp_num[i];
		yaw_avg[i] = yaw_avg[i] / _samp_num[i];

		//Debug
		cout<<"x_avg del QR "<<i+1<<": "<<x_avg[i]<<" su "<<_samp_num[i]<<" campioni."<<endl;
		cout<<"y_avg del QR "<<i+1<<": "<<y_avg[i]<<" su "<<_samp_num[i]<<" campioni."<<endl;
		cout<<"yaw_avg del QR "<<i+1<<": "<<yaw_avg[i]<<" su "<<_samp_num[i]<<" campioni."<<endl<<endl;

		//Message creation
		facing_quaternion.setRPY(0,0,yaw_avg[i]);

		path.poses.at(i).pose.position.x = x_avg[i];
		path.poses.at(i).pose.position.y = y_avg[i];
		path.poses.at(i).pose.position.z = 0;
		path.poses.at(i).pose.orientation.x = facing_quaternion.x();
		path.poses.at(i).pose.orientation.y = facing_quaternion.y();
		path.poses.at(i).pose.orientation.z = facing_quaternion.z();
		path.poses.at(i).pose.orientation.w = facing_quaternion.w();
	}

	//Kill nodes
	ROS_INFO("All positions detected.");
	system("rosnode kill /visp_auto_tracker1");
	system("rosnode kill /visp_auto_tracker2");
	system("rosnode kill /visp_auto_tracker3");
	system("rosnode kill /visp_auto_tracker4");
	system("rosnode kill /camnode1");
	system("rosnode kill /camnode2");
	system("rosnode kill /camnode3");
	system("rosnode kill /camnode4");

	end_detection.data = true;

    while(ros::ok()){
	    _path_pub.publish(path);
	    _end_pub.publish(end_detection);
    }
}


void CAMMASTER::run() {
	boost::thread ctrl_loop_t( &CAMMASTER::ctrl_loop, this );
	ros::spin();
}


int main(int argc, char** argv ) {

	ros::init(argc, argv, "CAMMASTER");

	CAMMASTER cm;
	cm.run();

	return 0;
}