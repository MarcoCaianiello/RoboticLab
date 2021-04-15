#include "ros/ros.h"
#include "boost/thread.hpp"
#include <stdio.h>
#include <iostream>
#include <string>
#include <math.h>
#include <cmath>
#include <numeric>
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/PoseStamped.h"
#include "exam/custom1.h"
#include <tf2/utils.h>
#include <kdl/frames.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

class CAMNODE {

	public:
		CAMNODE(char**);
		void run();
		void ctrl_loop();
		void id_sub_cb( std_msgs::String );
		void pose_sub_cb( geometry_msgs::PoseStamped );
		void odom_sub_cb( nav_msgs::Odometry );
		void status_sub_cb( std_msgs::Int8 );

	private:	
		float _world_length;
		float _world_width;

		string _camname;

		ros::NodeHandle _nh;

		ros::Subscriber _id_sub;
		ros::Subscriber _pose_sub;
		ros::Subscriber _odom_sub;
		ros::Subscriber _status_sub;
        ros::Publisher _detection_pub;

		int _DETECTION_RATE;
		int _SAMPLES;
		const int _QR_ELEMENTS = 7;

		int _qr_id = 0;
		int _status = 0;

		KDL::Frame _frame_WR;
		KDL::Frame _frame_RC;
		KDL::Frame _frame_CM;
		KDL::Frame _frame_MS;
};


CAMNODE::CAMNODE(char** arg) {

	_camname = arg[1];

	float MS_OFFSET_X;
	float MS_OFFSET_Y;
	float MS_OFFSET_Z;
	float RC_OFFSET_X;
	float RC_OFFSET_Y;
	float RC_OFFSET_Z;
	float RC_OFFSET_YAW;

	string topic_odom;
	string topic_robot;

	//Retrieving values from parameter server
	_nh.getParam("/world_length", _world_length);
	_nh.getParam("/world_width", _world_width);

	_nh.getParam("/MS_offset_x", MS_OFFSET_X);
	_nh.getParam("/MS_offset_y", MS_OFFSET_Y);
	_nh.getParam("/MS_offset_z", MS_OFFSET_Z);
	_nh.getParam("/RC_offset_x"+ _camname, RC_OFFSET_X);
	_nh.getParam("/RC_offset_y"+ _camname, RC_OFFSET_Y);
	_nh.getParam("/RC_offset_z"+ _camname, RC_OFFSET_Z);
	_nh.getParam("/RC_offset_yaw"+ _camname, RC_OFFSET_YAW);
	_nh.getParam("/detection_rate", _DETECTION_RATE);
	_nh.getParam("/samples", _SAMPLES);

	_nh.getParam("/topic_first_tb3", topic_robot);
	_nh.getParam("/topic_odom", topic_odom);
	topic_odom = topic_robot + topic_odom;

	//Publishers and subscribers initialization
	_id_sub = _nh.subscribe("/visp_auto_tracker"+ _camname+"/code_message", 0, &CAMNODE::id_sub_cb, this);
	_pose_sub = _nh.subscribe("/visp_auto_tracker"+ _camname+"/object_position", 0, &CAMNODE::pose_sub_cb, this);
	_status_sub = _nh.subscribe("/visp_auto_tracker"+ _camname+"/status", 0, &CAMNODE::status_sub_cb, this);
	_odom_sub = _nh.subscribe(topic_odom, 0, &CAMNODE::odom_sub_cb, this);

    _detection_pub = _nh.advertise<exam::custom1>("/new_detection", 3);

    //Time invariant frame initilization
	_frame_RC.M = KDL::Rotation(0,0,1,-1,0,0,0,-1,0);
	_frame_RC.M.DoRotY(-RC_OFFSET_YAW);
	_frame_RC.M.DoRotX(0.3);
	_frame_RC.p = KDL::Vector(RC_OFFSET_X,RC_OFFSET_Y,RC_OFFSET_Z);

	_frame_MS.M = KDL::Rotation::Identity();
	_frame_MS.p = KDL::Vector(MS_OFFSET_X,MS_OFFSET_Y,MS_OFFSET_Z);
}

void CAMNODE::id_sub_cb(std_msgs::String id) {

	if(id.data.compare("")!=0){
		_qr_id = stoi(id.data);
	} else { _qr_id = 0; }
}

void CAMNODE::pose_sub_cb(geometry_msgs::PoseStamped msg) {

	_frame_CM.M = KDL::Rotation::Quaternion(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w);
	_frame_CM.p = KDL::Vector(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);
}

void CAMNODE::status_sub_cb(std_msgs::Int8 msg) {

	_status = msg.data;
}

void CAMNODE::odom_sub_cb(nav_msgs::Odometry msg) {

	_frame_WR.M = KDL::Rotation::Quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);
	_frame_WR.p = KDL::Vector(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z);
}

void CAMNODE::ctrl_loop() {
	
	ros::Rate r(_DETECTION_RATE);
	sleep(1); //Needed if the program is launched with marker in front of the camera

	KDL::Frame frame_WS;
	KDL::Frame frame_WM;
	KDL::Vector facing_direction;
	float facing_yaw;

	exam::custom1 msg;

	int index = 0;
	float x_samples[_SAMPLES];
	float y_samples[_SAMPLES];
	float yaw_samples[_SAMPLES];
	float x_avg;
	float y_avg;
	float yaw_avg;

	bool found[_QR_ELEMENTS] = {0};

	int fix_qr_id;

	while(ros::ok()){

		fix_qr_id = _qr_id;
		
		if((fix_qr_id==1)||(fix_qr_id==2)||(fix_qr_id==3)||(fix_qr_id==4)||(fix_qr_id==5)||(fix_qr_id==6)||(fix_qr_id==7)){
			
			if(!found[fix_qr_id-1]){

				index = 0;

				while((index < _SAMPLES)&&(_status == 3)){

					//Spot position wrt world frame
	    			frame_WM = _frame_WR*_frame_RC*_frame_CM;
					frame_WS = frame_WM*_frame_MS;

	    			//Get the orientation facing the maker
	    			facing_direction = frame_WM.p - frame_WS.p;
	    			facing_yaw = atan2(facing_direction[1],facing_direction[0]);

	    			//Update _SAMPLES vector
	    			x_samples[index] = frame_WS.p[0];
	    			y_samples[index] = frame_WS.p[1];
	    			yaw_samples[index] = facing_yaw;

	    			//Counter
					index++;
					r.sleep();
				}

				if(index == _SAMPLES){

					//Average
					x_avg = accumulate(x_samples,x_samples + _SAMPLES,0.0) / _SAMPLES;
					y_avg = accumulate(y_samples,y_samples + _SAMPLES,0.0) / _SAMPLES;
					yaw_avg = accumulate(yaw_samples,yaw_samples + _SAMPLES,0.0) / _SAMPLES;

					//Boundary check
					if((x_avg >= 0)&&(x_avg <= _world_length)&&(y_avg >= 0)&&(y_avg <= _world_width)){

						//Counter update
						found[fix_qr_id-1] = 1;

						//Message creation
						msg.camname.data = _camname;
						msg.id.data = fix_qr_id;
						msg.spot.x = x_avg;
		    			msg.spot.y = y_avg;
						msg.spot.yaw = yaw_avg;

		    			_detection_pub.publish(msg);
		    		}
				}

			}
		}

		r.sleep();	
    }
}


void CAMNODE::run() {
	boost::thread ctrl_loop_t( &CAMNODE::ctrl_loop, this );
	ros::spin();
}
    

int main(int argc, char** argv ) {

	ros::init(argc, argv, "camnode");

	CAMNODE cn(argv);
	cn.run();

	return 0;
}