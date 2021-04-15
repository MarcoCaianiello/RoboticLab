#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"
#include "boost/thread.hpp"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <exception>
#include <string>
#include <time.h>

using namespace std;


class EXPLORATION {
	public:
		EXPLORATION();
		void end_cb ( std_msgs::BoolConstPtr );
		void ctrl_loop();
		vector<geometry_msgs::Point> get_points();
		geometry_msgs::Point get_goal(int);
		void run();
		

	private:
		ros::NodeHandle _nh;
		ros::Subscriber _end_sub;
		move_base_msgs::MoveBaseGoal goal;

		string topic_robot;
		string topic_detection_end;
		string map_command;

		bool end_detection = false;
		bool change = false;
		double world_length = 0.0;
		double world_width = 0.0;
		double out_dist_x = 0.0;
		double out_dist_y = 0.0;
		double in_dist_x = 0.0;
		double in_dist_y = 0.0;
		int side_points = 0;
		int in_points_x = 0;
		int in_points_y = 0;
		int dx = 0;
		int sx = 0;
		int counter = 0;
		vector<geometry_msgs::Point> points;

};


EXPLORATION::EXPLORATION() {
	_nh.getParam("/topic_first_tb3", topic_robot);
	_nh.getParam("/topic_detection_end", topic_detection_end);
	_nh.getParam("/map_abs_path", map_command);
	_nh.getParam("/world_length", world_length);
	_nh.getParam("/world_width", world_width);
	_nh.getParam("/out_dist_x", out_dist_x);
	_nh.getParam("/out_dist_y", out_dist_y);
	_nh.getParam("/in_dist_x", in_dist_x);
	_nh.getParam("/side_points", side_points);
	_nh.getParam("/in_points_x", in_points_x);
	_nh.getParam("/in_points_y", in_points_y);

	_end_sub = _nh.subscribe(topic_detection_end, 0, &EXPLORATION::end_cb, this);

	points.resize(2*side_points);
	map_command = "rosrun map_server map_saver -f " + map_command;

	//start index on right and left
	dx = 1;
	sx = side_points;

}

void EXPLORATION::end_cb ( std_msgs::BoolConstPtr msg) {
	if (!end_detection){ 

		end_detection = msg->data;

		try{
			system(map_command.c_str());
		} catch (exception& e) {
			ROS_ERROR_STREAM("Map not saved!");
		}
	}
}

void EXPLORATION::ctrl_loop() {
	ros::Rate rate(1000);
	bool done = false;
	points = get_points();
	geometry_msgs::Point temp;
	//for (int i=0; i<points.size(); i++){ ROS_INFO_STREAM("POINT TO REACH ["<< points.at(i).x <<"; "<< points.at(i).y <<"]");}

	int i = 0;

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac (topic_robot+"/move_base", true);
	goal.target_pose.header.frame_id = "map";

	ROS_INFO_STREAM("WAITING FOR THE SERVER ->>>>" << points.size() );
	ac.waitForServer(); //will wait for infinite time

	//while (ros::ok()){
	while (!(end_detection)){

		temp = get_goal(i);

		//update counter
		i++;
		
		goal.target_pose.pose.position.x = temp.x;
		goal.target_pose.pose.position.y = temp.y;
		goal.target_pose.pose.orientation.w = 0.1;

		ROS_INFO_STREAM("POINT TO REACH ["<< goal.target_pose.pose.position.x <<"; "<< goal.target_pose.pose.position.y <<"]");
		ac.sendGoal(goal);

		done = false;
		while ( !done ) {
		if ( ( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) || (ac.getState() == actionlib::SimpleClientGoalState::ABORTED) || end_detection) {
			done = true;
		}
		rate.sleep();
		}

		

	}

	
	
	goal.target_pose.pose.position.x = points.at(0).x;
	goal.target_pose.pose.position.y = points.at(0).y;
	goal.target_pose.pose.orientation.w = 0.1;

	ROS_INFO_STREAM("RETURN TO BASE ["<< goal.target_pose.pose.position.x <<"; "<< goal.target_pose.pose.position.y <<"]");
	ac.sendGoal(goal);

	done = false;
	while ( !done ) {
		if ( ( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) || (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)) {
			done = true;
		}
		rate.sleep();
	}
	
}

geometry_msgs::Point EXPLORATION::get_goal (int i){
	geometry_msgs::Point temp;
	int index = 0;
	srand (time(NULL));

	if (i<(2*side_points-1))
	{	
		if (!change){
			temp = points.at(sx);
			sx = sx + 1;
			if (sx >= (2*side_points)){
				sx = sx - side_points;
			}

		} else {
			temp = points.at(dx);
			dx = dx + 1;
			if (dx >= side_points){
				dx = dx - side_points;
			}
		}

		counter++;
		if (counter >= 2) {counter = 0; change = !change;}
		
	} else if (i<points.size()) {

		temp = points.at(i);
		
	} else {
		index = rand() % points.size();
		temp = points.at(index);
	}

	return temp;
}


vector<geometry_msgs::Point> EXPLORATION::get_points (){
	vector<geometry_msgs::Point> vett;
	geometry_msgs::Point temp;
	double ux = (world_length - 2*out_dist_x)/(side_points - 1);
	double uy = world_width - 2*out_dist_y;

	for (int j=0; j<2; j++){
		for (int i=0; i<side_points; i++){
			if ((i==0) && (j==0)){
				_nh.getParam("/first_tb3_x_pos", temp.x);
				_nh.getParam("/first_tb3_y_pos", temp.y);
			} else {
				temp.x = i*ux+out_dist_x;
				temp.y = j*uy+out_dist_y;
			}
			vett.push_back(temp);
		}
	}

	in_dist_y = uy/3.5;
	ux = (world_length - 2*in_dist_x)/(in_points_x - 1);
	uy = (world_width - 2*in_dist_y)/(in_points_y - 1);

	for (int j=0; j< in_points_y; j++){
		for (int i=0; i < in_points_x; i++){

			temp.x = i*ux+in_dist_x;
			temp.y = j*uy+in_dist_y;
			vett.push_back(temp);

		}
	}

	return vett;
}


void EXPLORATION::run() {
	boost::thread ctrl_loop_t( &EXPLORATION::ctrl_loop, this );
	ros::spin();
}


int main(int argc, char** argv ) {

	ros::init(argc, argv, "exploration");
	EXPLORATION explore;
	explore.run();
	return 0;
}
