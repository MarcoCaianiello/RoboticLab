#include "ros/ros.h"
#include "boost/thread.hpp"
#include "boost/lexical_cast.hpp"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <string>
#include<sstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


/*
Map limit:
x [0; 20.0]
y [0; 10.3]
*/

using namespace std;

class NAVIGATION {
	public:
		NAVIGATION();
		void run();
		void send_goal();
		void path_cb (const nav_msgs::Path& data);
		void set_user_path (string seq);
		string check_sequence (int, char**);

	private:
		ros::NodeHandle _nh;
		ros::Subscriber sub_path;
		move_base_msgs::MoveBaseGoal goal;
		string topic_robot;
		string topic_path;
		nav_msgs::Path path;
		bool enable_send_goal = false;
		vector<int> user_path;
};


NAVIGATION::NAVIGATION(){
	_nh.getParam("/topic_path", topic_path);
	_nh.getParam("/topic_second_tb3", topic_robot);
	
	sub_path = _nh.subscribe(topic_path, 0, &NAVIGATION::path_cb, this);
}

void NAVIGATION::path_cb(const nav_msgs::Path& data){
	if (!enable_send_goal) {
		ROS_INFO_STREAM("POSITIONS RECEIVED");
		path = data;
		enable_send_goal = true;
	}
}

void NAVIGATION::send_goal(){
	ros::Rate rate(1000);
	bool done = false;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac (topic_robot+"/move_base", true);
	goal.target_pose.header.frame_id = "map";

	ROS_INFO_STREAM("WAITING FOR THE SERVER");
	ac.waitForServer(); //will wait for infinite time
	
	while (ros::ok()){
		while (!enable_send_goal){ rate.sleep(); }
		
		
		//for (int i=0; i<path.poses.size(); i++){
		for (int i=0; i<user_path.size(); i++){
			
			int index = user_path.at(i);

			goal.target_pose.pose.position.x = path.poses.at( index ).pose.position.x;
			goal.target_pose.pose.position.y = path.poses.at( index ).pose.position.y;
			goal.target_pose.pose.position.z = path.poses.at( index ).pose.position.z;
			goal.target_pose.pose.orientation.w = path.poses.at( index ).pose.orientation.w;
			goal.target_pose.pose.orientation.x = path.poses.at( index ).pose.orientation.x;
			goal.target_pose.pose.orientation.y = path.poses.at( index ).pose.orientation.y;
			goal.target_pose.pose.orientation.z = path.poses.at( index ).pose.orientation.z;

			ROS_INFO_STREAM("QR ID "<< index+1 <<" TO REACH ["<< goal.target_pose.pose.position.x <<"; "<< goal.target_pose.pose.position.y <<"]");

			ac.sendGoal(goal);

			done = false;
			while ( !done ) {
			if ( ( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) || (ac.getState() == actionlib::SimpleClientGoalState::ABORTED) ){
				done = true;
				ros::Duration(1).sleep(); // sleep for a second
			}
			rate.sleep();
			}
		}

		ROS_INFO_STREAM("ALL POSITIONS REACHED");
		ros::shutdown();
		//enable_send_goal=false;

	}


}

void NAVIGATION::set_user_path (string seq) {
	user_path.clear();
	seq.erase(remove(seq.begin(), seq.end(), ' '), seq.end());

	for (int i=0; i<seq.length(); i++) { user_path.push_back(boost::lexical_cast<int>(seq[i])-1); }
	
}

string NAVIGATION::check_sequence (int len, char** seq){
	bool done = false;
	string user_sequence;
	istringstream qr_id;
	string temp;

	if (len == 1){
		ROS_INFO_STREAM("STANDARD SEQUENCE [1 2 3 4 5 6 7]");
		return "1234567";

	} else {
		for (int i=1; i<len; i++){
			user_sequence.append(seq[i]);
			user_sequence.append(" ");
		}
		user_sequence.erase(user_sequence.length()-1);
		len = user_sequence.length();

		while (!done){
			
				istringstream qr_id (user_sequence);
				temp.clear();
				int i = 1;
				bool out_of_bound = false;
				while ((qr_id >> temp) && (!out_of_bound)){
					if (i>7) {
						ROS_ERROR_STREAM("SEQUENCE LENGTH OUT OF BOUND!");
						done = false;
						out_of_bound = true;
					} else if ((boost::lexical_cast<int>(temp)>7) || (boost::lexical_cast<int>(temp)<=0)){
						ROS_ERROR_STREAM("NUMBER IN POSITION "<<i<<" OUT OF BOUND!");
						done = false;
						out_of_bound = true;
					} else {
						done = true;
					}
					i++;
				}
				

			if (!done) {
				ROS_INFO_STREAM("INSER NEW SEQUENCE");
				getline(cin, user_sequence);
				len = user_sequence.length();
			}

		}
	}
	
	ROS_INFO_STREAM("SEQUENCE ["<< user_sequence <<"] INSERT WITH SUCCEED");
	return user_sequence;

}



void NAVIGATION::run () {
	boost::thread send_goal_t( &NAVIGATION::send_goal, this );
	ros::spin();
}

int main( int argc, char** argv) {
	ros::init(argc, argv, "NAVIGATION");
	NAVIGATION client;
	client.set_user_path(client.check_sequence(argc, argv));
	client.run();
  	
}
