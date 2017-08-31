#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <string>
#include <vector>

int x_coords[4] = {0, 1, 1, 0};
int y_coords[4] = {0, 0, 1, 0};
int z_coords[4] = {1, 1, 1, 1};
int acceptable_dist = 0; 

int i; // integer in vector
double dist; // distance between current pos and goal
double x_pos;
double y_pos;
double z_pos;
bool increment = 0; 
bool status = 0; 

//REMOVE THE ARMING CODE TO PREVENT NAV FROM CONSTANTLY TRYING TO ARM

class MavrosGuider {
	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_pose_;
		ros::Subscriber sub_state_;
		ros::ServiceClient client_arming_;
		ros::ServiceClient client_set_mode_;
		ros::Timer timer_pose_out_;

		geometry_msgs::PoseStamped msg_pose_out_;
		geometry_msgs::PoseStamped msg_current_pose_;
		mavros_msgs::State msg_current_state_;
		mavros_msgs::SetMode msg_set_mode_;
		mavros_msgs::CommandBool msg_arm_cmd_; //remove
		mavros_msgs::SetMode msg_set_land_;

		std::string topic_output_pose_;
		double rate_timer_;
		std::vector<geometry_msgs::Pose> pos_target; // pose_goal_;

	public:
		MavrosGuider() :
			nh_( ros::this_node::getName() ),
			rate_timer_( 20.0 ),
			topic_output_pose_( "/mavros/setpoint_position/local" ) {

			//Get parameters, or if not defined, use the defaults
			nh_.param( "topic_output_pose", topic_output_pose_, topic_output_pose_ );

			sub_state_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &MavrosGuider::state_cb, this);
			pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_output_pose_, 10 );
			client_arming_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
			client_set_mode_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

			msg_pose_out_.header.frame_id = "world";
			msg_pose_out_.pose.position.x = 0.0;
			msg_pose_out_.pose.position.y = 0.0;
			msg_pose_out_.pose.position.z = 1.0;
			msg_pose_out_.pose.orientation.x = 0.0;
			msg_pose_out_.pose.orientation.y = 0.0;
			msg_pose_out_.pose.orientation.z = 0.0;
			msg_pose_out_.pose.orientation.w = 1.0;

			timer_pose_out_ = nh_.createTimer(ros::Duration( 1 / rate_timer_ ), &MavrosGuider::timer_cb, this);

			msg_set_mode_.request.custom_mode = "OFFBOARD";
			msg_set_land_.request.custom_mode = "LAND";
			msg_arm_cmd_.request.value = true; //remove

			ROS_INFO("Waiting for FCU connection...");

			//Wait for FCU connection
			while( ros::ok() && !msg_current_state_.connected ){
				ros::spinOnce();
				ros::Rate(rate_timer_).sleep();
			}

			ROS_INFO("FCU detected!");
			ROS_INFO("Attempting to take control of UAV...");

			//Set up a stamp to keep track of requests, so we don't flood the FCU
		    ros::Time last_request = ros::Time(0);

			//Wait for Armed and in OFFBOARD mode
			while( ros::ok() && ( ( msg_current_state_.mode != "OFFBOARD" ) || ( !msg_current_state_.armed ) ) ) {

				if( ( ros::Time::now() - last_request ) > ros::Duration(5.0) ) {
					if( msg_current_state_.mode != "OFFBOARD" ) {
						if( client_set_mode_.call(msg_set_mode_) && msg_set_mode_.response.success ) {
							ROS_INFO("Offboard mode enabled!");
						}
					} else if( !msg_current_state_.armed ) {
						if( client_arming_.call(msg_arm_cmd_) && msg_arm_cmd_.response.success) {
							ROS_INFO("UAV armed!");
						}
					}

					last_request = ros::Time::now();
				}

				ros::spinOnce();
				ros::Rate(rate_timer_).sleep();
			}

			ROS_INFO("UAV is in autonomous mode");
		}

		~MavrosGuider() {
			//This message won't actually send here, as the node will have already shut down
			ROS_INFO("Shutting down...");
		}

		void state_cb( const mavros_msgs::State::ConstPtr& msg_in ) {
			msg_current_state_ = *msg_in;
		}

		void pose_cb( const geometry_msgs::PoseStamped msg_in ) {
			msg_current_pose_ = msg_in;
			msg_current_pose_.header.stamp = ros::Time::now();
			pub_pose_.publish( msg_current_pose_ );

			x_pos = msg_current_pose_.pose.position.x-x_coords[i];
			y_pos = msg_current_pose_.pose.position.y-y_coords[i];
			z_pos = msg_current_pose_.pose.position.z-z_coords[i];

			dist = sqrt((x_pos*x_pos)+(y_pos*y_pos)+(z_pos*z_pos));

			double tolerance = 0.5; //reduce if more accuracy is required
			if (tolerance < dist){
				printf("Travelling to waypoint\n");
				acceptable_dist = 0;
			}
			else{
				printf("Waypoint reached\n");
				acceptable_dist++;
				printf("Difference to waipoint coord: %d\n", acceptable_dist);
			}
			printf("Increment: %d\n", increment);

			if (acceptable_dist>100){
				acceptable_dist = 0;
				increment = 1;
				printf("Increment is now: %d\n", increment);
			}
			if(increment == 1){
				increment = 0;
				i++;
				if(i>pos_target.size()){
					if(client_set_mode_.call(msg_set_land_) && msg_set_land_.response.success){
						ROS_INFO("Initiating Land");
						status = 0; 	
					}
				}
			
			else if(status){
				msg_pose_out_.pose=pos_target[i];
				msg_pose_out_.header.stamp = ros::Time::now();
				pub_pose_.publish(msg_pose_out_);
				increment = 0;
			}

			msg_pose_out_.header.stamp = ros::Time::now();
			pub_pose_.publish(msg_pose_out_);
		}
	}

	void timer_cb(const ros::TimerEvent &t_e){
	msg_pose_out_.header.stamp = ros::Time::now();
	pub_pose_.publish(msg_pose_out_);
	}
};
int main(int argc, char** argv) {
	ros::init(argc, argv, "guider_cpp");
	MavrosGuider mg;

	ros::spin();

	return 0;
}
