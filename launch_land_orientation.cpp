#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <string>
#include <vector>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
//#include <std_msgs/String.h>
// #include <sstream>


int acceptable_dist = 0; 

int i = 0; // integer in vector
double dist; // distance between current pos and goal
double x_pos;
double y_pos;
double z_pos;
bool increment = 0; 
bool mission_status = 0;
bool mission_complete = 0;

geometry_msgs::Vector3 toEuler(geometry_msgs::Quaternion q) {
    geometry_msgs::Vector3 e;

    double q2sqr = q.y * q.y;
    double t0 = -2.0 * (q2sqr + q.z * q.z) + 1.0;
    double t1 = +2.0 * (q.x * q.y + q.w * q.z);
    double t2 = -2.0 * (q.x * q.z - q.w * q.y);
    double t3 = +2.0 * (q.y * q.z + q.w * q.x);
    double t4 = -2.0 * (q.x * q.x + q2sqr) + 1.0;

    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;

    e.x = atan2(t3, t4);
    e.y = asin(t2);
    e.z = atan2(t1, t0);

    return e;
}


geometry_msgs::Quaternion toQuaternion(geometry_msgs::Vector3 e) {
    geometry_msgs::Quaternion q;

    double t0 = cos(e.z * 0.5);
    double t1 = sin(e.z * 0.5);
    double t2 = cos(e.x * 0.5);
    double t3 = sin(e.x * 0.5);
    double t4 = cos(e.y * 0.5);
    double t5 = sin(e.y * 0.5);

    q.w = t2 * t4 * t0 + t3 * t5 * t1;
    q.x = t3 * t4 * t0 - t2 * t5 * t1;
    q.y = t2 * t5 * t0 + t3 * t4 * t1;
    q.z = t2 * t4 * t1 - t3 * t5 * t0;

    return q;
}

//REMOVE THE ARMING CODE TO PREVENT NAV FROM CONSTANTLY TRYING TO ARM

class MavrosGuider {
	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_pose_;
		ros::Subscriber sub_state_;
		ros::Subscriber pos_sub; 
		ros::ServiceClient client_arming_;
		ros::ServiceClient client_set_mode_;
		ros::Timer timer_pose_out_;
                //ros::Publisher horizontal_state_;
                //ros::Rate loop_rate(10);

		geometry_msgs::PoseStamped msg_pose_out_;
		geometry_msgs::PoseStamped msg_current_pose_;
		mavros_msgs::State msg_current_state_;
		mavros_msgs::SetMode msg_set_mode_;
                mavros_msgs::CommandBool msg_arm_cmd_;
		mavros_msgs::SetMode msg_set_landing_;

		std::string topic_output_pose_;
		double rate_timer_;
		std::vector<geometry_msgs::Pose> pos_target; // pose_goal_;

	public:
		MavrosGuider() :
			nh_( ros::this_node::getName() ),
			rate_timer_( 20.0 ),
                        topic_output_pose_( "/mavel/setpoint/position" ) {

			//Get parameters, or if not defined, use the defaults
			nh_.param( "topic_output_pose", topic_output_pose_, topic_output_pose_ );

                        pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/vicon/UAVTAQG7/UAVTAQG7", 10, &MavrosGuider::pose_cb, this);
			sub_state_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &MavrosGuider::state_cb, this);
			pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_output_pose_, 10 );
			client_arming_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
			client_set_mode_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
                        //horizontal_state_ = nh_.advertise<std_msgs::String>("horizontal", 1000);

                        geometry_msgs::Vector3 temp_rotation;

			
                        msg_pose_out_.header.frame_id = "map";
			msg_pose_out_.pose.position.x = 0.0;
			msg_pose_out_.pose.position.y = 0.0;
                        msg_pose_out_.pose.position.z = 1.5; // was 1
                        //msg_pose_out_.pose.orientation.x = 0.0;
                        //msg_pose_out_.pose.orientation.y = 0.0;
                        //msg_pose_out_.pose.orientation.z = 0.0;
                        //msg_pose_out_.pose.orientation.w = 1.0; // was 1
                        temp_rotation.z = M_PI/2;
                        msg_pose_out_.pose.orientation = toQuaternion(temp_rotation);
                        pos_target.push_back(msg_pose_out_.pose);

                        // bottom left
                        msg_pose_out_.pose.position.x = -1.2;
                        msg_pose_out_.pose.position.y = 1.2;
                        pos_target.push_back(msg_pose_out_.pose);

                        // top left
                        msg_pose_out_.pose.position.x = 1.2;
                        msg_pose_out_.pose.position.y = 1.2;
                        pos_target.push_back(msg_pose_out_.pose);

                        // top right
                        msg_pose_out_.pose.position.x = 1.2;
                        msg_pose_out_.pose.position.y = -1.2;
                        pos_target.push_back(msg_pose_out_.pose);

                        // bottom right
                        msg_pose_out_.pose.position.x = -1.2;
                        msg_pose_out_.pose.position.y = -1.2;
                        pos_target.push_back(msg_pose_out_.pose);

                        // bottom left again
                        msg_pose_out_.pose.position.x = -1.2;
                        msg_pose_out_.pose.position.y = 1.2;
                        pos_target.push_back(msg_pose_out_.pose);

                        // centre
                        msg_pose_out_.pose.position.x = 0;
                        msg_pose_out_.pose.position.y = 0;
                        pos_target.push_back(msg_pose_out_.pose);

                        //pos_target.push_back(msg_pose_out_.pose);
                        //msg_pose_out_.pose.position.x = 1.0; //edit
                        //pos_target.push_back(msg_pose_out_.pose);
                        //msg_pose_out_.pose.position.y = 1.0;
                        //pos_target.push_back(msg_pose_out_.pose);
                        //msg_pose_out_.pose.position.x = 0.0;  // edit
                        //pos_target.push_back(msg_pose_out_.pose);
                        //msg_pose_out_.pose.position.y = 0.0;
                        //pos_target.push_back(msg_pose_out_.pose);



                        //msg_pose_out_.pose = pos_target[0];

			msg_set_mode_.request.custom_mode = "OFFBOARD";
			msg_set_landing_.request.custom_mode = "LAND";
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
                    timer_pose_out_ = nh_.createTimer(ros::Duration( 1 / rate_timer_ ), &MavrosGuider::timer_cb, this);

			//Wait for Armed and in OFFBOARD mode
			while( ros::ok() && ( ( msg_current_state_.mode != "OFFBOARD" ) || ( !msg_current_state_.armed ) ) ) {

				if( ( ros::Time::now() - last_request ) > ros::Duration(5.0) ) {
					if( msg_current_state_.mode != "OFFBOARD" ) {
                                                if( client_set_mode_.call(msg_set_mode_) && msg_set_mode_.response.mode_sent ) {
							ROS_INFO("Offboard mode enabled!");
						}
                                        }

					last_request = ros::Time::now();
				}

				ros::spinOnce();
				ros::Rate(rate_timer_).sleep();
			}

			ROS_INFO("UAV is in autonomous mode");
                        mission_status = true;

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

                        x_pos = msg_current_pose_.pose.position.x-pos_target[i].position.x;
                        y_pos = msg_current_pose_.pose.position.y-pos_target[i].position.y;
                        z_pos = msg_current_pose_.pose.position.z-pos_target[i].position.z;

                        geometry_msgs::Vector3 current_rpy = toEuler(msg_current_pose_.pose.orientation);
                        geometry_msgs::Vector3 goal_rpy = toEuler(pos_target[i].orientation);
                        double delta_yaw = fabs(current_rpy.z-goal_rpy.z);
			
			dist = sqrt((x_pos*x_pos)+(y_pos*y_pos)+(z_pos*z_pos));

			double tolerance = 0.1; //reduce if more accuracy is required
                        double yaw_tolerance = 0.17; // 10 degrees

                        if ((dist < tolerance) && (delta_yaw < yaw_tolerance)){
                            if(acceptable_dist == 0) {
                                ROS_INFO("Waypoint reached");
                            }

                            acceptable_dist++;
                            //ROS_INFO("Difference to waypoint coord: %d", acceptable_dist);
                        }else{
                            //ROS_INFO("Travelling to waypoint");
                            acceptable_dist = 0;
                        }
			
                        //ROS_INFO("Increment: %d", increment);

                        if (acceptable_dist>150){ // if within tolerance for 100 intervals?? // was <
				acceptable_dist = 0;
				increment = 1;
                                ROS_INFO("Stayed at waypoint long enough");
                                //ROS_INFO("Increment is now: %d", increment);
			}

                        if(mission_status && increment == 1)
                        {
                            increment = 0;
                            i++;
                            if( i < pos_target.size() )
                            {
                                msg_pose_out_.pose=pos_target[i];
                                //msg_pose_out_.header.stamp = ros::Time::now();
                                //pub_pose_.publish(msg_pose_out_);
                                increment = 0;
                                ROS_INFO("Moving to waypoint %i", i);
                                ROS_INFO("New waypoint location: [%0.2f,%0.2f,%0.2f]", msg_pose_out_.pose.position.x, msg_pose_out_.pose.position.y, msg_pose_out_.pose.position.z);

                            } else {
                                //if(client_set_mode_.call(msg_set_landing_) && msg_set_landing_.response.success){
                                //    ROS_INFO("Initiating Land");
                                //    mission_status = false; 	 // edit to set point to current location and initiate a landing (disarm if below 20cm)
                                //}
                                ROS_INFO("Initiating Land");
                                mission_status = false;
                                mission_complete = true;

                                msg_pose_out_.pose = msg_current_pose_.pose;
                                msg_pose_out_.pose.position.z = 0.0;

                            }
                        // insert the code so if mission status = false and height above ground is less than 20cm set to disarm


                        }
                        if(mission_complete == true && msg_current_pose_.pose.position.z<=0.25){
                           // msg_arm_cmd_.request.value = false;
                            //if( client_arming_.call(msg_arm_cmd_) && msg_arm_cmd_.response.success) {
                                    //ROS_INFO("UAV Disarmed!");
                            ROS_INFO("Please disarm the UAV");
                            //}

                        }
                }

	// ROS_INFO("Current location: %d", pub_pose_.c_str());

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
