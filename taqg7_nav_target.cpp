#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <string>
#include <vector>

int acceptable_dist = 0;

int i = 0;
int target_counter = 0;
double dist;
double x_pos;
double y_pos;
double z_pos;
bool increment = 0;
bool mission_status = 0;
bool mission_complete = 0;
bool target_found = 0;

class MavrosGuider {
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_pose_;
        ros::Subscriber sub_state_;
        ros::Subscriber pos_sub;
        ros::ServiceClient client_arming_;
        ros::ServiceClient client_set_mode_;
        ros::Timer timer_pose_out_;
        // ros::      subscriber to IMP
        // ros::      publisher to GCS for target
        // ros::      publisher to servo 

        geometry_msgs::PoseStamped msg_pose_out_;
        geometry_msgs::PoseStamped msg_current_pose_;
        mavros_msgs::State msg_current_state_;
        mavros_msgs::SetMode msg_set_mode_;
        mavros_msgs::CommandBool msg_arm_cmd_;

        std::string topic_output_pose_;
        double rate_timer_;
        std::vector<geometry_msgs::Pose> pos_target;
        std::vector<geometry_msgs::Pose> target_pos;

    public:
        MavrosGuider() :
            nh_( ros::this_node::getName() ),
            rate_timer_(20.0),
            topic_output_pose_( "/mavel/setpoint/position") {
            //topic_output_pose_("/mavros/setpoint_position/local"){

            nh_.param( "topic_output_pose", topic_output_pose_, topic_output_pose_ );

            pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/vicon/UAVTAQG7/UAVTAQG7", 10, &MavrosGuider::pose_cb, this);
            sub_state_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &MavrosGuider::state_cb, this);
            pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_output_pose_, 10);
            client_arming_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
            client_set_mode_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
            timer_pose_out_ = nh_.createTimer(ros::Duration(1/rate_timer_), &MavrosGuider::timer_cb, this);
            // sub to IMP
            // pub to GCS
            // pub to Servo

            msg_pose_out_.header.frame_id = "world";
            msg_pose_out_.pose.position.x = 0.0;
            msg_pose_out_.pose.position.y = 0.0;
            msg_pose_out_.pose.position.z = 1.0; // altitude = 1.5m for actual search... 1m for testing
            msg_pose_out_.pose.orientation.x = 0.0;
            msg_pose_out_.pose.orientation.y = 0.0;
            msg_pose_out_.pose.orientation.z = 0.0;
            msg_pose_out_.pose.orientation.w = 1.0;
            pos_target.push_back(msg_pose_out_.pose);

            // Wall search waypoints

            // Left hand wall
            msg_pose_out_.pose.position.x = -1.5; // bottom left
            msg_pose_out_.pose.position.y = 1.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 0.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 1.5; // Top left
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.y = 1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.y = 0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.y = 0.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.y = -0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.y = -1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.y = -1.5; // top right
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 0.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -1.5; // bottom right
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.y = -1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.y = -0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.y = 0.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.y = 0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.y = 1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.y = 1.5; // bottom left
            pos_target.push_back(msg_pose_out_.pose);

            // Horizontal search

            msg_pose_out_.pose.position.x = -1.5; // start line 1
            msg_pose_out_.pose.position.y = 1.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 0.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 1.5; // Top left
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 1.5; // start of second line
            msg_pose_out_.pose.position.y = 1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 0.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -1.5; // bottom of line 2
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -1.5; // start line 3
            msg_pose_out_.pose.position.y = 0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 0.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 1.5; // end line 3
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 1.5; // start line 4
            msg_pose_out_.pose.position.y = 0.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 0.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -1.5; // end line 4
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -1.5; // start line 5
            msg_pose_out_.pose.position.y = -0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 0.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 1.5; // end line 5
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 1.5; // start line 6
            msg_pose_out_.pose.position.y = -1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 0.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -1.5; // end line 6
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -1.5; // start line 7
            msg_pose_out_.pose.position.y = -1.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = -0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 0.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 0.5;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 1.0;
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose.position.x = 1.5; // end of line 7
            msg_pose_out_.pose.position.y = -1.5; // top right
            pos_target.push_back(msg_pose_out_.pose);

            msg_pose_out_.pose = pos_target[0];  // return to launch site
            pos_target.push_back(msg_pose_out_.pose);

            // end of horizontal search //


            msg_set_mode_.request.custom_mode = "OFFBOARD";
            msg_arm_cmd_.request.value = true;

            ROS_INFO("Waiting for FCU connection...");

            // Wait for FCU connection
            while( ros::ok() && !msg_current_state_.connected)
            {
                ros::spinOnce();
                ros::Rate(rate_timer_).sleep();
            }

            ROS_INFO("FCU detected!");
            ROS_INFO("Attempting to take control of UAVTAQG7...");

            // Set up a stamp to keep track of requests, so we don't flood the FCU
            ros::Time last_request = ros::Time(0);

            // Wait for armed and in offboard
            while( ros::ok() && ( (msg_current_state_.mode != "OFFBOARD") || (!msg_current_state_.armed) ) )
            {
                if( (ros::Time::now() - last_request) > ros::Duration(5.0) )
                {
                    if( msg_current_state_.mode != "OFFBOARD")
                    {
                        if( client_set_mode_.call(msg_set_mode_) && msg_set_mode_.response.mode_sent)
                        {
                            ROS_INFO("Offboard mode enabled!");
                        }
                    }

                    last_request = ros::Time::now();
                }

                ros::spinOnce();
                ros::Rate(rate_timer_).sleep();
            }

            ROS_INFO("UAVTAQG7 is in autonomous mode");
            mission_status = true;

            }

            ~MavrosGuider()
            {
                ROS_INFO("Shutting down...");
            }

            void state_cb( const mavros_msgs::State::ConstPtr& msg_in)
            {
                msg_current_state_ = *msg_in;
            }


            void pose_cb( const geometry_msgs::PoseStamped msg_in)
            {
                msg_current_pose_ = msg_in;
                msg_current_pose_.header.stamp = ros::Time::now();

                x_pos = msg_current_pose_.pose.position.x - pos_target[i].position.x;
                y_pos = msg_current_pose_.pose.position.y - pos_target[i].position.y;
                z_pos = msg_current_pose_.pose.position.z - pos_target[i].position.z;

                dist = sqrt((x_pos * x_pos) + (y_pos * y_pos) + (z_pos * z_pos));

                if(target_found == true)
                {
                    target_counter++;

                    ROS_INFO("Target Identified!");
                    ROS_INFO("Redirecting UAVTAQG7 to target");

                    target_x = msg_current_pose_.pose.position.x + x_target;
                    target_y = msg_current_pose_.pose.position.y + (-1*y_target);
                    target_z = msg_current_pose_.pose.position.z + z_target;

                    if(target_x > 1.5)
                    {
                        target_x = 1.5;
                    }
                    if(target_y > 1.5)
                    {
                        target_y = 1.5;
                    }
                    if(target_x > -1.5)
                    {
                        target_x = -1.5;
                    }
                    if(target_y > -1.5)
                    {
                        target_y = -1.5;
                    }
                    if(target_z < 1)
                    {
                        target_z = 1;
                    }

                    msg_pose_out_.pose.position.x = target_x;
                    msg_pose_out_.pose.position.y = target_y;
                    msg_pose_out_.pose.position.z = target_z;
                    pos_target.push_back(msg_pose_out_.pose);

                    target_pos.push_back(msg_pose_out_.pose);

                    if(msg_current_pose_.pose == target_pos[target_counter].pose)
                    {
                        ROS_INFO("Target reached!");
                        ROS_INFO("Initiating 10 second hover...");
                        hover_start = ros::Time::now();
                        while(ros::Time::now() hover_start > ros::Duration(10.0))
                        {
                            msg_pose_out_.pose = msg_current_pose_.pose;
                            pos_target.push_back(msg_current_pose_.pose);
                        }
                    }
                }

                double tolerance = 0.1;

                if(tolerance < dist){
                    acceptable_dist = 0;
                }

                else
                {
                    if(acceptable_dist == 0)
                    {
                        ROS_INFO("Waypoint reached!");
                    }

                    acceptable_dist++;
                }

                if(acceptable_dist>100)
                {
                    acceptable_dist = 0;
                    increment = 1;
                    ROS_INFO("Stayed at waypoint long enough");
                }

                if(increment == 1)
                {
                    increment = 0;
                    i++;

                    if(mission_status)
                    {
                        if(i < pos_target.size())
                        {
                            msg_pose_out_.pose = pos_target[i];
                            increment = 0;
                            ROS_INFO("Moving to waypoint %i", i);
                            ROS_INFO("New waypoint location: [%0.2f, %0.2f, %0.2f]", msg_pose_out_.pose.position.x, msg_pose_out_.pose.position.y, msg_pose_out_.pose.position.z);
                        }

                        else
                        {
                            ROS_INFO("Search complete!");
                            ROS_INFO("Initiating UAVTAQG7 land");
                            mission_status = false;
                            mission_complete = true;

                            msg_pose_out_.pose = msg_current_pose_.pose;
                            msg_pose_out_.pose.position.z = 0.0;
                        }
                    }
                }

                if(mission_complete == true && msg_current_pose_.pose.position.z <= 0.25)
                {
                    ROS_INFO("Please disarm UAVTAQG7!");
                }
            }

            void timer_cb( const ros::TimerEvent &t_e)
            {
                msg_pose_out_.header.stamp = ros::Time::now();
                pub_pose_.publish(msg_pose_out_);
            }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "guider_cpp");
    MavrosGuider mg;

    ros::spin();

    return 0;
}