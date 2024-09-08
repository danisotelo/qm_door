#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <qm_msgs/ee_state.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <cmath>

// Global variable to store the current pose of the end effector
geometry_msgs::Pose current_pose;
geometry_msgs::Pose target_pose;
geometry_msgs::Pose initial_target_pose;
bool initial_position_reached = false;
bool position_reached = false;
ros::Publisher gaitCommandPublisher;

// Callback function to update the current pose of the end effector
void eePoseCallback(const qm_msgs::ee_state::ConstPtr& msg) {
    current_pose.position.x = msg->state[0];
    current_pose.position.y = msg->state[1];
    current_pose.position.z = msg->state[2];
    current_pose.orientation.w = msg->state[6];
    current_pose.orientation.x = msg->state[3];
    current_pose.orientation.y = msg->state[4];
    current_pose.orientation.z = msg->state[5];

    // ROS_INFO("Received ee_state: (%.2f, %.2f, %.2f)", current_pose.position.x, current_pose.position.y, current_pose.position.z);

    // Check if the current position is close to the target position
    double dx = current_pose.position.x - target_pose.position.x;
    double dy = current_pose.position.y - target_pose.position.y;
    double dz = current_pose.position.z - target_pose.position.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance < 0.07) { // Threshold to determine if the position is reached
        position_reached = true;
        // ROS_INFO("Position reached: (%.2f, %.2f, %.2f)", current_pose.position.x, current_pose.position.y, current_pose.position.z);
    } else {
        position_reached = false;
        // ROS_INFO("Position not yet reached: (%.2f, %.2f, %.2f)", current_pose.position.x, current_pose.position.y, current_pose.position.z);
    }
}

void publishGaitCommand(const std::string& command) {
    std_msgs::String msg;
    msg.data = command;
    gaitCommandPublisher.publish(msg);
    ROS_INFO("Published gait command: %s", command.c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_circle");
    ros::NodeHandle nh;

    ros::Publisher positionCommandPublisher = nh.advertise<geometry_msgs::PoseStamped>("planner/cmd", 10);
    ros::Subscriber eePoseSubscriber = nh.subscribe<qm_msgs::ee_state>("qm_mpc_observation_ee_state", 10, eePoseCallback);
    gaitCommandPublisher = nh.advertise<std_msgs::String>("/gait_command_topic", 10);

    ros::Rate loop_rate(10); // 10 Hz

    double radius = 1.2;
    double centerX = -1.4;
    double centerY = 0.0;
    double centerZ = 1.0;
    double angle = 0.0;
    double angle_increment = 0.1; // Increment angle by 0.1 radians

    // Initial target position at the center of the circle
    geometry_msgs::PoseStamped initial_pose;
    initial_pose.header.stamp = ros::Time::now();
    initial_pose.header.frame_id = "world";
    initial_pose.pose.position.x = centerX;
    initial_pose.pose.position.y = centerY;
    initial_pose.pose.position.z = centerZ;
    initial_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    target_pose = initial_pose.pose; // Set the global target_pose to initial position

    positionCommandPublisher.publish(initial_pose);
    // ROS_INFO("Published initial target position: (%.2f, %.2f, %.2f)",
    //         initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z);

    position_reached = false; // Initial position has been sent, now wait for it to be reached

    while (ros::ok()) {
        ros::spinOnce();

        if (position_reached) {
            if (!initial_position_reached){
                initial_position_reached = true;
                ROS_INFO("Initial position reached. Ready to move.");

                // Publish "trot" command
                publishGaitCommand("trot");

                // Wait for 5 seconds
                ros::Duration(5.0).sleep();
                
                // Update target position
                geometry_msgs::PoseStamped new_target_pose;
                new_target_pose.header.stamp = ros::Time::now();
                new_target_pose.header.frame_id = "world";
                new_target_pose.pose.position.x = centerX + radius * std::cos(angle);
                new_target_pose.pose.position.y = centerY + radius * std::sin(angle);
                new_target_pose.pose.position.z = centerZ;
                new_target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0); //angle

                target_pose = new_target_pose.pose; // Update the global target_pose

                positionCommandPublisher.publish(new_target_pose);
                // ROS_INFO("Published new target position: (%.2f, %.2f, %.2f)", 
                //    new_target_pose.pose.position.x, new_target_pose.pose.position.y, new_target_pose.pose.position.z);

                // Increment angle
                angle += angle_increment;
                if (angle >= 2 * M_PI) {
                    angle = 0.0; // Reset angle to complete the circle
                }

                position_reached = false;
            } else {
                // Update target position
                geometry_msgs::PoseStamped new_target_pose;
                new_target_pose.header.stamp = ros::Time::now();
                new_target_pose.header.frame_id = "world";
                new_target_pose.pose.position.x = centerX + radius * std::cos(angle);
                new_target_pose.pose.position.y = centerY + radius * std::sin(angle);
                new_target_pose.pose.position.z = centerZ;
                new_target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0); //angle

                target_pose = new_target_pose.pose; // Update the global target_pose

                positionCommandPublisher.publish(new_target_pose);
                // ROS_INFO("Published new target position: (%.2f, %.2f, %.2f)", 
                //    new_target_pose.pose.position.x, new_target_pose.pose.position.y, new_target_pose.pose.position.z);

                // Increment angle
                angle += angle_increment;
                if (angle >= 2 * M_PI) {
                    angle = 0.0; // Reset angle to complete the circle
                }

                position_reached = false;
            }
        } else {
            // Republish the current target position
            geometry_msgs::PoseStamped current_target_pose;
            current_target_pose.header.stamp = ros::Time::now();
            current_target_pose.header.frame_id = "world";
            current_target_pose.pose = target_pose;

            positionCommandPublisher.publish(current_target_pose);
            // ROS_INFO("Republished current target position: (%.2f, %.2f, %.2f)",
            //         current_target_pose.pose.position.x, current_target_pose.pose.position.y, current_target_pose.pose.position.z);
        }

        loop_rate.sleep();
    }

    return 0;
}
