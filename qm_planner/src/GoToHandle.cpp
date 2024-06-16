#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <qm_msgs/ee_state.h>
#include <tf/tf.h>
#include <cmath>

// Global variables to store the current pose of the end effector and target pose
geometry_msgs::Pose current_pose;
geometry_msgs::Pose target_pose;
geometry_msgs::Pose initial_target_pose;
bool position_reached = false;
bool initial_position_reached = false;
bool new_centroid_received = false;

// Callback function to update the current pose of the end effector
void eePoseCallback(const qm_msgs::ee_state::ConstPtr& msg) {
    current_pose.position.x = msg->state[0];
    current_pose.position.y = msg->state[1];
    current_pose.position.z = msg->state[2];
    current_pose.orientation.w = msg->state[6];
    current_pose.orientation.x = msg->state[3];
    current_pose.orientation.y = msg->state[4];
    current_pose.orientation.z = msg->state[5];

    ROS_INFO_THROTTLE(1, "Received ee_state: (%.2f, %.2f, %.2f)", current_pose.position.x, current_pose.position.y, current_pose.position.z);

    // Check if the current position is close to the target position
    double dx = current_pose.position.x - target_pose.position.x;
    double dy = current_pose.position.y - target_pose.position.y;
    double dz = current_pose.position.z - target_pose.position.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance < 0.07) { // Threshold to determine if the position is reached
        if (!position_reached) {
            position_reached = true;
            ROS_INFO("Position reached: (%.2f, %.2f, %.2f)", current_pose.position.x, current_pose.position.y, current_pose.position.z);
        }
    } else {
        position_reached = false;
    }
}

// Callback function to update the target pose based on the centroid
void centroidCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if (initial_position_reached) {
        target_pose.position.x = msg->point.x;
        target_pose.position.y = msg->point.y;
        target_pose.position.z = msg->point.z;
        target_pose.orientation = tf::createQuaternionMsgFromYaw(0); // Maintain orientation as needed

        new_centroid_received = true; // Indicate that a new centroid has been received
        ROS_INFO("Received new target centroid: (%.2f, %.2f, %.2f)", msg->point.x, msg->point.y, msg->point.z);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "go_to_handle");
    ros::NodeHandle nh;

    ros::Publisher positionCommandPublisher = nh.advertise<geometry_msgs::PoseStamped>("planner/cmd", 10);
    ros::Subscriber eePoseSubscriber = nh.subscribe<qm_msgs::ee_state>("qm_mpc_observation_ee_state", 10, eePoseCallback);
    ros::Subscriber centroidSubscriber = nh.subscribe<geometry_msgs::PointStamped>("handle/centroid", 10, centroidCallback);

    ros::Rate loop_rate(10); // 10 Hz

    // Initial target position
    initial_target_pose.position.x = -1.4;
    initial_target_pose.position.y = 0.0;
    initial_target_pose.position.z = 1.0;
    initial_target_pose.orientation = tf::createQuaternionMsgFromYaw(0);

    target_pose = initial_target_pose; // Set the initial target pose

    position_reached = false; // Initial position has been sent, now wait for it to be reached

    while (ros::ok()) {
        ros::spinOnce();

        if (position_reached) {
            if (!initial_position_reached) {
                initial_position_reached = true;
                ROS_INFO("Initial position reached. Ready to move to centroid.");
            } else if (new_centroid_received) {
                position_reached = false; // Start moving to the new centroid
                new_centroid_received = false; // Reset the flag
                ROS_INFO("Moving to new centroid position.");
            } else {
                ROS_INFO_THROTTLE(5, "Target position reached. Waiting for new target...");
            }
        } else {
            // Publish the current target position
            geometry_msgs::PoseStamped target_pose_msg;
            target_pose_msg.header.stamp = ros::Time::now();
            target_pose_msg.header.frame_id = "world";
            target_pose_msg.pose = target_pose;

            positionCommandPublisher.publish(target_pose_msg);
            ROS_INFO_THROTTLE(1, "Published target position: (%.2f, %.2f, %.2f)",
                     target_pose_msg.pose.position.x, target_pose_msg.pose.position.y, target_pose_msg.pose.position.z);
        }

        loop_rate.sleep();
    }

    return 0;
}
