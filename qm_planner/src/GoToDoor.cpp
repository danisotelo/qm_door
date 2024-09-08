#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <qm_msgs/ee_state.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cmath>

#include "qm_controllers/StartingPosition.h"
#include "qm_vision/ComputeRef.h"

// Global variables to store the current pose of the end effector and target pose
geometry_msgs::Pose current_pose;
geometry_msgs::Pose target_pose;
geometry_msgs::Pose initial_target_pose;
geometry_msgs::Pose last_target_pose;
bool position_reached = false;
bool first_publish = true;
bool initial_position_reached = false;
bool door_initial_position_reached = false;
bool distance_threshold_met = false;
float distance_threshold = 0.5;
ros::Publisher gaitCommandPublisher;
tf::TransformListener* tf_listener;
ros::ServiceClient refClient;

bool isNewTargetPosition(const geometry_msgs::Pose& new_pose) {
    // Check if the new pose is different from the last published pose
    return new_pose.position.x != last_target_pose.position.x ||
           new_pose.position.y != last_target_pose.position.y ||
           new_pose.position.z != last_target_pose.position.z ||
           new_pose.orientation.x != last_target_pose.orientation.x ||
           new_pose.orientation.y != last_target_pose.orientation.y ||
           new_pose.orientation.z != last_target_pose.orientation.z ||
           new_pose.orientation.w != last_target_pose.orientation.w;
}

// Function to retrieve reference values from the service
bool getReferenceValues(double &x_ref, double &y_ref, double &psi_ref) {
    qm_vision::ComputeRef srv;
    ros::Duration retry_interval(5.0);  // 5-second interval

    // Wait for the service to become available
    if (!refClient.waitForExistence(ros::Duration(10.0))) {
        ROS_ERROR("Service get_ref is not available. Exiting...");
        return false;
    }

    while (ros::ok()) {
        if (refClient.call(srv)) {
            x_ref = srv.response.x_ref;
            y_ref = srv.response.y_ref;
            psi_ref = srv.response.psi_ref;

            if (!std::isnan(x_ref) && !std::isnan(y_ref) && !std::isnan(psi_ref)) {
                ROS_INFO("Retrieved from service - x_ref: %.2f, y_ref: %.2f, psi_ref: %.2f", x_ref, y_ref, psi_ref);
                return true;
            } else {
                ROS_WARN("Service returned NaN values. Retrying in 5 seconds...");
            }
        } else {
            ROS_WARN("Failed to call service get_ref. Retrying in 5 seconds...");
        }

        retry_interval.sleep();
    }

    return false;
}


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

// Callback function to update the distance threshold flag
void distanceCallback(const std_msgs::Float32::ConstPtr& msg){
    distance_threshold_met = (msg->data > distance_threshold);
}

void publishGaitCommand(const std::string& command) {
    std_msgs::String msg;
    msg.data = command;
    gaitCommandPublisher.publish(msg);
    ROS_INFO("Published gait command: %s", command.c_str());
}

// Function to calculate yaw from a quaternion
double getYawFromQuaternion(const geometry_msgs::Quaternion& q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "go_to_handle");
    ros::NodeHandle nh;

    // Initialize the transform listener
    tf_listener = new tf::TransformListener();

    // Initialize the service client
    refClient = nh.serviceClient<qm_vision::ComputeRef>("get_ref");

    ros::Subscriber eePoseSubscriber = nh.subscribe<qm_msgs::ee_state>("qm_mpc_observation_ee_state", 10, eePoseCallback);
    ros::Subscriber distanceSubscriber = nh.subscribe<std_msgs::Float32>("/door/distance", 10, distanceCallback);
    ros::Publisher positionCommandPublisher = nh.advertise<geometry_msgs::PoseStamped>("planner/cmd", 10);
    gaitCommandPublisher = nh.advertise<std_msgs::String>("/gait_command_topic", 10);
    
    ros::Rate loop_rate(10); // 10 Hz

    // Initial target position
    initial_target_pose.position.x = qm::X + qm::ARM_DIST * std::cos(qm::PSI);
    initial_target_pose.position.y = qm::Y + qm::ARM_DIST * std::sin(qm::PSI);
    initial_target_pose.position.z = 0.9;
    initial_target_pose.orientation = tf::createQuaternionMsgFromYaw(qm::PSI);

    target_pose = initial_target_pose; // Set the initial target pose

    position_reached = false; // Initial position has been sent, now wait for it to be reached

    double x_ref, y_ref, psi_ref;

    while (ros::ok()) {
        ros::spinOnce();

        // ROS_INFO("Loop variables: position_reached=%s, initial_position_reached=%s, distance_threshold_met=%s",
        //          position_reached ? "true" : "false",
        //          initial_position_reached ? "true" : "false"
        //          distance_threshold_met ? "true" : "false");

        // ROS_INFO("Target Pose - Position: (x: %.2f, y: %.2f, z: %.2f), Orientation: (w: %.2f, x: %.2f, y: %.2f, z: %.2f)",
        //         target_pose.position.x, target_pose.position.y, target_pose.position.z,
        //         target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);

        if (position_reached) {
            if (!initial_position_reached) {
                initial_position_reached = true;
                ROS_INFO("Initial position reached. Ready to move to door.");

                // Go to door starting position
                ROS_INFO("Getting reference values...");
                getReferenceValues(x_ref, y_ref, psi_ref);

                ROS_INFO("Changing from STANCE to TROT...");
                // Publish "trot" command
                publishGaitCommand("trot");

                // Wait for 5 seconds
                ros::Duration(5.0).sleep();

                // Calculate the yaw from the quaternion
                double yaw = getYawFromQuaternion(target_pose.orientation);
                target_pose.position.x += x_ref + qm::ARM_DIST * (std::cos(yaw + psi_ref) - std::cos(yaw));
                target_pose.position.y += y_ref - qm::ARM_DIST * (std::sin(yaw) - std::sin(yaw + psi_ref));
                target_pose.orientation = tf::createQuaternionMsgFromYaw(yaw + psi_ref);

                ROS_INFO("Target Pose - Position: (x: %.2f, y: %.2f, z: %.2f), Orientation: (w: %.2f, x: %.2f, y: %.2f, z: %.2f)",
                    target_pose.position.x, target_pose.position.y, target_pose.position.z,
                    target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);
                
                // Wait for 5 seconds
                // ros::Duration(5.0).sleep();    
                position_reached = false;

            } 
            else if (!door_initial_position_reached) {

                // Stop to take a better measurement
                publishGaitCommand("stance");

                // Wait for 2 seconds
                ros::Duration(2.0).sleep();

                // Check if the position is okay
                getReferenceValues(x_ref, y_ref, psi_ref);

                ROS_INFO("Checking position: (x_check: %.2f, y_check: %.2f, psi_check: %.2f)", x_ref, y_ref, psi_ref);

                if (std::max(std::sqrt(x_ref * x_ref + y_ref * y_ref), psi_ref) < qm::THRESHOLD){
                    door_initial_position_reached = true;
                    ROS_INFO("Door position reached. Ready to open it.");

                    // Wait for 2 seconds
                    ros::Duration(2.0).sleep();

                    // Add logic for reading the radius

                } else {
                    ROS_INFO("Repositioning...");

                    // Publish "trot" command
                    publishGaitCommand("trot");

                    // Wait for 2 seconds
                    ros::Duration(2.0).sleep();
                    
                    // Calculate the yaw from the quaternion
                    double yaw = getYawFromQuaternion(target_pose.orientation);
                    target_pose.position.x += x_ref + qm::ARM_DIST * (std::cos(yaw + psi_ref) - std::cos(yaw));
                    target_pose.position.y += y_ref - qm::ARM_DIST * (std::sin(yaw) - std::sin(yaw + psi_ref));
                    target_pose.orientation = tf::createQuaternionMsgFromYaw(yaw + psi_ref);
                    position_reached = false;
                }
            } 
            else {
                ROS_INFO_THROTTLE(5, "Target position reached. Waiting for new target...");
            }
        } else {
            if (first_publish || isNewTargetPosition(target_pose)){
            // Publish the current target position
            geometry_msgs::PoseStamped target_pose_msg;
            target_pose_msg.header.stamp = ros::Time::now();
            target_pose_msg.header.frame_id = "world";
            target_pose_msg.pose = target_pose;

            positionCommandPublisher.publish(target_pose_msg);
            
            // Update the last published pose
            last_target_pose = target_pose;
            first_publish = false;
            
            ROS_INFO_THROTTLE(1, "Published target position: (%.2f, %.2f, %.2f)",
                     target_pose_msg.pose.position.x, target_pose_msg.pose.position.y, target_pose_msg.pose.position.z);
            }

            // Wait for 2 seconds
            //ros::Duration(2.0).sleep();  
        }

        loop_rate.sleep();
    }

    delete tf_listener;

    return 0;
}
