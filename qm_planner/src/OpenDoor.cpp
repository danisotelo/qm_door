#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <qm_msgs/ee_state.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h> 
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cmath>

#include "qm_vision/ComputeRadius.h"

// Global variables to store the current pose of the end effector and target pose
geometry_msgs::Pose current_pose;
geometry_msgs::Pose target_pose;
geometry_msgs::Pose last_target_pose;
geometry_msgs::Pose initial_target_pose;

bool position_reached = false;
bool initial_position_reached = false;
bool new_centroid_received = false;
bool distance_threshold_met = true;
bool first_publish = true;
bool start_opening = false;
bool door_opened = false;

float distance_threshold = 0.55; // Distance from which the centroid is no longer updated
float delta = 0.2; // Angle step
int door_type = 0;
float radius = -1;
float offset = 0.2; // Offset to push the door
float current_angle = 0.0;
float final_angle = 60;

ros::Publisher gaitCommandPublisher;
ros::Publisher fixedBase;
tf::TransformListener* tf_listener;

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
    if (initial_position_reached && distance_threshold_met) {
        geometry_msgs::PointStamped centroid_in_world;
        try {
            // Transform centroid coordinates from camera frame to world frame
            tf_listener->transformPoint("world", *msg, centroid_in_world);
            target_pose.position.x = centroid_in_world.point.x - 0.1;
            target_pose.position.y = centroid_in_world.point.y;
            target_pose.position.z = centroid_in_world.point.z + 0.06;
            target_pose.orientation = tf::createQuaternionMsgFromYaw(0);

            new_centroid_received = true;
            ROS_INFO("Received new target centroid: (%.2f, %.2f, %.2f)", centroid_in_world.point.x, centroid_in_world.point.y, centroid_in_world.point.z);

        } catch (tf::TransformException& ex) {
            ROS_WARN("Transform failed: %s", ex.what());
        }
    }
}

// Callback function to update the distance threshold flag
void distanceCallback(const std_msgs::Float32::ConstPtr& msg){
    if (msg->data < distance_threshold && initial_position_reached) {
        distance_threshold_met = false;
    }
}

void publishGaitCommand(const std::string& command) {
    std_msgs::String msg;
    msg.data = command;
    gaitCommandPublisher.publish(msg);
    ROS_INFO("Published gait command: %s", command.c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "go_to_handle");
    ros::NodeHandle nh;

    // Initialize the transform listener
    tf_listener = new tf::TransformListener();

    ros::Subscriber eePoseSubscriber = nh.subscribe<qm_msgs::ee_state>("qm_mpc_observation_ee_state", 10, eePoseCallback);
    ros::Subscriber centroidSubscriber = nh.subscribe<geometry_msgs::PointStamped>("handle/centroid", 10, centroidCallback);
    ros::Subscriber distanceSubscriber = nh.subscribe<std_msgs::Float32>("/door/distance", 10, distanceCallback);
    ros::Publisher positionCommandPublisher = nh.advertise<geometry_msgs::PoseStamped>("planner/cmd", 10);
    gaitCommandPublisher = nh.advertise<std_msgs::String>("/gait_command_topic", 10);
    ros::Publisher fixedBasePublisher = nh.advertise<std_msgs::Bool>("/fixed_base", 10);

    // Create a service client
    ros::ServiceClient radius_client = nh.serviceClient<qm_vision::ComputeRadius>("/compute_radius");

    ros::Rate loop_rate(10); // 10 Hz

    // Initial target position
    initial_target_pose.position.x = -1.4;
    initial_target_pose.position.y = 0.0;
    initial_target_pose.position.z = 1;
    initial_target_pose.orientation = tf::createQuaternionMsgFromYaw(0);

    target_pose = initial_target_pose; // Set the initial target pose

    position_reached = false; // Initial position has been sent, now wait for it to be reached

    while (ros::ok()) {
        ros::spinOnce();
        std_msgs::Bool fixed_base_msg;

        // ROS_INFO("Loop variables: position_reached=%s, initial_position_reached=%s, new_centroid_received=%s, distance_threshold_met=%s, door_type=%d, radius=%.2f",
        //     position_reached ? "true" : "false",
        //     initial_position_reached ? "true" : "false",
        //     new_centroid_received ? "true" : "false",
        //     distance_threshold_met ? "true" : "false",
        //     door_type,
        //     radius);


        // ROS_INFO("Target Pose - Position: (x: %.2f, y: %.2f, z: %.2f), Orientation: (w: %.2f, x: %.2f, y: %.2f, z: %.2f)",
        //         target_pose.position.x, target_pose.position.y, target_pose.position.z,
        //         target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);


        if (position_reached) {
            if (!initial_position_reached) {
                initial_position_reached = true;
                ROS_INFO("Initial position reached. Ready to move to centroid.");

                // Call the service to compute the radius (with retry logic)
                qm_vision::ComputeRadius srv;
                while (ros::ok()) {
                    if (radius_client.call(srv)) {
                        radius = srv.response.radius;
                        door_type = srv.response.typedoor;

                        if (radius > 0) {
                            ROS_INFO("Successfully computed the radius: %.2f", radius);

                            // Check the door type
                            if (door_type == 1) {
                                ROS_INFO("The door handle is on the right side.");
                            } else if (door_type == -1) {
                                ROS_INFO("The door handle is on the left side.");
                            } else {
                                ROS_WARN("Unknown door type received: %d", door_type);
                            }

                            break;  // Exit the loop after a successful call
                        } else {
                            ROS_WARN("Received an invalid radius.");
                        }
                    } else {
                        ROS_ERROR("Failed to call the compute_radius service. Retrying in 5 seconds...");
                    }

                    ros::Duration(2.0).sleep();  // Wait for 5 seconds before retrying
                }

                // Publish "trot" command
                publishGaitCommand("trot");

                // Wait for 3 seconds
                ros::Duration(3.0).sleep();
           
            } else if (new_centroid_received) {
                position_reached = false; // Start moving to the new centroid
                new_centroid_received = false; // Reset the flag
                ROS_INFO("Moving to new centroid position.");

            } else if (start_opening && (current_angle < final_angle * M_PI / 180)){
                ROS_INFO("Opening door... Door angle is: %.2f", current_angle);

                target_pose.position.x += (radius - offset) * sin(current_angle);
                double yaw = tf::getYaw(last_target_pose.orientation);

                if (door_type == 1) {
                    target_pose.position.y += (radius - offset) * (1 - cos(current_angle));
                    target_pose.orientation = tf::createQuaternionMsgFromYaw(yaw + current_angle);
                
                } else if (door_type == -1){
                    target_pose.position.y -= radius * (1 - cos(current_angle));
                    target_pose.orientation = tf::createQuaternionMsgFromYaw(yaw - current_angle);
                }

                if (current_angle >= final_angle * M_PI / 180){
                    ROS_INFO("Door fully opened.");
                    start_opening = false;
                    door_opened = true;
                }

                current_angle += delta;

            } else if (door_opened){
                ROS_INFO("Moving to final position...");
                target_pose.position.x += 1;
                target_pose.position.y += radius / 2;
                target_pose.orientation = tf::createQuaternionMsgFromYaw(0);

            } else {
                ROS_INFO_THROTTLE(5, "Target position reached. Waiting for new target...");
                if (!distance_threshold_met && !door_opened){
                    publishGaitCommand("stance");
                    ROS_INFO("Pushing the door...");
                    target_pose.position.x += 0.1;

                    // Publish the current target position
                    geometry_msgs::PoseStamped target_pose_msg;
                    target_pose_msg.header.stamp = ros::Time::now();
                    target_pose_msg.header.frame_id = "world";
                    target_pose_msg.pose = target_pose;
                    positionCommandPublisher.publish(target_pose_msg);

                    if (position_reached){
                        publishGaitCommand("trot");
                        // Wait for 3 seconds
                        ros::Duration(3.0).sleep();
                        ROS_INFO("Base to center...");
                        fixed_base_msg.data = true;
                        fixedBasePublisher.publish(fixed_base_msg);
                        start_opening = true;
                    }
                }
                if (door_opened){
                    ROS_INFO("Door successfully opened!");
                    publishGaitCommand("stance");
                }
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
        }

        loop_rate.sleep();
    }

    delete tf_listener;

    return 0;
}
