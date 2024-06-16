//
// Created by danisotelo on 2024/6/16.
//

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

void gaitCommandCallback(const std_msgs::String::ConstPtr& msg){
    std::string command = msg->data;

    // Create a temporary file to store the command
    std::ofstream tempFile("/tmp/gait_command.txt");
    if (tempFile.is_open()) {
        tempFile << command << std::endl;
        tempFile.close();
    } else {
        ROS_ERROR("Unable to open temporary file for writing");
        return;
    }

    // Use fork and exec to simulate the manual input with a temporary file
    pid_t pid = fork();

    if (pid == 0) {
        // Child process
        execlp("bash", "bash", "-c", "rosrun ocs2_legged_robot_ros legged_robot_gait_command < /tmp/gait_command.txt", (char *)NULL);
        // If exec fails
        perror("execlp");
        exit(EXIT_FAILURE);
    } else if (pid > 0) {
        // Parent process
        int status;
        waitpid(pid, &status, 0);
        if (WIFEXITED(status)) {
            ROS_INFO("Command executed with status: %d", WEXITSTATUS(status));
        }
    } else {
        // Fork failed
        perror("fork");
        exit(EXIT_FAILURE);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gait_command_simulator");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("gait_command_topic", 10, gaitCommandCallback);

    ros::spin();

    return 0;
}
