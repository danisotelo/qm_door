#include "qm_controllers/GaitTopicPublisher.h"

#include <algorithm>

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/mode_schedule.h>
#include <ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h>

namespace qm {
using namespace ocs2;
using namespace legged_robot;

GaitTopicPublisher::GaitTopicPublisher(ros::NodeHandle nodeHandle, const std::string &gaitFile,
                                   const std::string &robotName, bool verbose) {
    ROS_INFO_STREAM(robotName + "_mpc_mode_schedule node is setting up ...");
    loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);

    modeSequenceTemplatePublisher_ = nodeHandle.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 1, true);

    gaitMap_.clear();
    for (const auto& gaitName : gaitList_) {
        gaitMap_.insert({gaitName, loadModeSequenceTemplate(gaitFile, gaitName, verbose)});
    }
    lastGaitCommand_ = "stance";
    ROS_INFO_STREAM(robotName + "_mpc_mode_schedule command node is ready.");

    cmdVelSub_ = nodeHandle.subscribe<std_msgs::String>("gait_command_topic", 1, &GaitTopicPublisher::gaitCommandCallback, this);
}

void GaitTopicPublisher::gaitCommandCallback(const std_msgs::String::ConstPtr &msg) {
    std::string gaitCommand = msg->data;

    if (lastGaitCommand_ != gaitCommand) {
        try {
            ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(gaitCommand);
            modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
            lastGaitCommand_ = gaitCommand;
            ROS_INFO_STREAM("Published gait command: " << gaitCommand);
        } catch (const std::out_of_range& e) {
            ROS_ERROR_STREAM("Gait \"" << gaitCommand << "\" not found.");
        }
    }
}

}  // namespace qm

int main(int argc, char** argv){
    ros::init(argc, argv, "gait_topic_publisher");
    ros::NodeHandle nh;

    std::string robotName = "legged_robot";
    std::string gaitCommandFile;
    nh.getParam("/gaitCommandFile", gaitCommandFile);

    qm::GaitTopicPublisher gaitTopicPublisher(nh, gaitCommandFile, robotName, true);

    ros::spin();
    return 0;
}
