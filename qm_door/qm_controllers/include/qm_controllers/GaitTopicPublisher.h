#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>

namespace qm {
using namespace ocs2;
using namespace legged_robot;

/** This class implements ModeSequence communication using ROS. */
class GaitTopicPublisher {
 public:
  GaitTopicPublisher(ros::NodeHandle nodeHandle, const std::string& gaitFile, const std::string& robotName, bool verbose = false);

  /** Callback to process commands received from a topic. */
  void gaitCommandCallback(const std_msgs::String::ConstPtr& msg);

 private:
  std::vector<std::string> gaitList_;
  std::map<std::string, ModeSequenceTemplate> gaitMap_;

  ros::Publisher modeSequenceTemplatePublisher_;
  ros::Subscriber cmdVelSub_;

  std::string lastGaitCommand_;
};

}  // namespace qm
