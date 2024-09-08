//
// Created by skywoodsz on 2023/3/5.
//

#include "qm_controllers/QmTargetTrajectoriesPublisher.h"
#include "qm_controllers/GaitJoyPublisher.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

using namespace ocs2;
using namespace qm;

namespace {
    scalar_t TARGET_DISPLACEMENT_VELOCITY; // ee
    scalar_t TARGET_ROTATION_VELOCITY; // ee
    scalar_t COM_HEIGHT;
    vector_t DEFAULT_JOINT_STATE(18); //18
    scalar_t TIME_TO_TARGET;
}  // namespace

/**
 * calculate arrive time
 */
scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) {
    // displacement
    const scalar_t& dx = desiredBaseDisplacement(0);
    const scalar_t& dy = desiredBaseDisplacement(1);
    const scalar_t& dz = desiredBaseDisplacement(2);
    const scalar_t displacement = std::sqrt(dx * dx + dy * dy + dz * dz);
    const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;

    // rotation
    const scalar_t& dyaw = desiredBaseDisplacement(3);
    const scalar_t& droll = desiredBaseDisplacement(4);
    const scalar_t& dpitch = desiredBaseDisplacement(5);
    const scalar_t ratation = std::sqrt(dyaw * dyaw + droll * droll + dpitch * dpitch);
    const scalar_t rotationTime = ratation / TARGET_ROTATION_VELOCITY;

    return std::max(rotationTime, displacementTime);
}

TargetTrajectories targetPoseToTargetTrajectories(const vector_t& EeTargetPose,
                                                  const vector_t& BaseTargetPose,
                                                  const SystemObservation& observation,
                                                  const SystemObservation& eeState,
                                                  const scalar_t& targetReachingTime) {
    // desired time trajectory
    const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

    // desired state trajectory
    const vector_t EeCurrentPose = eeState.state;
    vector_t BaseCurrenPose = observation.state.segment<6>(6);
    BaseCurrenPose(2) = COM_HEIGHT;
    BaseCurrenPose(4) = 0;
    BaseCurrenPose(5) = 0;

    // x, ee_pose
    vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size() + 7));
    stateTrajectory[0] << vector_t::Zero(6), BaseCurrenPose, DEFAULT_JOINT_STATE, EeCurrentPose;
    stateTrajectory[1] << vector_t::Zero(6), BaseTargetPose, DEFAULT_JOINT_STATE, EeTargetPose;

    // desired input trajectory (just right dimensions, they are not used)
    const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

    return {timeTrajectory, stateTrajectory, inputTrajectory};
}

/**
 * publish dog traj.
 */
TargetTrajectories cmdVelToTargetTrajectories(const vector_t& cmdVel,
                                              vector_t& lastEeTarget,
                                              const SystemObservation& observation,
                                              const SystemObservation& eeState)
{
    const vector_t BaseCurrenPose = observation.state.segment<6>(6);
    const Eigen::Matrix<scalar_t, 3, 1> zyx = BaseCurrenPose.tail(3);
    vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3); // world frame

    const scalar_t timeToTarget = TIME_TO_TARGET;
    const vector_t BaseTargetPose = [&]() {
        vector_t target(6);
        target(0) = BaseCurrenPose(0) + cmdVelRot(0) * timeToTarget;
        target(1) = BaseCurrenPose(1) + cmdVelRot(1) * timeToTarget;
        target(2) = COM_HEIGHT;
        target(3) = BaseCurrenPose(3) + cmdVel(3) * timeToTarget;
        target(4) = 0;
        target(5) = 0;
        return target;
    }();

    const vector_t EeTargetPose = [&]() {
        if((lastEeTarget.head(3) - eeState.state.head(3)).norm() > 0.1)
            lastEeTarget.head(3) = eeState.state.head(3);

        vector_t target(7);
        target = lastEeTarget;
        return target;
    }();

    SystemObservation eeStateLast = eeState;
    eeStateLast.state = EeTargetPose;

    // target reaching duration
    const scalar_t targetReachingTime = observation.time + timeToTarget;
    auto trajectories = targetPoseToTargetTrajectories(EeTargetPose, BaseTargetPose, observation, eeStateLast, targetReachingTime);
    trajectories.stateTrajectory[0].head(3) = cmdVelRot;
    trajectories.stateTrajectory[1].head(3) = cmdVelRot;

    return trajectories;
}

/**
 * publish ee traj.
 */
TargetTrajectories EeCmdVelToTargetTrajectories(const vector_t& cmdVel,
                                                vector_t& lastEeTarget,
                                                const SystemObservation& observation,
                                                const SystemObservation& eeState) {
    // current pose
    const vector_t EeCurrentPose = eeState.state;
    const vector_t BaseCurrenPose = observation.state.segment<6>(6);
    const Eigen::Quaterniond quat_init(1.0, 0.0, 0.0, 0.0);
    const Eigen::Quaterniond quat(EeCurrentPose(6), EeCurrentPose(3), EeCurrentPose(4), EeCurrentPose(5));
    vector_t cmdVelRot = quat.toRotationMatrix() * quat_init.toRotationMatrix().transpose() * cmdVel.head(3); // world frame

    const scalar_t timeToTarget = TIME_TO_TARGET;
    const vector_t EeTargetPose =  [&](){
        vector_t target(7);
        target = EeCurrentPose;
        target(0) = EeCurrentPose(0) + cmdVelRot(0) * timeToTarget;
        target(1) = EeCurrentPose(1) + cmdVelRot(1) * timeToTarget;
        // target(2) = EeCurrentPose(2) + cmdVelRot(2) * timeToTarget;
        target(2) = lastEeTarget(2);
        target(3) = lastEeTarget(3);
        target(4) = lastEeTarget(4);
        target(5) = lastEeTarget(5);
        target(6) = lastEeTarget(6);

        // Heigh limit
        // if(target(2) - COM_HEIGHT > 0.52)
        //     target(2) = COM_HEIGHT + 0.52;

        return target;
    }();

    const vector_t BaseTargetPose = [&](){
        vector_t target(6);
        target = BaseCurrenPose;
        target(0) = EeTargetPose(0) - 0.6;
        target(1) = EeTargetPose(1);
        target(2) = COM_HEIGHT;
        target(4) = 0;
        target(5) = 0;
        return target;
    }();

    const scalar_t targetReachingTime = observation.time + timeToTarget;

    // traj
    return targetPoseToTargetTrajectories(EeTargetPose, BaseTargetPose, observation, eeState, targetReachingTime);
}

/**
 * Converts the pose of the interactive marker to TargetTrajectories.
 */
TargetTrajectories EEgoalPoseToTargetTrajectories(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                                  const SystemObservation& observation, const SystemObservation& eeState) {

    // current pose
    const vector_t EeCurrentPose = eeState.state;
    const vector_t BaseCurrenPose = observation.state.segment<6>(6);
    // target pose
    const vector_t EeTargetPose = (vector_t(7) << position, orientation.coeffs()).finished();

    const vector_t BaseTargetPose = [&](){
        vector_t target(6);
        target.setZero();
        target = BaseCurrenPose;
        target(0) = position(0) - 0.6;
        target(1) = position(1);
        target(2) = COM_HEIGHT;
        target(4) = 0.0;
        target(5) = 0;
        return target;
    }();

    // time
    const vector_t deltaError = [&](){
        vector_t delta(6);
        delta.segment<3>(0) = EeTargetPose.segment<3>(0) - EeCurrentPose.segment<3>(0);

        Eigen::Quaterniond q_current(EeCurrentPose[6], EeCurrentPose[3], EeCurrentPose[4], EeCurrentPose[5]);
        Eigen::Quaterniond q_target(EeTargetPose[6], EeTargetPose[3], EeTargetPose[4], EeTargetPose[5]);

        delta.segment<3>(3) = quaternionDistance(q_current, q_target);

        return delta;
    }();
    const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(deltaError);

    // traj
    return targetPoseToTargetTrajectories(EeTargetPose, BaseTargetPose, observation, eeState, targetReachingTime);
}

int main(int argc, char* argv[]) {
    const std::string robotName = "qm";
    const std::string gaitName = "legged_robot";

    ::ros::init(argc, argv, robotName + "_target");
    ::ros::NodeHandle nodeHandle;
    // Get node parameters
    std::string referenceFile;
    std::string taskFile;
    std::string gaitCommandFile;
    nodeHandle.getParam("/referenceFile", referenceFile);
    nodeHandle.getParam("/taskFile", taskFile);
    nodeHandle.getParam("/gaitCommandFile", gaitCommandFile);

    loadData::loadCppDataType(referenceFile, "comHeight", COM_HEIGHT);
    loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE);
    loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
    loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
    loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET);

    QmTargetTrajectoriesInteractiveMarker targetPoseCommand(nodeHandle, robotName,
                                                            &EEgoalPoseToTargetTrajectories,
                                                            &cmdVelToTargetTrajectories,
                                                            &EeCmdVelToTargetTrajectories);

    GaitJoyPublisher gaitCommand(nodeHandle, gaitCommandFile, gaitName, false);

    ros::spin();
    // Successful exit
    return 0;
}
