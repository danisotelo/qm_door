//
// Created by skywoodsz on 2023/4/9.
//

#ifndef SRC_WBCBASE_H
#define SRC_WBCBASE_H

#include "qm_wbc/Task.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include "qm_wbc/WbcWeightConfig.h"

namespace qm{
using namespace ocs2;
using namespace legged_robot;

// Decision Variables: x = [\dot v^T, F^T]^T
class WbcBase{
    using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
    using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

public:
    WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
            const PinocchioEndEffectorKinematics& eeKinematics, const PinocchioEndEffectorKinematics& armEeKinematics,
            ros::NodeHandle &controller_nh);
    virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired,
                            const vector_t& rbdStateMeasured, size_t mode, scalar_t period, scalar_t time);

    virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

protected:
    void updateMeasured(const vector_t& rbdStateMeasured);
    void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired, ocs2::scalar_t period);
    vector_t updateCmd(vector_t x_optimal);

    size_t getNumDecisionVars() const { return numDecisionVars_; }

    Task formulateFloatingBaseEomTask();
    Task formulateTorqueLimitsTask();
    Task formulateNoContactMotionTask();
    Task formulateFrictionConeTask();

    Task formulateBaseHeightMotionTask();
    Task formulateBaseAngularMotionTask();
    Task formulateBaseXYLinearAccelTask();
    Task formulateSwingLegTask();

    Task formulateArmJointNomalTrackingTask();
    Task formulateEeLinearMotionTrackingTask();
    Task formulateEeAngularMotionTrackingTask();

    Task formulateContactForceTask(const vector_t& inputDesired) const;

    // with ee force
    Task formulateFloatingBaseEomWithEEForceTask();
    Task formulateTorqueLimitsTaskWithEEForceTask();
    vector_t updateCmdWithEEForce(vector_t x_optimal);

   void publishFeetCoordinates(const std::vector<Eigen::Vector3d>& posMeasured);

private:
    void dynamicCallback(qm_wbc::WbcWeightConfig& config, uint32_t /*level*/);

    size_t numDecisionVars_;
    PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
    CentroidalModelInfo info_;
    CentroidalModelPinocchioMapping mapping_;

    std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_, armEeKinematics_;

    contact_flag_t contactFlag_{};
    size_t numContacts_{};

    vector_t qMeasured_, vMeasured_, inputLast_;
    vector_t qDesired_, vDesired_, baseAccDesired_;
    vector_t jointAccel_;
    matrix_t j_, dj_;
    matrix_t arm_j_, arm_dj_;
    matrix_t base_j_, base_dj_;
    vector_t eeForce_;

    ::ros::Publisher z_feet_pub_;
    ::ros::Publisher z_feet_nc_pub_;

    // Task Parameters:
    std::shared_ptr<dynamic_reconfigure::Server<qm_wbc::WbcWeightConfig>> dynamic_srv_{};
    vector_t legTorqueLimits_, armTorqueLimits_;
    scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{};
    scalar_t baseHeightKp_{}, baseHeightKd_{};
    scalar_t baseAngularKp_{}, baseAngularKd_{};
    matrix_t jointKp_, jointKd_;
    matrix_t armEeLinearKp_{}, armEeLinearKd_{};
    matrix_t armEeAngularKp_{}, armEeAngularKd_{};
    size_t armEeFrameIdx_{};
};
}

#endif //SRC_WBCBASE_H
