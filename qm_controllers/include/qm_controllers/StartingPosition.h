// StartingPosition.h
#ifndef STARTINGPOSITION_H
#define STARTINGPOSITION_H

#include <cmath>

namespace qm {

    constexpr double X = -2; // Initial X position
    constexpr double Y = 0; // Initial Y position
    constexpr double PSI_DEG = 0; // Initial yaw angle in degrees
    constexpr double PSI = PSI_DEG * (M_PI / 180); 
    constexpr double ARM_DIST = 0.6; // Base CoM to End-effector distance in the XY plane
    constexpr double HEIGHT = 0.4; // Base CoM height (should be the same as comHeight in reference.info)
    constexpr double ARM_HEIGHT = 0.036; // Height difference between end-effector and base CoM
    constexpr double THRESHOLD = 0.1; // Distance threshold to check if the robot has reached its target

} // namespace qm

#endif // CONSTANTS_H
