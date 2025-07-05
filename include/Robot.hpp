#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "iostream"
#include <algorithm>
#include <cctype>
#include "map"
#include "yaml-cpp/yaml.h"
#include "Error.hpp"
#include <math.h>
#include "chain.hpp"
#include "chainfksolverpos_recursive.hpp"
#include "chainiksolvervel_pinv.hpp"
#include "chainiksolverpos_nr.hpp"
#include "chainiksolverpos_nr_jl.hpp"
#include "chainfdsolver_recursive_newton_euler.hpp"
#include "chainidsolver_recursive_newton_euler.hpp"

#include "path.hpp"
#include "path_point.hpp"
#include "path_line.hpp"
#include "path_circle.hpp"
#include "path_composite.hpp"
#include "path_roundedcomposite.hpp"
#include "path_cyclic_closed.hpp"
#include "trajectory.hpp"
#include "trajectory.hpp"
#include "velocityprofile_dirac.hpp"
#include "velocityprofile_rect.hpp"
#include "velocityprofile_spline.hpp"
#include "velocityprofile_trap.hpp"
#include "velocityprofile_traphalf.hpp"
#include "rotational_interpolation.hpp"
#include "rotational_interpolation_sa.hpp"
#include "trajectory_segment.hpp"

enum class VelocityProfileType
{
    Dirac,
    Rectangular,
    Spline,
    Trapezoidal,
    TrapezoidalHalf
};

enum class PathType
{
    Point,
    Line,
    Circle,
    Composite,
    RoundedComposite,
    CyclicClosed
};

enum class RobotState
{
    IDLE,           // Robot is powered but not doing anything
    STANDING,       // Robot is standing in a ready position
    RUNNING,        // Robot is running
    EMERGENCY_STOP, // Robot is stopped due to emergency
    FAULT           // Some unrecoverable hardware or software fault
};

struct JointStates
{
    KDL::JntArray currentJointPositions;
    KDL::JntArray currentJointVelocities;
    KDL::JntArray currentJointAccelerations;
};

class Robot
{
public:
    Robot();
    ~Robot();
    int GetNumberOfSegments();
    int GetNumberOfJoints();
    Error LoadRobotFromYAML(const std::string &filename, const std::string &robot_name);
    std::string StringToLower(const std::string &str);
    KDL::Joint::JointType JointTypeFromString(const std::string &jointTypeString, const std::string &axisRevolutionTypeString);
    Error InitializeSolvers();
    Error CalculateForwardKinematics(const KDL::JntArray &jointAngles, KDL::Frame &currentCartesianFrame);
    Error CalculateInverseKinematics(const KDL::JntArray &currentJointAngles, const KDL::Frame &destinationFrame, KDL::JntArray &destinationJointAngles);
    Error CreatePath(const PathType pathType, KDL::Path *&path, const KDL::Frame startFrame = KDL::Frame::Identity(), const KDL::Frame endFrame = KDL::Frame::Identity(), const KDL::Vector circleCenterVector = KDL::Vector::Zero(), const KDL::Vector circlePointVector = KDL::Vector::Zero(), const KDL::Rotation circleRotation = KDL::Rotation::Identity(), double circleAngle = -1, double radius = -1);
    Error CreateVelocityProfile(const VelocityProfileType velocityProfileType, KDL::VelocityProfile *&velocityProfile, const double maximumVelocity, const double maximumAcceleration);
    Error CreateTrajectory(KDL::Trajectory *trajectory, KDL::Path *path, KDL::VelocityProfile *velocityProfile, const double sampleTime);
    Error SetGravity(const double x, const double y, const double z);
    std::string CheckSolverError(const int solverResult);
    Error CalculateForwardDynamics(const KDL::JntArray &currentJointAngles, const KDL::JntArray &currentJointVelocities, const KDL::JntArray &currentJointTorques, const KDL::Wrenches &externalForces, KDL::JntArray jointAccelerations);
    Error CalculateInverseDynamics(const KDL::JntArray &currentJointAngles, const KDL::Frame &destinationFrame, KDL::JntArray &destinationJointAngles);
    RobotState GetRobotState();

    std::string name;
    KDL::Vector gravity = KDL::Vector();

private:
    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive *forwardKinematicsSolverPositionRecursive = nullptr;
    KDL::ChainIkSolverVel_pinv *inverseKinematicsSolverVelocity = nullptr;
    KDL::ChainIkSolverPos_NR *inverseKinematicsSolverPosition = nullptr;
    KDL::ChainIkSolverPos_NR_JL *inverseKinematicsSolverPositionWithJointLimits = nullptr;
    KDL::ChainFdSolver_RNE *forwardDynamicSolver = nullptr;
    KDL::ChainIdSolver_RNE *inverseDynamicSolver = nullptr;
    KDL::JntArray minimumJointsLimit, maximumJointsLimit;
    KDL::Path_Point *pathPoint;
    KDL::Path_Line *pathLine;
    KDL::Path_Circle *pathCircle;
    KDL::Path_Composite *pathComposite;
    KDL::Path_RoundedComposite *pathRounded;
    KDL::Path_Cyclic_Closed *pathCyclicClosed;
    RobotState robotState;
    JointStates currentJointStates;
};

#endif // ROBOT_HPP