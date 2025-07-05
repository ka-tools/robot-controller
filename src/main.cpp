#include "main.hpp"


int main()
{
    Logger logger;
    logger.log("Program Started!");
    Robot robot;
    Error error = robot.LoadRobotFromYAML("../Configuration/RobotConfiguration.yaml", "SixAxisBotA");
    if (error.hasError())
    {
        logger.log(error.GetErrorMessage());
        return 0;
    }
    // InitializeCyclic(10, &robot);

    // mainLoop(menu);

    KDL::JntArray joints(6);
    joints(4) = -KDL::PI_2;

    KDL::Frame startFrame;
    error = robot.CalculateForwardKinematics(joints, startFrame);

    KDL::Frame endFrame;
    endFrame = startFrame;
    endFrame.p[0] += 100;

    KDL::Path *path;
    error = robot.CreatePath(PathType::Line, path, startFrame, endFrame);
    if (error.hasError())
    {
        logger.log(error.GetErrorMessage());
        return 0;
    }

    KDL::VelocityProfile *velocityProfile;
    error = robot.CreateVelocityProfile(VelocityProfileType::Trapezoidal, velocityProfile, 300, 300);
    if (error.hasError())
    {
        logger.log(error.GetErrorMessage());
        return 0;
    }
    logger.log("Velocity Profile Created!");

    KDL::Trajectory *trajectory;
    error = robot.CreateTrajectory(trajectory, path, velocityProfile, 0.01);
    if (error.hasError())
    {
        logger.log(error.GetErrorMessage());
        return 0;
    }
    logger.log("Trajectory Created!");
    KDL::JntArray destJoint(6);
    error = robot.CalculateInverseKinematics(joints, endFrame, destJoint);

    std::cout << "\n"
              << destJoint(0) * KDL::rad2deg << "\t" << destJoint(1) * KDL::rad2deg << "\t" << destJoint(2) * KDL::rad2deg << "\t" << destJoint(3) * KDL::rad2deg << "\t" << destJoint(4) * KDL::rad2deg << "\t" << destJoint(5) * KDL::rad2deg << "\n";
    if (error.hasError())
    {
        logger.log(error.GetErrorMessage());
        return 0;
    }

    return 0;
}
