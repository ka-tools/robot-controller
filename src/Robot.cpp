#include "Robot.hpp"

Robot::Robot()
{
    std::cout << "\nRobot Created. Please load YAML file to initialize robot. (robot.LoadRobotFromYAML(string file name, string robot name)) \n";
}

Robot::~Robot()
{
    if (chain.getNrOfSegments() > 0)
    {
        for (int segmentCount = 0; segmentCount < chain.getNrOfSegments(); segmentCount++)
        {
            chain.ClearSegment(segmentCount);
        }
    }
}

int Robot::GetNumberOfSegments()
{
    return chain.getNrOfSegments();
}

int Robot::GetNumberOfJoints()
{
    return chain.getNrOfJoints();
}

std::string Robot::StringToLower(const std::string &str)
{
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(),
                   [](unsigned char c)
                   { return std::tolower(c); });
    return result;
}

KDL::Joint::JointType Robot::JointTypeFromString(const std::string &jointTypeString, const std::string &axisRevolutionTypeString)
{
    std::string jt = Robot::StringToLower(jointTypeString);
    std::string axis = Robot::StringToLower(axisRevolutionTypeString);

    if (jt == "rotational")
    {
        if (axis == "x")
            return KDL::Joint::RotX;
        else if (axis == "y")
            return KDL::Joint::RotY;
        else if (axis == "z")
            return KDL::Joint::RotZ;
        else if (axis == "axis")
            return KDL::Joint::RotAxis;
    }
    else if (jt == "translational")
    {
        if (axis == "x")
            return KDL::Joint::TransX;
        else if (axis == "y")
            return KDL::Joint::TransY;
        else if (axis == "z")
            return KDL::Joint::TransZ;
        else if (axis == "axis")
            return KDL::Joint::TransAxis;
    }

    return KDL::Joint::None;
}

Error Robot::LoadRobotFromYAML(const std::string &filename, const std::string &robot_name)
{
    Error error;
    if (GetNumberOfSegments() > 0)
    {
        error.set(ErrorCode::RobotAlreadyExists, "Robot has" + std::to_string(GetNumberOfJoints()) + "joints already");
        return error;
    }
    else
    {
        YAML::Node root;
        try
        {
            root = YAML::LoadFile(filename);
        }
        catch (const YAML::Exception &e)
        {
            error.set(ErrorCode::FileNotFound, "Can't open YAML: " + std::string(e.what()));
            return error;
        }

        YAML::Node robotNode;
        bool robotFound = false;
        for (const auto &robot : root["robots"])
        {
            if (robot["robot_name"].as<std::string>() == robot_name)
            {
                KDL::Vector gravity = KDL::Vector(robot["gravity"]["x"].as<double>(), robot["gravity"]["y"].as<double>(), robot["gravity"]["z"].as<double>());
                int jointCount = robot["joint_count"].as<int>();
                std::cout << "Robot Axis Count:" << jointCount << std::endl;
                std::cout << "Gravity:" << gravity << std::endl;

                minimumJointsLimit = KDL::JntArray(jointCount);
                maximumJointsLimit = KDL::JntArray(jointCount);
                int jointCounter = 0;
                for (const auto &jointNode : robot["joints"])
                {
                    std::string jointName = jointNode["name"].as<std::string>();
                    std::string type = jointNode["type"].as<std::string>();
                    std::string axisRevolutionType = jointNode["axis"].as<std::string>();
                    double minimumLimit = jointNode["limits"]["min"].as<double>();
                    double maximumLimit = jointNode["limits"]["max"].as<double>();
                    double a = jointNode["dh"]["a"].as<double>();
                    double alpha = jointNode["dh"]["alpha"].as<double>();
                    double d = jointNode["dh"]["d"].as<double>();
                    double theta = jointNode["dh"]["theta"].as<double>();

                    double scale = jointNode["scale"].as<double>();
                    double offset = jointNode["offset"].as<double>();
                    double inertia = jointNode["inertia"].as<double>();
                    double damping = jointNode["damping"].as<double>();
                    double stiffness = jointNode["stiffness"].as<double>();

                    std::cout << "Joint Information:" << std::endl;
                    std::cout << "-------------------" << std::endl;
                    std::cout << "Joint Name: " << jointName << std::endl;
                    std::cout << "Type: " << type << std::endl;
                    std::cout << "Axis Revolution Type: " << axisRevolutionType << std::endl;
                    std::cout << "Minimum Limit: " << minimumLimit << std::endl;
                    std::cout << "Maximum Limit: " << maximumLimit << std::endl;

                    std::cout << "\nDH Parameters:" << std::endl;
                    std::cout << "-------------------" << std::endl;
                    std::cout << "a (link length): " << a << std::endl;
                    std::cout << "alpha (link twist): " << alpha << std::endl;
                    std::cout << "d (link offset): " << d << std::endl;
                    std::cout << "theta (joint angle): " << theta << std::endl;

                    std::cout << "Scale:" << scale << std::endl;
                    std::cout << "Offset:" << offset << std::endl;
                    std::cout << "Inertia:" << inertia << std::endl;
                    std::cout << "Damping:" << damping << std::endl;
                    std::cout << "Stiffness:" << stiffness << std::endl;

                    minimumJointsLimit(jointCounter) = minimumLimit * KDL::deg2rad;
                    maximumJointsLimit(jointCounter) = maximumLimit * KDL::deg2rad;
                    jointCounter++;

                    chain.addSegment(KDL::Segment(KDL::Joint(jointName, JointTypeFromString(type, axisRevolutionType), scale, offset, inertia, damping, stiffness), KDL::Frame::DH(a, alpha, d, theta)));
                }

                currentJointStates.currentJointPositions = KDL::JntArray(jointCount);
                currentJointStates.currentJointVelocities = KDL::JntArray(jointCount);
                currentJointStates.currentJointAccelerations = KDL::JntArray(jointCount);

                robotFound = true;
                error = InitializeSolvers();

                if (error.hasError())
                {
                    return error;
                }
                else
                {
                    robotState = RobotState::IDLE;
                }
                break;
            }
        }
        if (!robotFound)
        {
            Error error;
            error.set(ErrorCode::RobotNotFound, "Can't find Robot");
            return error;
        }

        return Error();
    }
}

Error Robot::InitializeSolvers()
{
    forwardKinematicsSolverPositionRecursive = new KDL::ChainFkSolverPos_recursive(chain);
    inverseKinematicsSolverVelocity = new KDL::ChainIkSolverVel_pinv(chain);
    inverseKinematicsSolverPosition = new KDL::ChainIkSolverPos_NR(chain, *forwardKinematicsSolverPositionRecursive, *inverseKinematicsSolverVelocity, 100, 1e-6);
    inverseKinematicsSolverPositionWithJointLimits = new KDL::ChainIkSolverPos_NR_JL(chain, minimumJointsLimit, maximumJointsLimit, *forwardKinematicsSolverPositionRecursive, *inverseKinematicsSolverVelocity, 100, 1e-6);
    forwardDynamicSolver = new KDL::ChainFdSolver_RNE(chain, gravity);
    inverseDynamicSolver = new KDL::ChainIdSolver_RNE(chain, gravity);
    return Error();
}

Error Robot::SetGravity(const double x, const double y, const double z)
{
    Error error;
    gravity = KDL::Vector(x, y, z);

    return error;
}

std::string Robot::CheckSolverError(const int solverResult)
{
    std::string errorMessage;
    switch (solverResult)
    {
    case KDL::ChainFkSolverPos::E_DEGRADED:
        errorMessage = "Converged but degraded solution (e.g. WDLS with psuedo-inverse singular)";
        break;
    case KDL::ChainFkSolverPos::E_NOERROR:
        errorMessage = "Success";
        break;
    case KDL::ChainFkSolverPos::E_NO_CONVERGE:
        errorMessage = "Failed to converge";
        break;
    case KDL::ChainFkSolverPos::E_UNDEFINED:
        errorMessage = "Undefined value (e.g. computed a NAN, or tan(90 degrees))";
        break;
    case KDL::ChainFkSolverPos::E_NOT_UP_TO_DATE:
        errorMessage = "Chain size changed";
        break;
    case KDL::ChainFkSolverPos::E_SIZE_MISMATCH:
        errorMessage = "Input size does not match internal state";
        break;
    case KDL::ChainFkSolverPos::E_MAX_ITERATIONS_EXCEEDED:
        errorMessage = "Maximum number of iterations exceeded";
        break;
    case KDL::ChainFkSolverPos::E_OUT_OF_RANGE:
        errorMessage = "Requested index out of range";
        break;
    case KDL::ChainFkSolverPos::E_NOT_IMPLEMENTED:
        errorMessage = "Not yet implemented";
        break;
    case KDL::ChainFkSolverPos::E_SVD_FAILED:
        errorMessage = "Internal svd calculation failed";
        break;
    default:
        break;
    }
    return errorMessage;
}

Error Robot::CalculateForwardKinematics(const KDL::JntArray &jointAngles, KDL::Frame &currentCartesianFrame)
{
    Error error;
    int forwardKinematicsResult = forwardKinematicsSolverPositionRecursive->JntToCart(jointAngles, currentCartesianFrame);
    std::string errorMessage = CheckSolverError(forwardKinematicsResult);
    if (errorMessage == "Success")
    {
        error.set(ErrorCode::NoError, "Forward Kinematics Calculated!");
    }
    else
    {
        error.set(ErrorCode::ForwardKinematicsError, errorMessage);
    }

    return error;
}

Error Robot::CalculateInverseKinematics(const KDL::JntArray &currentJointAngles, const KDL::Frame &destinationFrame, KDL::JntArray &destinationJointAngles)
{
    Error error;
    int inverseKinematicsResult = inverseKinematicsSolverPositionWithJointLimits->CartToJnt(currentJointAngles, destinationFrame, destinationJointAngles);
    std::string errorMessage = CheckSolverError(inverseKinematicsResult);
    if (errorMessage == "Success")
    {
        error.set(ErrorCode::NoError, "Inverse Kinematics Calculated!");
    }
    else
    {
        error.set(ErrorCode::InverseKinematicsError, errorMessage);
    }

    return error;
}

Error Robot::CreatePath(const PathType pathType, KDL::Path *&path, const KDL::Frame startFrame, const KDL::Frame endFrame, const KDL::Vector circleCenterVector, const KDL::Vector circlePointVector, const KDL::Rotation circleRotation, double circleAngle, double radius)
{
    Error error;
    try
    {
        switch (pathType)
        {
        case PathType::Point:
            std::cout << "\n"
                      << "Point Path Selected";
            if (startFrame == KDL::Frame::Identity())
            {
                error.set(ErrorCode::MissingParameter, "Needs start frame");
                return error;
            }
            path = new KDL::Path_Point(startFrame);
            break;

        case PathType::Line:
            std::cout << "\n"
                      << "Line Path Selected";
            if (startFrame == KDL::Frame::Identity() || endFrame == KDL::Frame::Identity())
            {
                error.set(ErrorCode::MissingParameter, "Needs start and end frame");
                return error;
            }
            path = new KDL::Path_Line(startFrame, endFrame, new KDL::RotationalInterpolation_SingleAxis(), 0.01);
            break;
        case PathType::Circle:
            std::cout << "\n"
                      << "Circle Path Selected";
            if (startFrame == KDL::Frame::Identity() || circleCenterVector == KDL::Vector::Zero() || circlePointVector == KDL::Vector::Zero() || circleRotation == KDL::Rotation::Identity() || circleAngle == -1)
            {
                error.set(ErrorCode::MissingParameter, "Needs start frame, circle center vector, circle point vector, circle rotation and circle angle");
                return error;
            }
            path = new KDL::Path_Circle(startFrame, circleCenterVector, circlePointVector, circleRotation, circleAngle, new KDL::RotationalInterpolation_SingleAxis(), 0.01);
            break;
        case PathType::RoundedComposite:
            std::cout << "\n"
                      << "RoundedComposite Path Selected";
            if (radius == -1)
            {
                error.set(ErrorCode::MissingParameter, "Needs radius");
                return error;
            }
            path = new KDL::Path_RoundedComposite(radius, 0.01, new KDL::RotationalInterpolation_SingleAxis());
            break;
        case PathType::Composite:
            std::cout << "\n"
                      << "Composite Path Selected";
            path = new KDL::Path_Composite();
            break;
        default:
            error.set(ErrorCode::PathTypeNotFound, "Path type doesn't exists");
            break;
        }
    }
    catch (const std::exception &e)
    {
        error.set(ErrorCode::Unknown, e.what());
        return error;
    }
    return error;
}

Error Robot::CreateVelocityProfile(const VelocityProfileType velocityProfileType, KDL::VelocityProfile *&velocityProfile, const double maximumVelocity, const double maximumAcceleration)
{
    Error error;
    try
    {
        switch (velocityProfileType)
        {
        case VelocityProfileType::Dirac:
            std::cout << "\n"
                      << "Dirac Selected";
            velocityProfile = new KDL::VelocityProfile_Dirac();
            break;
        case VelocityProfileType::Rectangular:
            std::cout << "\n"
                      << "Rectangular Profile Selected Maximum Velocity:" << maximumVelocity;
            velocityProfile = new KDL::VelocityProfile_Rectangular(maximumVelocity);
            break;
        case VelocityProfileType::Spline:
            std::cout << "\n"
                      << "Spline Profile Selected";
            velocityProfile = new KDL::VelocityProfile_Spline();
            break;
        case VelocityProfileType::Trapezoidal:
            std::cout << "\n"
                      << "Trapezoidal Profile Selected Maximum Velocity:" << maximumVelocity << " Maximum Acceleration:" << maximumAcceleration;
            velocityProfile = new KDL::VelocityProfile_Trap(maximumVelocity, maximumAcceleration);
            break;
        case VelocityProfileType::TrapezoidalHalf:
            std::cout << "\n"
                      << "TrapezoidalHalf Profile Selected Maximum Velocity:" << maximumVelocity << " Maximum Acceleration:" << maximumAcceleration;
            velocityProfile = new KDL::VelocityProfile_TrapHalf(maximumVelocity, maximumAcceleration);
            break;

        default:
            error.set(ErrorCode::VelocityProfileNotFound, "Velocity Profile Type doesn't exists");
            break;
        }
    }
    catch (const std::exception &e)
    {
        error.set(ErrorCode::Unknown, e.what());
        return error;
    }

    return error;
}

Error Robot::CreateTrajectory(KDL::Trajectory *trajectory, KDL::Path *path, KDL::VelocityProfile *velocityProfile, const double sampleTime)
{
    Error error;
    try
    {
        std::cout << "\nTrajectory started\n";
        double pathLength = path->PathLength();
        velocityProfile->SetProfile(0, pathLength);
        trajectory = new KDL::Trajectory_Segment(path, velocityProfile);
        for (double currentTime = 0.0; currentTime < trajectory->Duration(); currentTime += sampleTime)
        {
            KDL::Frame current_pose;
            current_pose = trajectory->Pos(currentTime);
            std::cout << current_pose.p[0] << "\t";
        }
    }
    catch (const std::exception &e)
    {
        error.set(ErrorCode::Unknown, e.what());
        return error;
    }

    return error;
}

Error Robot::CalculateForwardDynamics(const KDL::JntArray &currentJointAngles, const KDL::JntArray &currentJointVelocities, const KDL::JntArray &currentJointTorques, const KDL::Wrenches &externalForces, KDL::JntArray jointAccelerations)
{
    Error error;
    int forwardDynamicsResult = forwardDynamicSolver->CartToJnt(currentJointAngles, currentJointVelocities, currentJointTorques, externalForces, jointAccelerations);
    std::string errorMessage = CheckSolverError(forwardDynamicsResult);

    if (errorMessage == "Success")
    {
        error.set(ErrorCode::NoError, "Forward Dynamics Calculated!");
    }
    else
    {
        error.set(ErrorCode::ForwardDynamicsError, errorMessage);
    }

    return error;
}

RobotState Robot::GetRobotState()
{
    return robotState;
}