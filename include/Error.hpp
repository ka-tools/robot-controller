#ifndef ERROR_HPP
#define ERROR_HPP

#pragma once
#include <string>

enum class ErrorCode
{
    NoError,
    FileNotFound,
    ParseError,
    RobotNotFound,
    InvalidFormat,
    RobotAlreadyExists,
    ForwardKinematicsError,
    InverseKinematicsError,
    VelocityProfileNotFound,
    MissingParameter,
    PathTypeNotFound,
    ForwardDynamicsError,
    InverseDynamicsError,
    Unknown
};

class Error
{
public:
    Error();

    void set(ErrorCode code, const std::string &message);
    void clear();

    bool hasError() const;
    ErrorCode code() const;
    const std::string &GetErrorMessage() const;

private:
    ErrorCode code_;
    std::string message_;
};
#endif // ERROR_HPP