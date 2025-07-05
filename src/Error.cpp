#include "Error.hpp"

Error::Error() : code_(ErrorCode::NoError), message_("No Error") {}

void Error::set(ErrorCode code, const std::string &message)
{
    code_ = code;
    message_ = message;
}

void Error::clear()
{
    code_ = ErrorCode::NoError;
    message_.clear();
}

bool Error::hasError() const
{
    return code_ != ErrorCode::NoError;
}

ErrorCode Error::code() const
{
    return code_;
}

const std::string &Error::GetErrorMessage() const
{
    return message_;
}
