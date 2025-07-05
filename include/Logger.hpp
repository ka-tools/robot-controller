#ifndef LOGGER_HPP
#define LOGGER_HPP

#pragma once
#include <string>
#include <fstream>
#include <iostream>
#include <ctime>

class Logger
{
public:
    enum class Level
    {
        Info,
        Warning,
        Error
    };

    Logger(const std::string &filename = "log.txt");
    ~Logger();

    void log(const std::string &message, Level level = Level::Info);
    void saveToFile();

private:
    std::ofstream logFile;
    std::string formatMessage(const std::string &message, Level level);
};

#endif // LOGGER_HPP