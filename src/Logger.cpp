// Logger.cpp
#include "Logger.hpp"

Logger::Logger(const std::string &filename)
{
    logFile.open(filename, std::ios::app); // dosyaya ekler
}

Logger::~Logger()
{
    if (logFile.is_open())
        logFile.close();
}

void Logger::log(const std::string &message, Level level)
{
    std::string formatted = formatMessage(message, level);

    // Konsola yaz
    std::cout << "\n"
              << formatted << std::endl;

    // Dosyaya yaz
    if (logFile.is_open())
        logFile << formatted << std::endl;
}

std::string Logger::formatMessage(const std::string &message, Level level)
{
    std::string levelStr;
    switch (level)
    {
    case Level::Info:
        levelStr = "[INFO]";
        break;
    case Level::Warning:
        levelStr = "[WARNING]";
        break;
    case Level::Error:
        levelStr = "[ERROR]";
        break;
    }

    // Saat bilgisi ekle
    std::time_t now = std::time(nullptr);
    char buf[20];
    std::strftime(buf, sizeof(buf), "[%H:%M:%S]", std::localtime(&now));

    return std::string(buf) + " " + levelStr + " " + message;
}

void Logger::saveToFile()
{
}
