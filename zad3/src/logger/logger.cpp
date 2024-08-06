#include "logger/logger.h"

Logger::Logger(std::string fileNameBase)
{
    const char* homeDir = std::getenv("HOME"); // Get the home directory

    if (homeDir == nullptr)
    {
        std::cerr << "Error: HOME environment variable not set." << std::endl;
        return;
    }

    std::time_t t = std::time(nullptr);
    std::tm* now = std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(now, "%Y-%m-%d_%H-%M-%S");
    std::string dateSuffix = oss.str();

    std::string filePath = std::string(homeDir) + "/Documents/log_" + fileNameBase + "_" + dateSuffix + ".csv";

    outputFile_.open(filePath);

    if (!outputFile_.is_open())
        std::cerr << "Error opening file!" << std::endl;
    else
        std::cout << "File [" << filePath << "] opened!" << std::endl;
}

Logger::~Logger()
{
    if (outputFile_.is_open())
        outputFile_.close();
}

void Logger::logHeader(std::vector<std::string> data)
{
    for (int i = 0; i < data.size(); ++i)
        outputFile_ << data[i] << ", ";
    outputFile_ << std::endl;
}

void Logger::logData(double time, std::vector<std::string> data)
{
    outputFile_ << time;
    for (int i = 0; i < data.size(); ++i)
        outputFile_ << ", " << data[i];
    outputFile_ << std::endl;
}