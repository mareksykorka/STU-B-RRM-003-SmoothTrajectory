#ifndef CATKIN_TRETIE_ZADANIE_LOGGER_H
#define CATKIN_TRETIE_ZADANIE_LOGGER_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ctime>

class Logger
{
    public:
        Logger() = delete;
        Logger(std::string fileNameBase);
        ~Logger();
        void logHeader(std::vector<std::string> data);
        void logData(double time, std::vector<std::string> data);
    private:
        std::ofstream outputFile_;
};

#endif //CATKIN_TRETIE_ZADANIE_LOGGER_H
