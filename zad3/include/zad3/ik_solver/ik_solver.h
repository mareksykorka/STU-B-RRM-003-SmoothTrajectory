#ifndef CATKIN_TRETIE_ZADANIE_IK_SOLVER_H
#define CATKIN_TRETIE_ZADANIE_IK_SOLVER_H

// ROS Libraries
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

class IKSolver
{
    public:
        IKSolver();
        bool solve(Eigen::Affine3d target, std::vector<std::vector<double>>& solutions, kinematics::KinematicsResult& result);
        std::vector<double> findBestSolution(std::vector<std::vector<double>>& solutions);
    private:
        robot_model_loader::RobotModelLoader loader_;
        robot_state::JointModelGroup* joint_model_group_;

        std::vector<double> previousBestSolution_;
        std::vector<double> newBestSolution_;
};

#endif //CATKIN_TRETIE_ZADANIE_IK_SOLVER_H
