// ROS Libraries
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <moveit_msgs/DisplayTrajectory.h>

// Custom Math
#include "custom_math/polynomial_tools.h"
#include "custom_math/inverse_matrix_solver.h"

// Helper libraries
#include "zad3/trajectory/trajectory_plan.h"
#include "zad3/custom_math/math_ext.h"
#include "logger/logger.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_trajectory_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/trajectory", 1);

    // Logger Initialization
    Logger logger("jointState");
    logger.logHeader(std::vector<std::string> {"time","q1","q1d","q1dd","q1ddd","q3","q3d","q3dd","q3ddd"});

    // Trajectory msg defeinition
    moveit_msgs::RobotTrajectory trajectory;
    trajectory.joint_trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    #pragma region Define trajectory plan
    MatrixSolver MatrixSix(6);
    std::vector<TrajectoryPlan> q3;

    TrajectoryPlan q1(std::vector<int> ({0,0,0,4,4,4}), std::vector<double> ({0,0,0,degToRad(90), 0, 0}),&MatrixSix);
    q3.push_back(TrajectoryPlan(std::vector<int> ({0,0,0,1,1,1}), std::vector<double> ({0,0,0,degToRad(30),0,0}),&MatrixSix));
    q3.push_back(TrajectoryPlan(std::vector<int> ({1,1,1,4,4,4}), std::vector<double> ({degToRad(30),0,0,0,0,0}),&MatrixSix));
    #pragma endregion

    #pragma region Calculate smooth trajectory polynomials
    q1.solve();
    q1.createPolynomials(4);

    q3[0].solve();
    q3[0].createPolynomials(4);

    q3[1].solve();
    q3[1].createPolynomials(4);
    #pragma endregion

    for (double t = 0.0; t <= 4.0; t += 0.05)
    {
        TrajectoryPlan *q3calc;
        for (int i = 0; i < q3.size(); ++i)
            if(q3[i].isInTime(t))
                q3calc = &q3[i];

        #pragma region Fill in TrajectoryMsg
        // Vytvorenie prejazdoveho bodu
        trajectory_msgs::JointTrajectoryPoint point;

        // Robot ma 6 klbov
        point.positions.resize(6);
        point.velocities.resize(6);
        point.accelerations.resize(6);

        // Klb 1
        point.positions[0] = q1.polynomials_[0].calc(t);
        point.velocities[0] = q1.polynomials_[1].calc(t);
        point.accelerations[0] = q1.polynomials_[2].calc(t);

        // Klb 2
        point.positions[1] = 0;
        point.velocities[1] = 0;
        point.accelerations[1] = 0;

        // Klb 3
        point.positions[2] = q3calc->polynomials_[0].calc(t);
        point.velocities[2] = q3calc->polynomials_[1].calc(t);
        point.accelerations[2] = q3calc->polynomials_[2].calc(t);

        // Klb 4
        point.positions[3] = 0;
        point.velocities[3] = 0;
        point.accelerations[3] = 0;

        // Klb 5
        point.positions[4] = 0;
        point.velocities[4] = 0;
        point.accelerations[4] = 0;

        // Klb 6
        point.positions[5] = 0;
        point.velocities[5] = 0;
        point.accelerations[5] = 0;

        // Vlozenie casu prejazdu
        point.time_from_start = ros::Duration(t);

        // VLozenie bodu do trajektorie
        trajectory.joint_trajectory.points.push_back(point);
        #pragma endregion

        #pragma region Log data in to csv
        std::vector<std::string> data;
        data.push_back(std::to_string(point.positions[0]));
        data.push_back(std::to_string(point.velocities[0]));
        data.push_back(std::to_string(point.accelerations[0]));
        data.push_back(std::to_string(q1.polynomials_[3].calc(t)));
        data.push_back(std::to_string(point.positions[2]));
        data.push_back(std::to_string(point.velocities[2]));
        data.push_back(std::to_string(point.accelerations[2]));
        data.push_back(std::to_string(q3calc->polynomials_[3].calc(t)));
        logger.logData(t,data);
        #pragma endregion
    }

    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.model_id = "abb_irb";
    display_trajectory.trajectory.push_back(trajectory);
    for (int i = 0; i < 6; ++i) {
        display_trajectory.trajectory_start.joint_state.name.push_back("joint_" + std::to_string(i + 1) );
        display_trajectory.trajectory_start.joint_state.header.stamp = ros::Time();
        display_trajectory.trajectory_start.joint_state.position.push_back(0);
        display_trajectory.trajectory_start.joint_state.velocity.push_back(0);
        display_trajectory.trajectory_start.joint_state.effort.push_back(0);
    }

    ros::Rate loop_rate(0.5);
    while (ros::ok())
    {
        publisher.publish(display_trajectory);
        loop_rate.sleep();
    }
    return 0;
}