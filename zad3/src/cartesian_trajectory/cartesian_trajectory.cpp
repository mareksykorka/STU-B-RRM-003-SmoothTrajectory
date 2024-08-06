// ROS Libraries
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// Custom Math
#include "custom_math/polynomial_tools.h"
#include "custom_math/inverse_matrix_solver.h"

// Helper libraries
#include "zad3/trajectory/trajectory_plan.h"
#include "logger/logger.h"
#include "zad3/ik_solver/ik_solver.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cartesian_trajectory_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/trajectory", 1);

    // Logger Initialization
    Logger logger("cartesian");
    logger.logHeader(std::vector<std::string> {"time","y","yd","ydd","yddd", "z","zd","zdd","zddd", "rz","rzd","rzdd","rzddd","q1","q2","q3","q4","q5","q6"});

    // Trajectory msg defeinition
    moveit_msgs::RobotTrajectory trajectory;
    trajectory.joint_trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    // Encapsulated IK solver class initialization
    IKSolver ikSolver = IKSolver();

    // Definition of trajectory plans
    MatrixSolver MatrixSix(6), MatrixFour(4), MatrixTwo(2);
    std::vector<TrajectoryPlan> x, y, z, rx, ry, rz;
    #pragma region Define trajectory plans
        #pragma region X trajectory plan
        x.push_back(TrajectoryPlan(std::vector<int> ({0,0,1,1}), std::vector<double> ({1,0,1,0}),&MatrixFour));
        x.push_back(TrajectoryPlan(std::vector<int> ({1,1,2,2}), std::vector<double> ({1,0,1,0}),&MatrixFour));
        x.push_back(TrajectoryPlan(std::vector<int> ({2,2,3,3}), std::vector<double> ({1,0,1,0}),&MatrixFour));
        x.push_back(TrajectoryPlan(std::vector<int> ({3,3,4,4}), std::vector<double> ({1,0,1,0}),&MatrixFour));
        x.push_back(TrajectoryPlan(std::vector<int> ({4,4,5,5}), std::vector<double> ({1,0,1,0}),&MatrixFour));
        x.push_back(TrajectoryPlan(std::vector<int> ({5,5,9,9}), std::vector<double> ({1,0,1,0}),&MatrixFour));
        #pragma endregion
        #pragma region Y trajectory plan
        y.push_back(TrajectoryPlan(std::vector<int> ({0,0,1,1}), std::vector<double> ({0,0,0,0}),&MatrixFour));
        y.push_back(TrajectoryPlan(std::vector<int> ({1,1,2,2}), std::vector<double> ({0,0,0,0}),&MatrixFour));
        y.push_back(TrajectoryPlan(std::vector<int> ({2,2,3,3}), std::vector<double> ({0,0,0,0}),&MatrixFour));
        MatrixSolver MatrixFiveY(std::vector<int> ({3, 3, 4, 5, 5}));
        y.push_back(TrajectoryPlan(std::vector<int> ({3,3,4,5,5}), std::vector<double> ({0,0,0.5,0.5,0}),&MatrixFiveY));
        y.push_back(TrajectoryPlan(std::vector<int> ({5,5,9,9}), std::vector<double> ({0.5,0,0,0}),&MatrixFour));
        #pragma endregion
        #pragma region Z trajectory plan
        z.push_back(TrajectoryPlan(std::vector<int> ({0,0,1,1}), std::vector<double> ({1.6,0,1,0}),&MatrixFour));
        z.push_back(TrajectoryPlan(std::vector<int> ({1,1,2,2}), std::vector<double> ({1,0,1,0}),&MatrixFour));
        z.push_back(TrajectoryPlan(std::vector<int> ({2,2,3,3}), std::vector<double> ({1,0,1,0}),&MatrixFour));
        MatrixSolver MatrixFiveZ(std::vector<int> ({3, 3, 4, 5, 5}));
        z.push_back(TrajectoryPlan(std::vector<int> ({3,3,4,5,5}), std::vector<double> ({1,0,1.0,1.6,0}),&MatrixFiveZ));
        z.push_back(TrajectoryPlan(std::vector<int> ({5,5,9,9}), std::vector<double> ({1.6,0,1.6,0}),&MatrixFour));
        #pragma endregion
        #pragma region rX trajectory plan
        rx.push_back(TrajectoryPlan(std::vector<int> ({0,1}), std::vector<double> ({0,0}),&MatrixTwo));
        rx.push_back(TrajectoryPlan(std::vector<int> ({1,2}), std::vector<double> ({0,0}),&MatrixTwo));
        rx.push_back(TrajectoryPlan(std::vector<int> ({2,3}), std::vector<double> ({0,0}),&MatrixTwo));
        rx.push_back(TrajectoryPlan(std::vector<int> ({3,4}), std::vector<double> ({0,0}),&MatrixTwo));
        rx.push_back(TrajectoryPlan(std::vector<int> ({4,5}), std::vector<double> ({0,0}),&MatrixTwo));
        rx.push_back(TrajectoryPlan(std::vector<int> ({5,9}), std::vector<double> ({0,0}),&MatrixTwo));
        #pragma endregion
        #pragma region rY trajectory plan
        ry.push_back(TrajectoryPlan(std::vector<int> ({0,1}), std::vector<double> ({M_PI/2,M_PI/2}),&MatrixTwo));
        ry.push_back(TrajectoryPlan(std::vector<int> ({1,2}), std::vector<double> ({M_PI/2,M_PI/2}),&MatrixTwo));
        ry.push_back(TrajectoryPlan(std::vector<int> ({2,3}), std::vector<double> ({M_PI/2,M_PI/2}),&MatrixTwo));
        ry.push_back(TrajectoryPlan(std::vector<int> ({3,4}), std::vector<double> ({M_PI/2,M_PI/2}),&MatrixTwo));
        ry.push_back(TrajectoryPlan(std::vector<int> ({4,5}), std::vector<double> ({M_PI/2,M_PI/2}),&MatrixTwo));
        ry.push_back(TrajectoryPlan(std::vector<int> ({5,9}), std::vector<double> ({M_PI/2,M_PI/2}),&MatrixTwo));
        #pragma endregion
        #pragma region rZ trajectory plan
        rz.push_back(TrajectoryPlan(std::vector<int> ({0,0,0,1,1,1}), std::vector<double> ({0,0,0,0,0,0}),&MatrixSix));
        rz.push_back(TrajectoryPlan(std::vector<int> ({1,1,1,2,2,2}), std::vector<double> ({0,0,0,M_PI/2,0,0}),&MatrixSix));
        rz.push_back(TrajectoryPlan(std::vector<int> ({2,2,2,3,3,3}), std::vector<double> ({M_PI/2,0,0,M_PI/2,0,0}),&MatrixSix));
        rz.push_back(TrajectoryPlan(std::vector<int> ({3,3,3,4,4,4}), std::vector<double> ({M_PI/2,0,0,M_PI/2,0,0}),&MatrixSix));
        rz.push_back(TrajectoryPlan(std::vector<int> ({4,4,4,5,5,5}), std::vector<double> ({M_PI/2,0,0,0,0,0}),&MatrixSix));
        rz.push_back(TrajectoryPlan(std::vector<int> ({5,5,5,9,9,9}), std::vector<double> ({0,0,0,0,0,0}),&MatrixSix));
        #pragma endregion
    #pragma endregion

    // Solving all planned trajectories for their coefficients and creation of corresponding polynomials
    // Even if coordinatePlans are diffrent sizes
    std::vector<std::vector<TrajectoryPlan>*> trajectoryPlan = {&x, &y, &z, &rx, &ry, &rz};
    for (auto coordinatePlan : trajectoryPlan)
    {
        for (int i = 0; i < coordinatePlan->size(); i++)
        {
            (*coordinatePlan)[i].solve();
            (*coordinatePlan)[i].createPolynomials(3);
        }
    }

    for (double t = 0; t <= 9; t += 0.05)
    {
        TrajectoryPlan *xcalc, *ycalc, *zcalc, *rxcalc, *rycalc, *rzcalc;
        #pragma region Pick TrajectoryPlan in specified time range
        for (int i = 0; i < x.size(); i++)
            if(x[i].isInTime(t))
                xcalc = &x[i];
        for (int i = 0; i < y.size(); i++)
            if(y[i].isInTime(t))
                ycalc = &y[i];
        for (int i = 0; i < z.size(); i++)
            if(z[i].isInTime(t))
                zcalc = &z[i];
        for (int i = 0; i < rx.size(); i++)
            if(rx[i].isInTime(t))
                rxcalc = &rx[i];
        for (int i = 0; i < ry.size(); i++)
            if(ry[i].isInTime(t))
                rycalc = &ry[i];
        for (int i = 0; i < rz.size(); i++)
            if(rz[i].isInTime(t))
                rzcalc = &rz[i];
        #pragma endregion

        Eigen::Affine3d target = Eigen::Translation3d(Eigen::Vector3d(xcalc->polynomials_[0].calc(t), ycalc->polynomials_[0].calc(t), zcalc->polynomials_[0].calc(t)))*
                                 Eigen::AngleAxisd(rxcalc->polynomials_[0].calc(t),Eigen::Vector3d::UnitX())*
                                 Eigen::AngleAxisd(rycalc->polynomials_[0].calc(t),Eigen::Vector3d::UnitY())*
                                 Eigen::AngleAxisd(rzcalc->polynomials_[0].calc(t),Eigen::Vector3d::UnitZ());

        std::vector<std::vector<double>> solutions;
        kinematics::KinematicsResult result;
        if(ikSolver.solve(target, solutions, result)) // If there is a solution for IK
        {
            std::vector<double> final_solution = ikSolver.findBestSolution(solutions); // Then pick the best solution

            trajectory_msgs::JointTrajectoryPoint point; // Fill out the trajectory_msgs
            point.positions.resize(6);
            point.velocities.resize(6);
            point.accelerations.resize(6);

            for (int i = 0; i < 6; i++)
            {
                point.positions[i] = final_solution[i];
                point.velocities[i] = 0;
                point.accelerations[i] = 0;
            }
            point.time_from_start = ros::Duration(t);
            trajectory.joint_trajectory.points.push_back(point);

            #pragma region Log data in to csv
            std::vector<std::string> data;
            data.push_back(std::to_string(ycalc->polynomials_[0].calc(t)));
            data.push_back(std::to_string(ycalc->polynomials_[1].calc(t)));
            data.push_back(std::to_string(ycalc->polynomials_[2].calc(t)));
            data.push_back(std::to_string(ycalc->polynomials_[3].calc(t)));
            data.push_back(std::to_string(zcalc->polynomials_[0].calc(t)));
            data.push_back(std::to_string(zcalc->polynomials_[1].calc(t)));
            data.push_back(std::to_string(zcalc->polynomials_[2].calc(t)));
            data.push_back(std::to_string(zcalc->polynomials_[3].calc(t)));
            data.push_back(std::to_string(rzcalc->polynomials_[0].calc(t)));
            data.push_back(std::to_string(rzcalc->polynomials_[1].calc(t)));
            data.push_back(std::to_string(rzcalc->polynomials_[2].calc(t)));
            data.push_back(std::to_string(rzcalc->polynomials_[3].calc(t)));
            data.push_back(std::to_string(point.positions[0]));
            data.push_back(std::to_string(point.positions[1]));
            data.push_back(std::to_string(point.positions[2]));
            data.push_back(std::to_string(point.positions[3]));
            data.push_back(std::to_string(point.positions[4]));
            data.push_back(std::to_string(point.positions[5]));
            logger.logData(t,data);
            #pragma endregion
        }
    }

    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.model_id = "abb_irb";
    display_trajectory.trajectory.push_back(trajectory);
    for (int i = 0; i < 6; i++) {
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