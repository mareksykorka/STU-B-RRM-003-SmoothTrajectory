#include "zad3/ik_solver/ik_solver.h"

IKSolver::IKSolver() : loader_("robot_description")
{
    joint_model_group_ = loader_.getModel()->getJointModelGroup("robot");
    previousBestSolution_ = {0,0,0,0,0,0};
    newBestSolution_ = {0,0,0,0,0,0};
}

bool IKSolver::solve(Eigen::Affine3d target, std::vector<std::vector<double>>& solutions, kinematics::KinematicsResult& result)
{
    // Convert "target" position to ik_pose compatible "geometry_msgs::Pose"
    geometry_msgs::Pose target_msg;
    tf::poseEigenToMsg(target, target_msg);
    std::vector<geometry_msgs::Pose> ik_poses = {target_msg};

    // IK seed state set to zeros
    std::vector<double> ik_seed_state = {0,0,0,0,0,0};

    // Calculate IK solution
    joint_model_group_->getSolverInstance()->getPositionIK(ik_poses,ik_seed_state,solutions,result,kinematics::KinematicsQueryOptions());

    // Return IK solve state
    return result.kinematic_error == kinematics::KinematicError::OK;
}

std::vector<double> IKSolver::findBestSolution(std::vector<std::vector<double>>& solutions)
{
    double dist, last_dist = std::numeric_limits<double>::infinity();

    for (const auto &solution: solutions)
    {
        dist = 0;
        for (int i = 0; i < solution.size(); i++)
        {
            dist += pow((solution[i]-previousBestSolution_[i]),2); // Compute Euclidean dist d=(q1−p1)^2+(q2−p2)^2+…+(qn−pn)^2
        }
        dist = sqrt(dist); // Compute Euclidean dist d=sqrt(d)

        if(dist < last_dist) // Find solution with the smallest dist from the previous one.
        {
            last_dist = dist;
            newBestSolution_ = solution;
        }
    }
    previousBestSolution_ = newBestSolution_;
    return newBestSolution_;
}