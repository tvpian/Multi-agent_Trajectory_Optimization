#pragma once

#include <algorithm>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <safe_corridor.hpp>
#include <init_traj_planner.hpp>
#include <mission.hpp>
#include <param.hpp>

#include "opt_const.h"
#include "time.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <eigen3/Eigen/Core>

using CppAD::AD;

size_t x_start;
size_t y_start;
size_t theta_start;
size_t v_start;
size_t omega_start;

std::vector<ros::Publisher> way_point_pub;

class MPCPlanner
{
public:
    std_msgs::Float64MultiArray msgs_traj_info;
    std::vector<std_msgs::Float64MultiArray> msgs_traj_coef;

    MPCPlanner(std::shared_ptr<SafeCorridor> _corridor_obj,
               std::shared_ptr<InitTrajPlanner> _initTrajPlanner_obj,
               TrajPlanning::Mission _mission,
               TrajPlanning::Param _param)
        : corridor_obj(std::move(_corridor_obj)),
          initTrajPlanner_obj(std::move(_initTrajPlanner_obj)),
          mission(std::move(_mission)),
          param(std::move(_param))

    {
        M = initTrajPlanner_obj.get()->T.size() - 1; // the number of segments

        outdim = 3; // the number of outputs (x,y,z)

        T = initTrajPlanner_obj.get()->T;
        initTraj = initTrajPlanner_obj.get()->initTraj;
        SFC = corridor_obj.get()->SFC;
    }

    virtual bool update(bool log) = 0;

    std::shared_ptr<SafeCorridor> corridor_obj;
    std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
    TrajPlanning::Mission mission;
    TrajPlanning::Param param;

    initTraj_t initTraj;
    std::vector<double> T;
    SFC_t SFC;

    int M, phi, outdim;
    std::vector<Eigen::MatrixXd> coef;

    void createMsg()
    {
        std::vector<double> traj_info;
        traj_info.emplace_back(N);
        traj_info.emplace_back(qn);
        traj_info.insert(traj_info.end(), T.begin(), T.end());
        msgs_traj_info.data = traj_info;

        msgs_traj_coef.resize(N);
        for (int qi = 0; qi < N; qi++)
        {
            std_msgs::MultiArrayDimension rows;
            rows.size = M * (qn + 1);
            msgs_traj_coef[qi].layout.dim.emplace_back(rows);

            std_msgs::MultiArrayDimension cols;
            cols.size = outdim;
            msgs_traj_coef[qi].layout.dim.emplace_back(cols);

            std::vector<double> coef_temp(coef[qi].data(), coef[qi].data() + coef[qi].size());
            msgs_traj_coef[qi].data.insert(msgs_traj_coef[qi].data.end(), coef_temp.begin(), coef_temp.end());
        }
    }
};