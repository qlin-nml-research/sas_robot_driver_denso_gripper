﻿#pragma once
/*
# Copyright (c) 2016-2020 Murilo Marques Marinho
#
#    This file is part of sas_denso_communcation.
#
#    sas_denso_communcation is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_denso_communcation is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_denso_communcation.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#   Contributors: Hung-Ching Lin, email:qlin1806@g.ecc.u-tokyo.ac.jp
#       -- added robot hand for cobotta robot
# ################################################################*/

#include <exception>
#include <tuple>
#include <atomic>
#include <vector>
#include <memory>

#include <dqrobotics/DQ.h>

#include <sas_robot_driver/sas_robot_driver.h>

using namespace DQ_robotics;
using namespace Eigen;

namespace sas
{
//Declared internally
class DriverBcap;
class RobotMutex;

struct RobotDriverDensoConfiguration
{
    std::string ip_address;
    int port;
    double speed;
    bool using_hand = false;  // for Cobotta robot, if using hand, we need to add a mutex to lock joint if hand is enabled
    std::string lock_name;
};


class RobotDriverDenso: public RobotDriver
{
private:
    RobotDriverDensoConfiguration configuration_;

    //BCAP driver
    std::unique_ptr<DriverBcap> bcap_driver_;

    //Robot Resource Mutex
    std::unique_ptr<RobotMutex> robot_resource_mutex_;

    //Joint positions
    VectorXd joint_positions_;
    VectorXd end_effector_pose_;
    std::vector<double> joint_positions_buffer_;
    std::vector<double> end_effector_pose_euler_buffer_;
    std::vector<double> end_effector_pose_homogenous_transformation_buffer_;

    DQ _homogenous_vector_to_dq(const VectorXd& homogenousvector) const;
    VectorXd _dq_to_homogenous_vector(const DQ& pose) const;
    VectorXd _get_end_effector_pose_homogenous_transformation();
    double _sign(const double &a) const;

    void _connect(); //Throws std::runtime_error()

    void _motor_on(); //Throws std::runtime_error()
    void _motor_off() noexcept; //No exceptions should be thrown in the path to turn off the robot

    void _set_speed(const float& speed, const float& acceleration, const float& deacceleration); //Throws std::runtime_error()

    void _slave_mode_on(int mode); //Throws std::runtime_error()
    void _slave_mode_off() noexcept; //No exceptions should be thrown in the path to turn off the robot

public:
    const static int SLAVE_MODE_JOINT_CONTROL;
    const static int SLAVE_MODE_END_EFFECTOR_CONTROL;

    RobotDriverDenso(const RobotDriverDenso&)=delete;
    RobotDriverDenso()=delete;
    ~RobotDriverDenso();

    RobotDriverDenso(const RobotDriverDensoConfiguration& configuration, std::atomic_bool* break_loops);

    VectorXd get_joint_positions() override;
    void set_target_joint_positions(const VectorXd& desired_joint_positions_rad) override;

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;

    bool set_end_effector_pose_dq(const DQ& pose);
    DQ get_end_effector_pose_dq();

    // Added 2024_08_21 by Quentin Lin for cobotta robot
    bool mutex_is_locked() const;
    bool mutex_acquire(const unsigned int &timeout_ms=0);
    void mutex_release();
};
}



