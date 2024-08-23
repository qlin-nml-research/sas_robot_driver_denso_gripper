#pragma once
/*
# Copyright (c) 2024 Hung-Ching Lin
#
#    This file is part of sas_robot_driver_dense.
#
#    sas_robot_driver_dense is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver_dense is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver_dense.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Hung-Ching Lin, email: qlin1806@g.ecc.u-tokyo.ac.jp
#
#   - Added to sas_robot_driver_dense for
#        gripper control and some other functions for cobotta robot
# ################################################################*/

#include <sas_robot_driver_denso/GripperState.h>
#include <sas_robot_driver_denso/Move.h>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <functional>
#include <utility>
#include <chrono>
#include <thread>

#define MOVE_SERVICE_SUFFIX "/action/move"
#define STATUS_TOPIC_SUFFIX "/get/gripper_state"
#define MOVE_TIMEOUT_MS 3000

namespace sas {
    typedef sas_robot_driver_denso::Move::Request MoveRequest_t;
    typedef sas_robot_driver_denso::Move::Response MoveResponse_t;
    typedef sas_robot_driver_denso::GripperState GripperState_t;

    struct CobottaGripperProviderConfiguration
    {
        double default_speed = 1.0;
        std::string topic_prefix;
    };

    class CobottaGripperProvider {
    private:
        CobottaGripperProviderConfiguration configuration_;
        ros::NodeHandle nh_;

        std::function<bool(const double&, const double&)> gripper_move_function_;

        std::mutex *gripper_in_use_ptr_;
        ros::ServiceServer move_server_;

        ros::Publisher gripper_status_publisher_;

        static inline double _clip(const double &n, const double &lower, const double &upper) {
            return std::max(lower, std::min(n, upper));
        }

    public:
        CobottaGripperProvider(const CobottaGripperProvider&)=delete;
        CobottaGripperProvider()=delete;
        CobottaGripperProvider(ros::NodeHandle &nh, const CobottaGripperProviderConfiguration &configuration, std::mutex *resource_ptr);

        ~CobottaGripperProvider()=default;

        void register_move_function(std::function<bool(const double&, const double&)> move_function);

        void deregister_move_function();

        void send_gripper_state(const double &pos, const bool &busy, const bool &holding, const bool &in_position, const double &current_load) const;


    protected:
        bool _srv_move_callback(MoveRequest_t &req, MoveResponse_t &res);


    };

}

