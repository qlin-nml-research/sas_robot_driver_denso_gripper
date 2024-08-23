#pragma once
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
#   Author: Hung-Ching Lin, email:qlin1806@g.ecc.u-tokyo.ac.jp
#       -- added robot hand for cobotta robot
#
# ################################################################*/
#include <exception>
#include <tuple>
#include <atomic>
#include <vector>
#include <memory>
#include <mutex>
#include <ros/ros.h>
// #include <sas_clock/sas_clock.h>

#define ROBOT_DRIVER_HAND_LOOP_RATE 10  //100ms
#define ROBOT_DRIVER_GRIPPER_SPEED_SCALING 100.0   // float in percentage to 0-100


namespace sas {
    //Declared internally
    class DriverBcap;
    class RobotMutex;
    class CobottaGripperProvider;

    struct RobotDriverDensoHandConfiguration
    {
        std::string prefix;
        std::string ip_address;
        int port;
        double default_speed;
        std::string lock_name;
        int gripper_range_min;
        int gripper_range_max;
    };


    class RobotDriverDensoHand {
    private:
        ros::NodeHandle& nodehandle_;
        const RobotDriverDensoHandConfiguration configuration_;
        std::atomic_bool* break_loops_;
        std::mutex gripper_resource_lock_;

        double last_gripper_width_ = 0;
        double desired_gripper_width_ = 0;

        //BCAP driver
        std::unique_ptr<DriverBcap> bcap_driver_;

        //Robot Resource Mutex
        std::unique_ptr<RobotMutex> robot_resource_mutex_;

        std::unique_ptr<CobottaGripperProvider> cobotta_gripper_provider_;

        static inline double _clip(const double &n, const double &lower, const double &upper) {
            return std::max(lower, std::min(n, upper));
        }

    public:
        RobotDriverDensoHand(const RobotDriverDensoHand&)=delete;
        RobotDriverDensoHand()=delete;
        ~RobotDriverDensoHand();

        RobotDriverDensoHand(ros::NodeHandle& nh, const RobotDriverDensoHandConfiguration& configuration, std::atomic_bool* break_loops);

        void connect();

        void initialize();

        void control_loop();

        void deinitialize() const;

        void disconnect() const;

        bool blocking_move(const double& width, const double& speed_ratio);

    };



}
