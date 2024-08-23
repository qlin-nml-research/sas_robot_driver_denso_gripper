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
#include "robot_gripper_provider.h"



namespace sas {
    CobottaGripperProvider::CobottaGripperProvider(ros::NodeHandle &nh, const CobottaGripperProviderConfiguration &configuration, std::mutex *resource_ptr):
        nh_(nh),
        configuration_(configuration),
        gripper_move_function_(nullptr),
        gripper_in_use_ptr_(resource_ptr)
    {
        move_server_ = nh_.advertiseService(configuration_.topic_prefix+MOVE_SERVICE_SUFFIX, &CobottaGripperProvider::_srv_move_callback, this);
        gripper_status_publisher_ = nh_.advertise<GripperState_t>(configuration_.topic_prefix+STATUS_TOPIC_SUFFIX, 1);

    }

    void CobottaGripperProvider::send_gripper_state(const double &pos, const bool &busy, const bool &holding, const bool &in_position, const double &current_load) const {
        GripperState_t msg;
        msg.busy = busy;
        msg.holding= holding;
        msg.in_position = in_position;
        msg.current_load = static_cast<float>(current_load);
        msg.position = static_cast<float>(pos);
        gripper_status_publisher_.publish(msg);
    }

    void CobottaGripperProvider::register_move_function(std::function<bool(const double&, const double&)> move_function) {
        gripper_move_function_ = std::move(move_function);
    }

    void CobottaGripperProvider::deregister_move_function() {
        gripper_move_function_ = nullptr;
    }


    bool CobottaGripperProvider::_srv_move_callback(MoveRequest_t &req, MoveResponse_t &res) {
        double speed = req.speed; // between 0 and 1
        double width = req.width; // between 0 and 1
        if(speed <= 0) {
            speed = configuration_.default_speed;
        }
        speed = _clip(speed, 0.1, 1);
        width = _clip(width, 0, 1);

        if(gripper_move_function_==nullptr) {
            ROS_WARN_STREAM("CobottaGripperProvider::_move_cb: Gripper move function not set.");
            res.success = false;
            return true;
        }
        ROS_INFO_STREAM("CobottaGripperProvider::Service Callback: Requested to move gripper to position: " << width << " with speed: " << speed);
        auto timeout = std::chrono::steady_clock::now() + std::chrono::milliseconds(MOVE_TIMEOUT_MS);
        bool lock_ret = false;
        while (std::chrono::steady_clock::now()<timeout) {
            lock_ret = gripper_in_use_ptr_->try_lock();
            if(lock_ret) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if(!lock_ret) {
            ROS_INFO_STREAM("CobottaGripperProvider::_move_cb: resource timeout, Gripper is in use.");
            res.success = false;
            return true;
        }
        try {
            auto ret = gripper_move_function_(width, speed);
            res.success = ret;

        }catch(std::exception &e) {
            ROS_ERROR_STREAM("CobottaGripperProvider::_move_cb: Exception caught: " << e.what());
            res.success = false;
        }catch (...) {
            ROS_ERROR_STREAM("CobottaGripperProvider::_move_cb: Unknown exception caught.");
            res.success = false;
        }

        gripper_in_use_ptr_->unlock();
        return true;
    }
}


