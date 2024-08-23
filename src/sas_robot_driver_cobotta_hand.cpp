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
#   Contributors: Hung-Ching Lin, email:qlin1806@g.ecc.u-tokyo.ac.jp
#       -- added robot hand for cobotta robot
#
# ################################################################*/

#include <sas_robot_driver_denso/sas_robot_driver_cobotta_hand.h>

#include "../../src/sas_driver_bcap.h"
#include "../../src/modules/robot_mutex.h"
#include "../../src/modules/robot_gripper_provider.h"

namespace sas {
    RobotDriverDensoHand::~RobotDriverDensoHand()=default;

    RobotDriverDensoHand::RobotDriverDensoHand(ros::NodeHandle& nh, const RobotDriverDensoHandConfiguration& configuration, std::atomic_bool* break_loops):
    nodehandle_(nh),
    configuration_(configuration),
    break_loops_(break_loops),
    gripper_resource_lock_(),
    bcap_driver_(new DriverBcap(configuration.ip_address, configuration.port)),
    robot_resource_mutex_(new RobotMutex(configuration.lock_name, "hand", false))
    {
        auto gripper_provider_configuration = CobottaGripperProviderConfiguration();
        gripper_provider_configuration.default_speed = configuration.default_speed;
        gripper_provider_configuration.topic_prefix = configuration.prefix;
        cobotta_gripper_provider_ = std::unique_ptr<CobottaGripperProvider>(new CobottaGripperProvider(nh, gripper_provider_configuration, &gripper_resource_lock_));

    }

    void RobotDriverDensoHand::connect() {
        bool worked; //bCap communication error code

        worked = bcap_driver_->open();
        if(!worked)
        {
            throw std::runtime_error("  FAILED TO Open() IN FUNCTION Connect(). Error code = " + bcap_driver_->get_last_error_info());
        }

        worked = bcap_driver_->service_start();
        if(!worked)
        {
            throw std::runtime_error("  FAILED TO ServiceStart() IN FUNCTION Connect(). Error code = " + bcap_driver_->get_last_error_info());
        }

        worked = bcap_driver_->controller_connect();
        if(!worked)
        {
            throw std::runtime_error("  FAILED TO ControllerConnect() IN FUNCTION Connect(). Error code = " + bcap_driver_->get_last_error_info());
        }

    }

    void RobotDriverDensoHand::initialize() {
        cobotta_gripper_provider_->register_move_function(std::bind(&RobotDriverDensoHand::blocking_move, this, std::placeholders::_1, std::placeholders::_2));
    }

    void RobotDriverDensoHand::deinitialize() const {
        cobotta_gripper_provider_->deregister_move_function();
    }

    void RobotDriverDensoHand::disconnect() const {
        bcap_driver_->controller_disconnect();
        bcap_driver_->service_stop();
        bcap_driver_->close();
    }

    void RobotDriverDensoHand::control_loop() {

        ros::Rate loop_rate(ROBOT_DRIVER_HAND_LOOP_RATE);

        ROS_INFO_STREAM("RobotDriverDensoHand::control_loop: Starting control loop.");
        while(!(*break_loops_)) {
            last_gripper_state_.position = desired_gripper_width_;
            /**
             * TODO: currently below function are not available in whe joint driver has arm in SlvMode
             */
//            robot_resource_mutex_->acquire();
//            try {
//                auto ret = _update_gripper_state();
//                if (!ret) {ROS_WARN_STREAM("RobotDriverDensoHand::control_loop: Failed to update gripper state.");}
//            }catch (std::exception &e) {
//                ROS_WARN_STREAM("RobotDriverDensoHand::control_loop: Failed to update gripper state. Error: " + std::string(e.what()));
//            }
//            robot_resource_mutex_->release();
            cobotta_gripper_provider_->send_gripper_state(
                    last_gripper_state_.position, last_gripper_state_.busy,
                    last_gripper_state_.holding, last_gripper_state_.in_position,
                    last_gripper_state_.current_load);
            ros::spinOnce();
            loop_rate.sleep();
        }
        ROS_INFO_STREAM("RobotDriverDensoHand::control_loop: Exiting control loop.");
    }

    /**
     * @brief update the gripper state from bcap
     * @return success
     */
    bool RobotDriverDensoHand::_update_gripper_state(){
        /**
         * position: HandCurPos: get_gripper_position
         * busy: HandBusyState: get_gripper_is_busy
         * holding: HandHoldState: get_gripper_is_holding
         * in_position: HandInposState: get_gripper_in_position
         * current_load: HandCurLoad: get_gripper_current_load
         */
        GripperStateStruct current_state = {false, false, false, 0, 0};
//        try {
//            if (!robot_resource_mutex_->acquire(4000)) {
//                throw std::runtime_error("  FAILED TO acquire() IN FUNCTION _update_gripper_state(). Attempted acquiring robot resource lock TIMEOUT.");
//            }
//        }catch (std::exception &e) {
//            throw std::runtime_error("  FAILED TO acquire() IN FUNCTION _update_gripper_state(). Error in acquiring robot resource lock: " + std::string(e.what()));
//        }
//        gripper_resource_lock_.lock();
        bool worked; //bCap communication error code

        worked = bcap_driver_->get_gripper_position(current_state.position);
        if(!worked)
        {
            ROS_WARN_STREAM("RobotDriverDensoHand::control_loop: Failed to get gripper position. Error code = " + bcap_driver_->get_last_error_info());
            return false;
        }else {
            // normalize the position
            current_state.position = (current_state.position - configuration_.gripper_range_min) / (configuration_.gripper_range_max - configuration_.gripper_range_min);
        }
        worked = bcap_driver_->get_gripper_is_busy(current_state.busy);
        if(!worked)
        {
            ROS_WARN_STREAM("RobotDriverDensoHand::control_loop: Failed to get gripper busy state. Error code = " + bcap_driver_->get_last_error_info());
            return false;
        }
        worked = bcap_driver_->get_gripper_is_holding(current_state.holding);
        if(!worked)
        {
            ROS_WARN_STREAM("RobotDriverDensoHand::control_loop: Failed to get gripper holding state. Error code = " + bcap_driver_->get_last_error_info());
            return false;
        }
        worked = bcap_driver_->get_gripper_in_position(current_state.in_position);
        if(!worked)
        {
            ROS_WARN_STREAM("RobotDriverDensoHand::control_loop: Failed to get gripper in position state. Error code = " + bcap_driver_->get_last_error_info());
            return false;
        }
        worked = bcap_driver_->get_gripper_current_load(current_state.current_load);
        if(!worked)
        {
            ROS_WARN_STREAM("RobotDriverDensoHand::control_loop: Failed to get gripper current load. Error code = " + bcap_driver_->get_last_error_info());
            return false;
        }

//        try { robot_resource_mutex_->release(); }catch(...){}
//        gripper_resource_lock_.unlock();

        last_gripper_state_ = current_state;
        return true;
    }

    bool RobotDriverDensoHand::blocking_move(const double& width, const double& speed_ratio) {
        if(0>width || width>1)
        {
            ROS_WARN_STREAM("RobotDriverDensoHand::blocking_move: Requested width is out of range. Clipping to range [0,1].");
        }
        const auto _width = _clip(width, 0.0, 1.0);
        const auto _speed = _clip(speed_ratio, ROBOT_DRIVER_GRIPPER_SPEED_MIN_CAP, 1.0);
        auto pos_double = _width*(configuration_.gripper_range_max-configuration_.gripper_range_min)+configuration_.gripper_range_min;
        auto position = static_cast<unsigned char>(pos_double);
        auto speed = static_cast<unsigned char>(_speed*ROBOT_DRIVER_GRIPPER_SPEED_SCALING);

        try {
            auto ret = robot_resource_mutex_->acquire();
            if (!ret) {
                throw std::runtime_error("  FAILED TO acquire() IN FUNCTION blocking_move(). Attempted acquiring robot resource lock TIMEOUT.");
            }
        }catch (std::exception &e) {
            throw std::runtime_error("  FAILED TO acquire() IN FUNCTION blocking_move(). Error in acquiring robot resource lock: " + std::string(e.what()));
        }
        bool worked; //bCap communication error code

        worked = bcap_driver_->set_gripper_position(position, speed);
        if(!worked)
        {
            robot_resource_mutex_->release();
            throw std::runtime_error("  FAILED TO set_gripper_position() IN FUNCTION blocking_move(). Error code = " + bcap_driver_->get_last_error_info());
        }
        desired_gripper_width_ = width;

        // TODO: function call to bcap appear to be blocking already

        robot_resource_mutex_->release();
        return true;
    }



}