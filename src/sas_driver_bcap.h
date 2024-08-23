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
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################*/

#include <stdint.h>
#include "bcap/bCAPClient/bcap_client.h"

#include <map>
#include <string>
#include <sstream>
#include <vector>

#include <sys/socket.h>

//I tried using constants but ow lord how C++ is annoying some times
#define EMPTY_CHARP std::string("").c_str()

namespace sas
{

class DriverBcap
{
private:

    //Member variables
    //b-Cap communication
    std::string  server_ip_address_   ; //IP Address
    int          server_port_number_  ; //Port Number
    int          socket_;               //Socket (iSockFD in b-Cap.h)
    uint32_t     controller_handle_    ;//Controller Handle (lhController in b-Cap.h)
    uint32_t     robot_handle_;         //Robot Handle (lhRobot in b-Cap.h)
    long         socket_timeout_;

    //Pointer for default results store locally (if for instance the user is not interested them)
    long unwanted_result_;

    VARIANT unwanted_variant_result_;
    VARIANT empty_variant_;

    //Variable handle
    std::map<std::string, u_int>   controller_variable_handles_;

    //Common results stored for optimization
    double get_joint_positions_array_[8];
    double get_end_effector_pose_array_euler_[7];
    double get_end_effector_pose_array_homogenous_transformation_[10];

    VARIANT set_joint_positions_variant_;
    VARIANT set_end_effector_pose_variant_;

    HRESULT last_error_;
    std::string last_error_info_;

    bool _error_check(const HRESULT& result);
    inline bool _robot_execute(const std::wstring& command, const VARIANT& option, VARIANT& result);
    inline bool _controller_execute(const std::wstring& command, const VARIANT& option, VARIANT& result);
    HRESULT _set_socket_timeout(long micro_seconds);
    HRESULT _get_last_error() const;

public:
    DriverBcap()=delete;
    DriverBcap(const DriverBcap&)=delete;

    DriverBcap(std::string server_ip_address = std::string("") , const int server_port_number = 0, long socket_timeout = 2000000);

    ~DriverBcap();

    bool open();

    bool close();

    bool service_start();

    bool service_stop();

    bool controller_connect();

    bool controller_disconnect();

    bool get_robot();

    bool release_robot();

    bool get_controller_variable_handle(const std::wstring& variable_name, uint32_t& handle);

    bool get_controller_variable_int(const uint32_t& variable_handle, int& value_int);

    void initialize_controller_variable_handles();

    bool get_mode(int& mode);

    bool get_emergency_stop(int& status);

    bool get_controller_status(int& status);

    bool clear_error(VARIANT& result);
    inline bool clear_error() {return clear_error(unwanted_variant_result_);}

    bool take_arm(VARIANT& result);
    inline bool take_arm() {return take_arm(unwanted_variant_result_);}

    bool give_arm(VARIANT& result);
    inline bool give_arm() {return give_arm(unwanted_variant_result_);}

    bool set_motor_state(bool state, VARIANT& result);
    inline bool set_motor_state(bool state) {return set_motor_state(state,unwanted_variant_result_);}

    bool get_joint_positions(std::vector<double>& get_joint_positions_vector);

    bool get_end_effector_pose_homogenous_transformation(std::vector<double>& get_end_effector_pose_homogeouns_transformation_vector);

    bool set_speed(const float& speed, const float& acceleration, const float& deacceleration);

    bool set_and_get_joint_positions(const std::vector<double>& set_joint_positions_vector, std::vector<double>& get_joint_positions_vector);

    bool set_joint_positions(const std::vector<double>& set_joint_positions_vector, VARIANT& variant_result);
    inline bool set_joint_positions(std::vector<double>& set_joint_positions_vector) {return set_joint_positions(set_joint_positions_vector,unwanted_variant_result_);}

    bool set_end_effector_pose_homogenous_transformation(const std::vector<double>& set_end_effector_pose_vector, VARIANT& variant_result);
    inline bool set_end_effector_pose_homogenous_transformation(std::vector<double>& set_end_effector_pose_vector) {return set_end_effector_pose_homogenous_transformation(set_end_effector_pose_vector,unwanted_variant_result_);}

    bool set_slave_mode(const int32_t& value, VARIANT& result);
    inline bool set_slave_mode(long value) {return set_slave_mode(value,unwanted_variant_result_);}

    std::string get_last_error_info() const;


    // added by Hung-Ching Lin for gripper
    bool set_gripper_position(const unsigned char &position, const unsigned char &speed, VARIANT& result);
    inline bool set_gripper_position(const unsigned char &position, const unsigned char &speed) {return set_gripper_position(position, speed, unwanted_variant_result_);}

    bool get_gripper_is_busy(bool& is_busy);
    bool get_gripper_is_holding(bool& is_holding);
    bool get_gripper_in_position(bool& in_position);
    bool get_gripper_position(double& position);
    bool get_gripper_current_load(double& current_load);

};

}


